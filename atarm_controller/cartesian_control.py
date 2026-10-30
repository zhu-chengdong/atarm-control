"""3D position control with simulation and explicit calibrated FD I/O contracts.

The controlled point is the metric CameraPose child_frame. Three virtual
actuation coordinates map to four cable-length channels. Defaults and limits
are simulation assumptions, NOT calibrated arm parameters. The supplied I/O
is simulation-only unless the calibrated FD adapter contract is supplied.
"""
from collections import deque
from dataclasses import dataclass, asdict
from statistics import median
import math
import threading

from .camera_feedback import CameraPose, _valid_geometry
from .cartesian_math import dot, norm, sub, mv, gram, least_squares, singular_values, mix_matrix, quaternion_angle


def finite_vector(x, n):
    return len(x) == n and all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in x)


@dataclass(frozen=True)
class PositionGoal:
    position_m: tuple
    parent_frame: str
    child_frame: str
    map_epoch: str
    # An optional *constraint*, never an optimized six-DOF orientation target.
    orientation_reference: tuple | None = None
    orientation_limit_rad: float | None = None

    def __post_init__(self):
        object.__setattr__(self, 'position_m', tuple(self.position_m))
        if self.orientation_reference is not None:
            object.__setattr__(self, 'orientation_reference', tuple(self.orientation_reference))
        if not finite_vector(self.position_m, 3) or not all(isinstance(x, str) and x for x in self.key):
            raise ValueError('CARTESIAN_INVALID_GOAL')
        if (self.orientation_reference is None) != (self.orientation_limit_rad is None):
            raise ValueError('CARTESIAN_ORIENTATION_POLICY_INCOMPLETE')
        if self.orientation_reference is not None:
            if not finite_vector(self.orientation_reference, 4) or abs(dot(self.orientation_reference, self.orientation_reference)-1) > .01:
                raise ValueError('CARTESIAN_ORIENTATION_INVALID')
            if not math.isfinite(self.orientation_limit_rad) or not 0 < self.orientation_limit_rad <= math.pi:
                raise ValueError('CARTESIAN_ORIENTATION_LIMIT_INVALID')

    @property
    def key(self):
        return self.parent_frame, self.child_frame, self.map_epoch


@dataclass(frozen=True)
class MotorEvidence:
    positions_m: tuple
    frame_ids: tuple
    captured_s: tuple
    valid: tuple


@dataclass(frozen=True)
class Observation:
    pose: CameraPose
    motors: MotorEvidence


@dataclass(frozen=True)
class CartesianConfig:
    initial_jacobian: tuple
    cable_mix: tuple
    reference_lengths_m: tuple
    min_lengths_m: tuple
    max_lengths_m: tuple
    strategy: str = 'alternating'
    position_tolerance_m: float = .0005
    camera_noise_bound_m: float = .00006
    max_pose_age_s: float = .150
    max_motor_age_s: float = .100
    stable_s: float = .100
    settle_s: float = .050
    poll_s: float = .010
    max_speed_m_s: float = .6
    pose_step_margin_m: float = .0003
    motor_tolerance_m: float = .00010
    max_motor_step_m: float = .0025
    min_step_m: float = .00001
    max_total_travel_m: float = .6
    max_model_radius_m: float = .10
    max_steps: int = 100
    timeout_s: float = 40.
    settle_timeout_s: float = 1.
    gain: float = .75
    damping: float = .06
    min_singular_value: float = .15
    max_condition: float = 15.
    no_progress_steps: int = 8
    min_progress_m: float = .000015
    filter_frames: int = 5
    online_update: bool = True
    learning_rate: float = .65
    min_learning_step_m: float = .00015
    max_pair_s: float = 2.
    max_update_fraction: float = .25

    def __post_init__(self):
        for key, rows, cols in (('initial_jacobian', 3, 3), ('cable_mix', 3, 2)):
            a = tuple(tuple(row) for row in getattr(self, key))
            if len(a) != rows or not all(finite_vector(row, cols) for row in a):
                raise ValueError('CARTESIAN_INVALID_'+key.upper())
            object.__setattr__(self, key, a)
        for key in ('reference_lengths_m', 'min_lengths_m', 'max_lengths_m'):
            v = tuple(getattr(self, key)); object.__setattr__(self, key, v)
            if not finite_vector(v, 4):
                raise ValueError('CARTESIAN_INVALID_'+key.upper())
        if any(not lo < hi for lo, hi in zip(self.min_lengths_m, self.max_lengths_m)):
            raise ValueError('CARTESIAN_INVALID_LIMITS')
        if any(not lo <= q <= hi for lo, q, hi in zip(self.min_lengths_m, self.reference_lengths_m, self.max_lengths_m)):
            raise ValueError('CARTESIAN_REFERENCE_OUT_OF_LIMITS')
        if self.strategy not in ('alternating', 'coordinated'):
            raise ValueError('CARTESIAN_INVALID_STRATEGY')
        integer = ('max_steps', 'no_progress_steps', 'filter_frames')
        for key, value in asdict(self).items():
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                if not math.isfinite(value) or value <= 0:
                    raise ValueError('CARTESIAN_INVALID_'+key.upper())
        for key in integer:
            if not isinstance(getattr(self, key), int) or isinstance(getattr(self, key), bool):
                raise ValueError('CARTESIAN_INVALID_COUNT')
        if self.filter_frames < 3 or self.filter_frames % 2 != 1 or self.camera_noise_bound_m >= self.position_tolerance_m:
            raise ValueError('CARTESIAN_INVALID_OBSERVATION_BUDGET')
        if not 0 < self.gain <= 1 or not 0 < self.learning_rate <= 1:
            raise ValueError('CARTESIAN_INVALID_GAIN')
        if not isinstance(self.online_update, bool):
            raise ValueError('CARTESIAN_INVALID_LEARNING_POLICY')
        if any(abs(sum(row[i] for row in self.cable_mix)) > 1e-9 for i in (0, 1)) or singular_values(self.cable_mix)[0] < 1e-8:
            raise ValueError('CARTESIAN_INVALID_CABLE_MIX')
        lo, hi = singular_values(self.initial_jacobian)[::2]
        if lo < self.min_singular_value or hi/lo > self.max_condition:
            raise ValueError('CARTESIAN_MODEL_RANK_OR_CONDITION')


class CartesianController:
    def __init__(self, config, io):
        if not (getattr(io, 'execution_domain', None) == 'simulation-only' or
                (getattr(io, 'execution_domain', None) == 'fd-actuator' and
                 getattr(io, 'closed_loop_io_contract', None) == 'calibrated-a1-v1')):
            raise ValueError('CARTESIAN_PHYSICAL_IO_DISABLED')
        self.cfg, self.io = config, io
        self.mix = mix_matrix(config.cable_mix)
        self.jacobian = config.initial_jacobian
        self.lock = threading.Lock(); self.state_lock = threading.RLock()
        self.cancel_event = threading.Event(); self.active = False
        self.fault = None; self.reference = None; self.last_pose = None; self.last_motor = None
        self.history = []; self.updates = []; self.coverage = [[0.]*3 for _ in range(3)]
        self.result = {'state': 'IDLE', 'success': False, 'complete': False}

    def cancel(self):
        with self.state_lock:
            if not self.active or self.result.get('state') != 'RUNNING':
                return False
            self.cancel_event.set()
            return True

    def check(self):
        if self.cancel_event.is_set() or getattr(self.io, 'cancel_requested', lambda: False)():
            raise ValueError('CARTESIAN_CANCELLED')
        if self.io.now()-self.started > self.cfg.timeout_s:
            raise ValueError('CARTESIAN_TIME_LIMIT')

    def read(self, goal):
        self.check()
        obs = self.io.read(); now = self.io.now(); p, m = obs.pose, obs.motors
        if not isinstance(p, CameraPose) or not _valid_geometry(p):
            raise ValueError('CARTESIAN_POSE_INVALID')
        if not p.tracking_ok:
            raise ValueError('CARTESIAN_TRACKING_LOST')
        if (p.parent_frame, p.child_frame, p.map_epoch) != goal.key:
            raise ValueError('CARTESIAN_REFERENCE_CHANGED')
        if not isinstance(p.frame_id, int) or isinstance(p.frame_id, bool) or p.frame_id < 0:
            raise ValueError('CARTESIAN_POSE_FRAME_INVALID')
        if not math.isfinite(p.capture_s) or not 0 <= now-p.capture_s <= self.cfg.max_pose_age_s:
            raise ValueError('CARTESIAN_POSE_STALE_OR_FUTURE')
        old = self.last_pose
        if old is not None:
            if p.frame_id < old.frame_id or (p.frame_id == old.frame_id and p != old):
                raise ValueError('CARTESIAN_POSE_REPLAY_OR_MUTATION')
            if p.frame_id > old.frame_id:
                dt = p.capture_s-old.capture_s
                if dt <= 0 or norm(sub(p.position_m, old.position_m)) > self.cfg.max_speed_m_s*dt+self.cfg.pose_step_margin_m:
                    raise ValueError('CARTESIAN_POSE_DISCONTINUITY')
        if not finite_vector(m.positions_m, 4) or any(len(x) != 4 for x in (m.frame_ids, m.captured_s, m.valid)):
            raise ValueError('CARTESIAN_MOTOR_EVIDENCE_INVALID')
        if not all(v is True for v in m.valid):
            raise ValueError('CARTESIAN_MOTOR_FEEDBACK_INVALID')
        for i in range(4):
            if not isinstance(m.frame_ids[i], int) or isinstance(m.frame_ids[i], bool) or m.frame_ids[i] < 0:
                raise ValueError('CARTESIAN_MOTOR_FRAME_INVALID')
            if not math.isfinite(m.captured_s[i]) or not 0 <= now-m.captured_s[i] <= self.cfg.max_motor_age_s:
                raise ValueError('CARTESIAN_MOTOR_FEEDBACK_STALE')
            if self.last_motor is not None:
                old_m = self.last_motor
                if m.frame_ids[i] < old_m.frame_ids[i] or (m.frame_ids[i] == old_m.frame_ids[i] and (m.positions_m[i], m.captured_s[i]) != (old_m.positions_m[i], old_m.captured_s[i])):
                    raise ValueError('CARTESIAN_MOTOR_REPLAY_OR_MUTATION')
                if m.frame_ids[i] > old_m.frame_ids[i] and m.captured_s[i] <= old_m.captured_s[i]:
                    raise ValueError('CARTESIAN_MOTOR_TIME_NOT_ADVANCING')
            if not self.cfg.min_lengths_m[i] <= m.positions_m[i] <= self.cfg.max_lengths_m[i]:
                raise ValueError('CARTESIAN_MEASURED_LIMIT')
        displacement = sub(m.positions_m, self.cfg.reference_lengths_m)
        coordinates = least_squares(self.mix, displacement)
        if norm(sub(displacement, mv(self.mix, coordinates))) > self.cfg.motor_tolerance_m:
            raise ValueError('CARTESIAN_ACTUATION_OUTSIDE_CONTROL_SUBSPACE')
        if norm(coordinates) > self.cfg.max_model_radius_m:
            raise ValueError('CARTESIAN_MODEL_VALIDITY_LIMIT')
        if goal.orientation_reference is not None and quaternion_angle(p.quaternion_xyzw, goal.orientation_reference) > goal.orientation_limit_rad:
            raise ValueError('CARTESIAN_ORIENTATION_CONSTRAINT')
        self.last_pose, self.last_motor = p, m
        return obs

    def sample(self, goal, target=None):
        """Stable, fresh pose window acquired AFTER all four motors settle."""
        began = self.io.now(); settled = None; window = deque()
        last_id = None; pose = None
        while self.io.now()-began <= self.cfg.settle_timeout_s:
            obs = self.read(goal)
            q = obs.motors.positions_m
            if target is None:
                target = q
            at_target = max(abs(a-b) for a, b in zip(q, target)) <= self.cfg.motor_tolerance_m
            if not at_target:
                settled = None; window.clear()
            elif settled is None:
                settled = self.io.now()
            if settled is not None and obs.pose.capture_s >= settled+self.cfg.settle_s and obs.pose.frame_id != last_id:
                last_id = obs.pose.frame_id; pose = obs.pose
                window.append(obs)
                if len(window) >= self.cfg.filter_frames and window[-1].pose.capture_s-window[0].pose.capture_s >= self.cfg.stable_s:
                    p = tuple(median(o.pose.position_m[i] for o in window) for i in range(3))
                    q = tuple(median(o.motors.positions_m[i] for o in window) for i in range(4))
                    spread = max(norm(sub(o.pose.position_m, p)) for o in window)
                    if spread > 2*self.cfg.camera_noise_bound_m:
                        raise ValueError('CARTESIAN_POSE_WINDOW_UNSTABLE')
                    return {'p': p, 'q': q, 'first_frame': window[0].pose.frame_id, 'last_frame': pose.frame_id,
                            'first_s': window[0].pose.capture_s, 'last_s': pose.capture_s, 'spread_m': spread,
                            'max_goal_error_m': max(norm(sub(o.pose.position_m, goal.position_m)) for o in window)}
            self.io.advance(self.cfg.poll_s)
        raise ValueError('CARTESIAN_SETTLE_TIMEOUT')

    def candidates(self, error, q):
        c = self.cfg
        lo, hi = singular_values(self.jacobian)[::2]
        if lo < c.min_singular_value or hi/lo > c.max_condition:
            raise ValueError('CARTESIAN_MODEL_RANK_OR_CONDITION')
        groups = [('coordinated', (0, 1, 2))] if c.strategy == 'coordinated' else [('bend', (0, 1)), ('extend', (2,))]
        candidates = []
        for label, axes in groups:
            a = tuple(tuple(row[i] for i in axes) for row in self.jacobian)
            answer = least_squares(a, error, c.damping)
            du = [0.]*3
            for i, v in zip(axes, answer):
                du[i] = c.gain*v
            dq = mv(self.mix, du)
            scale = min(1., c.max_motor_step_m/max(max(abs(v) for v in dq), 1e-12))
            for x, v, low, high in zip(q, dq, c.min_lengths_m, c.max_lengths_m):
                if v > 0:
                    scale = min(scale, max(0., (high-x)/v))
                elif v < 0:
                    scale = min(scale, max(0., (low-x)/v))
            du = tuple(v*scale for v in du); dq = mv(self.mix, du)
            if max(abs(v) for v in dq) < c.min_step_m:
                continue
            target = tuple(x+v for x, v in zip(q, dq))
            if any(not low-1e-12 <= x <= high+1e-12 for low, x, high in zip(c.min_lengths_m, target, c.max_lengths_m)):
                continue
            u = least_squares(self.mix, sub(target, c.reference_lengths_m))
            if norm(u) > c.max_model_radius_m:
                continue
            reduction = norm(error)-norm(sub(error, mv(self.jacobian, du)))
            if reduction > 1e-10:
                candidates.append((reduction, label, du, dq, target))
        if not candidates:
            raise ValueError('CARTESIAN_NO_FEASIBLE_DESCENT_STEP')
        return max(candidates, key=lambda x: x[0])

    def learn(self, before, after):
        c = self.cfg; dq = sub(after['q'], before['q']); du = least_squares(self.mix, dq)
        dp = sub(after['p'], before['p'])
        event = {'du': du, 'dp': dp, 'accepted': False, 'reason': None}
        self.updates.append(event)
        def reject(reason):
            event['reason'] = reason
        if after['first_frame'] <= before['last_frame'] or after['first_s'] <= before['last_s'] or after['last_s']-before['last_s'] > c.max_pair_s:
            return reject('INVALID_PAIR_TIME')
        if norm(sub(dq, mv(self.mix, du))) > c.motor_tolerance_m:
            return reject('ACTUATION_OUTSIDE_CONTROL_SUBSPACE')
        if norm(du) < c.min_learning_step_m:
            return reject('INSUFFICIENT_EXCITATION')
        direction = tuple(v/norm(du) for v in du)
        for i in range(3):
            for j in range(3):
                self.coverage[i][j] += direction[i]*direction[j]
        if not c.online_update:
            return reject('LEARNING_DISABLED')
        if norm(dp) < 4*c.camera_noise_bound_m:
            return reject('NOISE_DOMINATED_TRANSLATION')
        innovation = sub(dp, mv(self.jacobian, du))
        if norm(innovation) <= 2*c.camera_noise_bound_m:
            return reject('MODEL_CONSISTENT_WITH_NOISE')
        # Never learn an unobserved direction reversal as a successful calibration.
        if dot(mv(self.jacobian, du), dp) <= 0:
            return reject('DIRECTION_INCONSISTENT')
        delta = tuple(tuple(c.learning_rate*e*v/(dot(du, du)+c.min_learning_step_m**2) for v in du) for e in innovation)
        size = norm(tuple(v for row in delta for v in row)); cap = c.max_update_fraction*norm(tuple(v for row in self.jacobian for v in row))
        scale = min(1., cap/max(size, 1e-12))
        candidate = tuple(tuple(x+scale*y for x, y in zip(row, change)) for row, change in zip(self.jacobian, delta))
        lo, hi = singular_values(candidate)[::2]
        if lo < c.min_singular_value or hi/lo > c.max_condition:
            return reject('UPDATED_MODEL_RANK_OR_CONDITION')
        self.jacobian = candidate; event.update(accepted=True, reason='ACCEPTED')

    def move_position(self, goal):
        if not self.lock.acquire(blocking=False):
            return {'success': False, 'complete': False, 'state': 'REJECTED', 'reason': 'CARTESIAN_BUSY'}
        try:
            if self.fault:
                return {'success': False, 'complete': False, 'state': 'REJECTED', 'reason': 'CARTESIAN_EXPLICIT_RECOVERY_REQUIRED'}
            if not isinstance(goal, PositionGoal):
                return {'success': False, 'complete': False, 'state': 'REJECTED', 'reason': 'CARTESIAN_INVALID_GOAL'}
            if self.reference is not None and goal.key != self.reference:
                return {'success': False, 'complete': False, 'state': 'REJECTED', 'reason': 'CARTESIAN_NEW_REFERENCE_SESSION_REQUIRED'}
            self.reference = goal.key; self.started = self.io.now()
            self.history = []; self.updates = []; self.total = 0.
            with self.state_lock:
                self.cancel_event.clear(); self.active = True
                self.result = {'success': False, 'complete': False, 'state': 'RUNNING'}
            before = self.sample(goal); best = norm(sub(goal.position_m, before['p'])); no_progress = 0
            for _ in range(self.cfg.max_steps+1):
                self.check(); error = sub(goal.position_m, before['p']); magnitude = norm(error)
                if magnitude+self.cfg.camera_noise_bound_m <= self.cfg.position_tolerance_m and before['max_goal_error_m'] <= self.cfg.position_tolerance_m:
                    # Independent additional post-motion window, never a sub-action success.
                    verified = self.sample(goal, before['q'])
                    if norm(sub(goal.position_m, verified['p']))+self.cfg.camera_noise_bound_m <= self.cfg.position_tolerance_m and verified['max_goal_error_m'] <= self.cfg.position_tolerance_m:
                        with self.state_lock:
                            self.check()
                            self.result = {'success': True, 'complete': True, 'state': 'SUCCEEDED', 'reason': 'CARTESIAN_POSITION_CONFIRMED',
                                           'measured_error_m': norm(sub(goal.position_m, verified['p'])), 'verification': verified}
                            self.active = False
                        break
                    before = verified; continue
                if len(self.history) >= self.cfg.max_steps:
                    raise ValueError('CARTESIAN_ITERATION_LIMIT')
                _, label, du, dq, target = self.candidates(error, before['q'])
                travel = sum(abs(v) for v in dq)
                if self.total+travel > self.cfg.max_total_travel_m:
                    raise ValueError('CARTESIAN_TOTAL_TRAVEL_LIMIT')
                self.total += travel
                with self.state_lock:
                    self.check()
                    if self.io.command(target) is not True:
                        raise ValueError('CARTESIAN_COMMAND_REJECTED')
                event = {'block': label, 'from_q': before['q'], 'target_q': target, 'du': du, 'error_before_m': magnitude,
                         'at_s': self.io.now(), 'jacobian': self.jacobian}
                self.history.append(event)
                after = self.sample(goal, target)
                event.update({'error_after_m': norm(sub(goal.position_m, after['p'])), 'observed': after})
                self.learn(before, after)
                remaining = norm(sub(goal.position_m, after['p']))
                if best-remaining >= self.cfg.min_progress_m:
                    best = remaining; no_progress = 0
                else:
                    no_progress += 1
                if no_progress >= self.cfg.no_progress_steps:
                    raise ValueError('CARTESIAN_NO_PROGRESS')
                before = after
            return self.result
        except Exception as exc:
            self.fault = str(exc)
            try:
                receipts = self.io.stop()
            except Exception as stop_exc:
                receipts = [{'reply_valid': False, 'error': str(stop_exc)}]
            self.result = {'success': False, 'complete': False, 'state': 'CANCELLED' if self.cancel_event.is_set() else 'FAULT',
                           'reason': str(exc), 'stop_receipts': receipts}
            return self.result
        finally:
            with self.state_lock:
                self.active = False
            self.lock.release()

    def status(self):
        return {'execution_domain': self.io.execution_domain, 'result': self.result, 'history': self.history, 'updates': self.updates,
                'jacobian': self.jacobian, 'excitation_gram': self.coverage,
                'full_hardware_calibration_claimed': False, 'limits': asdict(self.cfg)}
