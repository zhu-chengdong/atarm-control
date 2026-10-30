"""Single-owner asynchronous jobs for the simulation-only Cartesian controller."""
from copy import deepcopy
from dataclasses import replace
import json
import math
import threading
import time

from .cartesian_control import CartesianController, PositionGoal, Observation
from .cartesian_simulation import SimulatedPlant, simulation_config
from .cartesian_math import norm, sub


class JobPlant(SimulatedPlant):
    def __init__(self, owner, epoch, *, fault=None, speed=4., paced=True):
        super().__init__(fault=fault)
        self.owner = owner; self.epoch = epoch; self.wall_speed = speed; self.paced = paced
        self.io_threads = set(); self.stop_wall_ns = None

    def checkpoint(self):
        self.io_threads.add(threading.current_thread().name)
        if self.owner.cancel_event.is_set() or self.owner.quit_event.is_set():
            raise ValueError('ROS_CARTESIAN_CANCELLED')

    def read(self):
        self.checkpoint()
        observation = super().read()
        epoch = self.epoch if observation.pose.map_epoch == 'sim_map_0' else self.epoch+'_reset'
        observation = Observation(replace(observation.pose, map_epoch=epoch), observation.motors)
        self.owner.observe(observation)
        return observation

    def command(self, target):
        self.checkpoint()
        return super().command(target)

    def advance(self, dt):
        self.checkpoint()
        if self.paced:
            self.owner.cancel_event.wait(dt/self.wall_speed)
        self.checkpoint()
        return super().advance(dt)

    def stop(self):
        self.io_threads.add(threading.current_thread().name)
        self.stop_wall_ns = time.perf_counter_ns()
        return super().stop()


class CartesianJobRunner:
    def __init__(self, *, strategy='alternating', simulation_speed=4., fault=None, paced=True):
        allowed_faults = {None, '', 'tracking_lost', 'frozen_pose', 'map_reset', 'motor_invalid', 'motor_stale', 'motor_stall'}
        if fault not in allowed_faults:
            raise ValueError('ROS_SIM_UNKNOWN_FAULT')
        if isinstance(simulation_speed, bool) or not math.isfinite(simulation_speed) or not 0 < simulation_speed <= 20:
            raise ValueError('ROS_SIM_INVALID_SPEED')
        self.config = simulation_config(strategy=strategy)
        self.lock = threading.RLock(); self.wake = threading.Event()
        self.cancel_event = threading.Event(); self.quit_event = threading.Event()
        self.speed = simulation_speed; self.paced = paced; self.fault = fault or None
        self.session = 0; self.counter = 0; self.job = None; self.seen = set(); self.needs_reset = False
        self.cached_pose = None; self.history = []; self.last = {'state': 'IDLE', 'complete': False}
        self.new_session()
        self.thread = threading.Thread(target=self.run, name='cartesian-sim-owner', daemon=True)
        self.thread.start()

    @property
    def reference(self):
        return ('sim_base', 'camera_optical', 'sim_map_'+str(self.session))

    def new_session(self):
        self.plant = JobPlant(self, self.reference[2], fault=self.fault, speed=self.speed, paced=self.paced)
        self.controller = CartesianController(self.config, self.plant)
        self.cached_pose = None; self.needs_reset = False; self.cancel_event.clear()

    def response(self, request_id, accepted, reason, **extra):
        return {'request_id': request_id, 'accepted': accepted, 'success': accepted, 'complete': False,
                'reason': reason, 'execution_domain': 'simulation-only', **extra}

    def submit_json(self, payload):
        request_id = None
        try:
            # Parse the correlation ID before validating geometry. PositionGoal
            # rejects non-finite numeric fields, retaining the request ID in the
            # negative acknowledgment instead of leaving the client waiting.
            d = json.loads(payload)
            if not isinstance(d, dict):
                raise ValueError('Goal must be an object')
            request_id = d.get('request_id')
            if not isinstance(request_id, str) or not 1 <= len(request_id) <= 64:
                request_id = None
                raise ValueError('Invalid request_id')
            allowed = {'request_id', 'position_m', 'parent_frame', 'child_frame', 'map_epoch', 'orientation_reference', 'orientation_limit_rad'}
            if set(d)-allowed:
                raise ValueError('Unknown goal fields')
            g = PositionGoal(**{k: v for k, v in d.items() if k != 'request_id'})
        except (ValueError, TypeError, KeyError) as exc:
            return self.response(request_id, False, 'ROS_CARTESIAN_INVALID_GOAL', detail=str(exc))
        with self.lock:
            if self.quit_event.is_set():
                return self.response(request_id, False, 'ROS_CARTESIAN_SHUTTING_DOWN')
            if request_id in self.seen:
                return self.response(request_id, False, 'ROS_CARTESIAN_DUPLICATE_ID')
            if g.key != self.reference:
                return self.response(request_id, False, 'ROS_CARTESIAN_REFERENCE_MISMATCH')
            if self.job is not None:
                return self.response(request_id, False, 'ROS_CARTESIAN_BUSY')
            if self.needs_reset:
                return self.response(request_id, False, 'ROS_CARTESIAN_RESET_REQUIRED')
            if len(self.seen) >= 256:
                return self.response(request_id, False, 'ROS_CARTESIAN_REQUEST_LIMIT_RESTART_REQUIRED')
            self.seen.add(request_id); self.counter += 1; self.cancel_event.clear()
            self.job = {'id': self.counter, 'request_id': request_id, 'goal': g, 'submitted_ns': time.perf_counter_ns()}
            self.last = {'job_id': self.counter, 'request_id': request_id, 'state': 'ACCEPTED', 'success': False, 'complete': False}
            self.wake.set()
            return self.response(request_id, True, 'ROS_CARTESIAN_ACCEPTED', job_id=self.counter)

    def cancel(self):
        with self.lock:
            if self.job is None:
                return self.response(None, False, 'ROS_CARTESIAN_NO_ACTIVE_JOB')
            self.job.setdefault('cancel_received_ns', time.perf_counter_ns())
            self.cancel_event.set(); self.controller.cancel(); self.wake.set()
            return self.response(self.job['request_id'], True, 'ROS_CARTESIAN_CANCEL_ACCEPTED', job_id=self.job['id'])

    def reset(self):
        with self.lock:
            if self.quit_event.is_set():
                return self.response(None, False, 'ROS_CARTESIAN_SHUTTING_DOWN')
            if self.job is not None:
                return self.response(None, False, 'ROS_CARTESIAN_BUSY')
            self.session += 1; self.new_session()
            self.last = {'state': 'IDLE', 'success': False, 'complete': False}
            return self.response(None, True, 'ROS_SIM_EXPLICIT_RESET', map_epoch=self.reference[2])

    def observe(self, obs):
        with self.lock:
            self.cached_pose = {'position_m': obs.pose.position_m, 'frame_id': obs.pose.frame_id,
                                'capture_s': obs.pose.capture_s, 'tracking_ok': obs.pose.tracking_ok,
                                'map_epoch': obs.pose.map_epoch, 'motor_positions_m': obs.motors.positions_m,
                                'time_domain': 'synthetic_monotonic_seconds'}

    def snapshot(self):
        with self.lock:
            return deepcopy({'schema_version': 1, 'execution_domain': 'simulation-only',
                'reference': dict(zip(('parent_frame', 'child_frame', 'map_epoch'), self.reference)),
                'needs_reset': self.needs_reset, 'job': self.last, 'observation': self.cached_pose,
                'simulation_speed': self.speed, 'strategy': self.config.strategy})

    def run(self):
        try:
            while not self.quit_event.is_set():
                self.wake.wait(.05); self.wake.clear()
                with self.lock:
                    job = self.job
                    if job is not None:
                        self.last['state'] = 'RUNNING'
                if job is None:
                    continue
                try:
                    result = self.controller.move_position(job['goal'])
                except Exception as exc:
                    result = {'success': False, 'complete': False, 'state': 'FAULT', 'reason': str(exc)}
                # Completion and cancellation share this lock. A cancel accepted
                # before this commit cannot become a successful ROS result.
                with self.lock:
                    cancelled = self.cancel_event.is_set()
                    if cancelled or result.get('success') is not True:
                        if not self.plant.stopped:
                            receipts = self.plant.stop()
                        else:
                            receipts = result.get('stop_receipts', [])
                        self.needs_reset = True
                        result = dict(result, success=False, complete=False,
                                      state='CANCELLED' if cancelled else 'FAULT', stop_receipts=receipts)
                    self.last = {'job_id': job['id'], 'request_id': job['request_id'], 'state': result['state'],
                        'success': result['success'], 'complete': result['complete'], 'result': result,
                        'submitted_ns': job['submitted_ns'], 'cancel_received_ns': job.get('cancel_received_ns'),
                        'stop_wall_ns': self.plant.stop_wall_ns, 'ended_ns': time.perf_counter_ns(),
                        'simulation_truth_error_m': norm(sub(self.plant.true_position(), job['goal'].position_m)),
                        'simulation_truth_position_m': self.plant.true_position(), 'final_motor_positions_m': self.plant.q_truth,
                        'step_count': len(self.controller.history), 'io_threads': sorted(self.plant.io_threads)}
                    self.history.append(deepcopy(self.last)); self.job = None
        finally:
            if not self.plant.stopped:
                self.plant.stop()

    def close(self):
        with self.lock:
            self.quit_event.set(); self.cancel_event.set(); self.controller.cancel(); self.wake.set()
        self.thread.join(timeout=5.)
        if self.thread.is_alive():
            raise RuntimeError('ROS_CARTESIAN_WORKER_SHUTDOWN_TIMEOUT')
