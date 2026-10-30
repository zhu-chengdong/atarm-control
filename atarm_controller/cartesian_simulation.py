"""Independent, deterministic simulated plant; no transport or hardware access.

The controller uses only now/read/advance/command/stop, never reading the
truth function or true coordinates. Camera noise is separately
seeded. Model parameters are illustrative and are not fit to the real arm.
"""
from dataclasses import replace
import math
import random

from .camera_feedback import CameraPose
from .cartesian_control import CartesianConfig, MotorEvidence, Observation


def simulation_config(**options):
    c = CartesianConfig(
        initial_jacobian=((1., .10, .10), (-.05, .90, -.05), (.05, 0., 1.10)),
        cable_mix=((1., 0.), (-.5, math.sqrt(3)/2), (-.5, -math.sqrt(3)/2)),
        reference_lengths_m=(0., 0., 0., .04),
        min_lengths_m=(-.04, -.04, -.04, 0.), max_lengths_m=(.04, .04, .04, .08))
    return replace(c, **options)


class SimulatedPlant:
    execution_domain = 'simulation-only'

    def __init__(self, *, nonlinear=True, noise_m=.000025, seed=7, fault=None, speed_m_s=.035, camera_period_s=.03):
        self.nonlinear = nonlinear; self.noise_m = noise_m; self.random = random.Random(seed)
        self.fault = fault; self.speed = speed_m_s; self.camera_period = camera_period_s
        self.clock_ns = 10_000_000_000; self.t = self.clock_ns/1e9
        self.q_truth = (0., 0., 0., .04); self.target = self.q_truth
        self.commands = []; self.stop_calls = 0; self.stopped = False
        self.last_camera = None; self.last_motor = None; self.camera_index = -1
        self.motor_index = 0; self.motor_time = -1.; self.on_command = None
        self.reads = 0

    def now(self):
        return self.t

    def forward(self, u):
        """Truth model used only by simulation sensors and test oracle."""
        a, b, s = u
        if self.fault == 'rank_loss' and len(self.commands) >= 3:
            s = 0.
        x = .30+.85*a+.18*b+.22*s
        y = .05-.12*a+1.05*b-.16*s
        z = .18+.12*a-.08*b+.90*s
        if self.nonlinear:
            x += 3*a*s+1.2*b*b
            y += 2*b*s-1.5*a*a
            z += 2.8*a*a+2*b*b-1.5*s*a
        return x, y, z

    def coordinates(self):
        a, b, c, s = self.q_truth
        return (2*a-b-c)/3, (b-c)/math.sqrt(3), s-.04

    def true_position(self):
        return self.forward(self.coordinates())

    def read(self):
        self.reads += 1
        active = len(self.commands) >= 2
        period_ns = round(self.camera_period*1e9)
        frame = self.clock_ns//period_ns
        if self.last_camera is None or (frame != self.camera_index and not (active and self.fault == 'frozen_pose')):
            position = tuple(p+self.random.uniform(-self.noise_m, self.noise_m) for p in self.true_position())
            if active and self.fault == 'pose_jump':
                position = (position[0]+.1, *position[1:])
            u = self.coordinates(); angle = .8*u[0]-.4*u[1]
            self.last_camera = CameraPose(frame, (frame*period_ns)/1e9, 'sim_base', 'camera_optical',
                'sim_map_0' if not (active and self.fault == 'map_reset') else 'sim_map_1',
                not (active and self.fault == 'tracking_lost'), position, (0., math.sin(angle/2), 0., math.cos(angle/2)))
            self.camera_index = frame
        if self.last_motor is None or self.t > self.motor_time:
            self.motor_index += 1; self.motor_time = self.t
            positions = self.target if self.fault == 'fake_motor_success' else self.q_truth
            if active and self.fault == 'common_mode':
                positions = tuple(x+.001 if i < 3 else x for i, x in enumerate(positions))
            if active and self.fault == 'motor_limit':
                positions = (.2, *positions[1:])
            valid = (True, True, not (active and self.fault == 'motor_invalid'), True)
            timestamps = (self.t,)*4 if not (active and self.fault == 'motor_stale') else (self.t-1.,)*4
            self.last_motor = MotorEvidence(tuple(positions), (self.motor_index,)*4, timestamps, valid)
        return Observation(self.last_camera, self.last_motor)

    def command(self, target):
        if self.stopped:
            raise RuntimeError('SIM_COMMAND_AFTER_STOP')
        if len(target) != 4 or not all(math.isfinite(x) for x in target):
            raise ValueError('SIM_INVALID_COMMAND')
        if self.fault == 'command_rejected' and self.commands:
            return False
        self.target = tuple(target)
        self.commands.append({'s': self.t, 'target_q': tuple(target)})
        if self.on_command is not None:
            self.on_command()
        return True

    def advance(self, dt):
        if dt <= 0 or not math.isfinite(dt):
            raise ValueError('SIM_INVALID_TIME_STEP')
        if self.fault not in ('motor_stall', 'fake_motor_success') and not self.stopped:
            delta = tuple(b-a for a, b in zip(self.q_truth, self.target))
            largest = max(abs(x) for x in delta)
            alpha = min(1., self.speed*dt/max(largest, 1e-15))
            self.q_truth = tuple(a+alpha*d for a, d in zip(self.q_truth, delta))
        self.clock_ns += round(dt*1e9); self.t = self.clock_ns/1e9

    def stop(self):
        self.stop_calls += 1; self.stopped = True; self.target = self.q_truth
        return [{'actuator_index': i, 'reply_valid': self.fault != 'stop_unconfirmed',
                 'simulated': True, 'meaning': 'simulation stop acknowledgment, not physical braking'} for i in range(4)]
