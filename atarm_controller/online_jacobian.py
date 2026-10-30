"""Guarded regularized Broyden update for a single translation Jacobian column.

Only pairs of settled, independent camera windows and measured actuator motion
are used. No simulation truth is accessible here. A noise bound and model limits
must be configured; skipped evidence never overwrites the last accepted model.
Reference for the update family: https://flowers.inria.fr/mlopes/myrefs/06-iros-vs.pdf
"""
from dataclasses import dataclass, asdict
import math


def norm(v):
    return math.sqrt(sum(x*x for x in v))


@dataclass(frozen=True)
class JacobianUpdateConfig:
    position_noise_bound_m: float
    min_norm: float
    max_norm: float
    learning_rate: float = 0.5
    regularization_m: float = 0.0002
    min_actuation_m: float = 0.001
    min_translation_m: float = 0.0003
    min_signal_to_noise: float = 1.5
    max_fractional_change: float = 0.25
    max_observation_angle_rad: float = 0.8
    max_update_angle_rad: float = 0.2
    motor_window_spread_m: float = 0.00025
    other_motor_tolerance_m: float = 0.0002
    tracking_tolerance_m: float = 0.0004
    max_pair_duration_s: float = 1.0

    def __post_init__(self):
        for name,value in asdict(self).items():
            if not math.isfinite(value) or value<=0:
                raise ValueError(name+' must be finite and positive')
        if self.min_norm>=self.max_norm or self.learning_rate>1 or self.max_fractional_change>0.5:
            raise ValueError('Invalid Jacobian learning or model bounds')
        if not 0<self.max_update_angle_rad<=self.max_observation_angle_rad<math.pi/2:
            raise ValueError('Model directions must retain a positive local orientation')


class OnlineJacobian:
    def __init__(self,initial,config):
        self.initial=tuple(initial)
        self.config=config
        if not config.min_norm<=norm(self.initial)<=config.max_norm:
            raise ValueError('Initial Jacobian outside configured model bounds')
        self.reset()

    def reset(self):
        self.column=self.initial
        self.events=[]
        self.pending=None
        self.reference=None

    def new_request(self):
        # Never pair measurements across separate requests or an unobserved gap.
        self.pending=None

    def remember(self,window,command_target):
        if self.reference is None:
            self.reference=window['reference']
        if window['reference']!=self.reference:
            raise ValueError('JACOBIAN_REFERENCE_CHANGED')
        self.pending={'before':dict(window),'command_target_m':command_target}

    def _reject(self,event,reason):
        event.update({'accepted':False,'reason':reason,'after_column':self.column})
        self.events.append(event)
        return False

    def observe(self,after):
        if self.reference is None:
            self.reference=after['reference']
        if after['reference']!=self.reference:
            raise ValueError('JACOBIAN_REFERENCE_CHANGED')
        if self.pending is None:
            return False
        pair=self.pending
        self.pending=None  # Exactly one evaluation per actually requested step.
        before=pair['before']; c=self.config
        dq=after['actuation_m']-before['actuation_m']
        dp=tuple(b-a for a,b in zip(before['position_m'],after['position_m']))
        event={'before_column':self.column,'dq_m':dq,'dp_m':dp,'before':before,'after':dict(after),
            'command_target_m':pair['command_target_m']}
        values=(dq,*dp,after['capture_end_s'],before['capture_end_s'])
        if not all(math.isfinite(v) for v in values): return self._reject(event,'NONFINITE_PAIR')
        if (after['reference']!=before['reference'] or
            after['first_frame']<=before['last_frame'] or after['capture_start_s']<=before['capture_end_s']):
            return self._reject(event,'UNPAIRED_SOURCE_FRAMES')
        if after['capture_end_s']-before['capture_end_s']>c.max_pair_duration_s:
            return self._reject(event,'PAIR_TOO_OLD')
        if max(before['motor_spread_m'],after['motor_spread_m'])>c.motor_window_spread_m:
            return self._reject(event,'MOTOR_NOT_SETTLED')
        if max(before['pose_mad_m'],after['pose_mad_m'])>c.position_noise_bound_m:
            return self._reject(event,'NOISY_POSE_WINDOW')
        if any(abs(a-b)>c.other_motor_tolerance_m for a,b in zip(before['other_lengths_m'],after['other_lengths_m'])):
            return self._reject(event,'OTHER_ACTUATOR_MOVED')
        if max(before['other_spread_m'],after['other_spread_m'])>c.other_motor_tolerance_m:
            return self._reject(event,'OTHER_ACTUATOR_NOT_SETTLED')
        if abs(dq)<c.min_actuation_m: return self._reject(event,'INSUFFICIENT_EXCITATION')
        if abs(after['actuation_m']-pair['command_target_m'])>c.tracking_tolerance_m:
            return self._reject(event,'TARGET_NOT_TRACKED')
        if dq*(pair['command_target_m']-before['actuation_m'])<=0:
            return self._reject(event,'ACTUATION_DIRECTION_MISMATCH')
        noise_threshold=2*c.position_noise_bound_m*c.min_signal_to_noise
        if norm(dp)<max(c.min_translation_m,noise_threshold):
            return self._reject(event,'NOISE_DOMINATED_TRANSLATION')
        observed=tuple(v/dq for v in dp)
        event['observed_column']=observed
        if not c.min_norm<=norm(observed)<=c.max_norm:
            return self._reject(event,'OBSERVED_GAIN_OUT_OF_BOUNDS')
        old=self.column; old_norm=norm(old)
        cosine=sum(a*b for a,b in zip(old,observed))/(old_norm*norm(observed))
        if cosine<math.cos(c.max_observation_angle_rad):
            return self._reject(event,'OBSERVED_DIRECTION_INCONSISTENT')
        innovation=tuple(p-j*dq for p,j in zip(dp,old))
        if norm(innovation)<=2*c.position_noise_bound_m:
            return self._reject(event,'MODEL_CONSISTENT_WITH_NOISE')
        # J+ = J + eta*(dp-J*dq)*dq/(dq^2+lambda^2).
        delta=tuple(c.learning_rate*v*dq/(dq*dq+c.regularization_m**2) for v in innovation)
        change=norm(delta); cap=c.max_fractional_change*old_norm
        if change>cap: delta=tuple(v*cap/change for v in delta)
        candidate=tuple(j+d for j,d in zip(old,delta))
        if not c.min_norm<=norm(candidate)<=c.max_norm:
            return self._reject(event,'UPDATED_GAIN_OUT_OF_BOUNDS')
        cosine=sum(a*b for a,b in zip(old,candidate))/(old_norm*norm(candidate))
        if cosine<math.cos(c.max_update_angle_rad):
            return self._reject(event,'UPDATED_DIRECTION_TOO_LARGE')
        self.column=candidate
        event.update({'accepted':True,'reason':'ACCEPTED','after_column':candidate,
            'change_norm':norm(delta),'noise_threshold_m':noise_threshold})
        self.events.append(event)
        return True

    def status(self):
        return {'enabled':True,'initial_column':self.initial,'column':self.column,
            'accepted_updates':sum(e['accepted'] for e in self.events),
            'rejected_updates':sum(not e['accepted'] for e in self.events),
            'pending_pair':self.pending is not None,'events':list(self.events),'limits':asdict(self.config)}
