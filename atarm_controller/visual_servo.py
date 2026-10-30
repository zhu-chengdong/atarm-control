"""Bounded, stop-and-observe correction for ONE deployment actuator.

The supplied local Jacobian column maps actuator metres to camera translation
metres in the goal frame. It must be measured/configured separately. No model,
calibration, source independence or general six-DOF reachability is inferred.
"""
from collections import deque
from dataclasses import dataclass, asdict
import math
from statistics import median
from .online_jacobian import JacobianUpdateConfig,OnlineJacobian


@dataclass(frozen=True)
class VisualServoConfig:
    jacobian_column: tuple
    min_length_m: float
    max_length_m: float
    max_offset_m: float
    max_total_correction_m: float
    max_step_m: float = 0.001
    gain: float = 0.5
    deadband_m: float = 0.0005
    resume_error_m: float = 0.0008
    min_step_m: float = 0.00002
    settle_wait_s: float = 0.05
    update_period_s: float = 0.1
    max_duration_s: float = 5.0
    min_improvement_m: float = 0.0002
    max_adjustments: int = 20
    no_progress_steps: int = 3
    filter_frames: int = 3
    online_update: object = None

    def __post_init__(self):
        object.__setattr__(self,'jacobian_column',tuple(self.jacobian_column))
        j=self.jacobian_column
        if len(j)!=3 or not all(math.isfinite(v) for v in j) or sum(v*v for v in j)<1e-12:
            raise ValueError('Explicit finite nonzero local Jacobian column required')
        if not all(math.isfinite(v) for v in (self.min_length_m,self.max_length_m)) or self.min_length_m>=self.max_length_m:
            raise ValueError('Explicit ordered finite actuator limits required')
        for key in ('max_offset_m','max_total_correction_m','max_step_m','gain','deadband_m','resume_error_m',
                    'min_step_m','settle_wait_s','update_period_s','max_duration_s','min_improvement_m'):
            v=getattr(self,key)
            if not math.isfinite(v) or v<=0: raise ValueError(key+' must be finite and positive')
        if self.gain>1 or self.deadband_m>self.resume_error_m or self.min_step_m>self.max_step_m:
            raise ValueError('Invalid gain, hysteresis or step limits')
        for key in ('max_adjustments','no_progress_steps','filter_frames'):
            v=getattr(self,key)
            if not isinstance(v,int) or isinstance(v,bool) or v<1: raise ValueError(key+' must be a positive integer')
        if self.filter_frames<3 or self.filter_frames%2!=1:
            raise ValueError('Use an odd median window of at least three fresh frames')
        if self.online_update is not None and not isinstance(self.online_update,JacobianUpdateConfig):
            raise ValueError('online_update must be an explicit JacobianUpdateConfig')


class BoundedVisualServo:
    def __init__(self,config,position_tolerance_m):
        self.config=config
        self.tolerance=position_tolerance_m
        if config.resume_error_m>position_tolerance_m:
            raise ValueError('Correction resume threshold must not exceed camera completion tolerance')
        self.online=None if config.online_update is None else OnlineJacobian(config.jacobian_column,config.online_update)
        self.reset()

    def reset(self,reset_estimator=True):
        if self.online is not None:
            if reset_estimator: self.online.reset()
            else: self.online.new_request()
        self.active=False
        self.history=[]
        self.fault=None
        self.stop_receipts=[]
        self.stop_requested=False
        self.nominal=None

    def _fail(self,code):
        self.fault=code
        raise ValueError(code)

    def begin(self,nominal,target,now,*,confirmed_hold=False):
        self.reset(reset_estimator=False)
        self.active=True
        self.nominal=nominal
        self.started=now
        self.last_update=now-self.config.update_period_s
        self.last_frame=None
        self.window=deque(maxlen=self.config.filter_frames)
        self.measurement_window=deque(maxlen=self.config.filter_frames)
        self.holding=True
        self.confirmed_hold=bool(confirmed_hold)
        self.best_error=None
        self.no_progress=0
        self.total_correction=0.0
        self.filtered_error=None
        self.check_target(target)

    def check_target(self,target):
        if not self.config.min_length_m<=target<=self.config.max_length_m:
            self._fail('VISUAL_SERVO_TRAVEL_LIMIT')
        if abs(target-self.nominal)>self.config.max_offset_m+1e-12:
            self._fail('VISUAL_SERVO_OFFSET_LIMIT')

    def check_deadline(self,now):
        if now-self.started>=self.config.max_duration_s:
            self._fail('VISUAL_SERVO_TIMEOUT')

    def plan(self,sample,goal,target,settled_since,now,*,actual_length_m=None,other_lengths_m=None,allow_motion=True):
        self.check_deadline(now)
        if sample.capture_s<settled_since+self.config.settle_wait_s:
            return None
        if self.last_frame is not None and sample.frame_id<=self.last_frame:
            return None
        self.last_frame=sample.frame_id
        self.window.append(tuple(g-p for g,p in zip(goal.position_m,sample.position_m)))
        if self.online is not None:
            if actual_length_m is None or other_lengths_m is None or len(other_lengths_m)!=3 or not all(
                    math.isfinite(v) for v in (actual_length_m,*other_lengths_m)):
                self._fail('JACOBIAN_ACTUAL_FEEDBACK_REQUIRED')
            self.measurement_window.append({'position_m':tuple(sample.position_m),'actuation_m':actual_length_m,
                'other_lengths_m':tuple(other_lengths_m),'frame_id':sample.frame_id,'capture_s':sample.capture_s})
        if len(self.window)<self.config.filter_frames or now-self.last_update<self.config.update_period_s:
            return None
        error=tuple(median(v[i] for v in self.window) for i in range(3))
        magnitude=math.sqrt(sum(v*v for v in error))
        self.filtered_error=magnitude
        paired_window=None
        if self.online is not None:
            entries=list(self.measurement_window)
            position=tuple(median(e['position_m'][i] for e in entries) for i in range(3))
            paired_window={'position_m':position,'actuation_m':median(e['actuation_m'] for e in entries),
                'other_lengths_m':tuple(median(e['other_lengths_m'][i] for e in entries) for i in range(3)),
                'first_frame':entries[0]['frame_id'],'last_frame':entries[-1]['frame_id'],
                'capture_start_s':entries[0]['capture_s'],'capture_end_s':entries[-1]['capture_s'],
                'motor_spread_m':max(e['actuation_m'] for e in entries)-min(e['actuation_m'] for e in entries),
                'other_spread_m':max(max(e['other_lengths_m'][i] for e in entries)-min(e['other_lengths_m'][i] for e in entries) for i in range(3)),
                'pose_mad_m':math.sqrt(sum(median(abs(e['position_m'][i]-position[i]) for e in entries)**2 for i in range(3))),
                'reference':(goal.parent_frame,goal.child_frame,goal.map_epoch)}
            self.online.observe(paired_window)
        if not allow_motion:
            return None
        if self.confirmed_hold:
            noise=0.0 if self.config.online_update is None else self.config.online_update.position_noise_bound_m
            if magnitude<=self.tolerance+noise:
                return None  # Reconfirm an unchanged endpoint with new frames first.
            self.confirmed_hold=False
        if self.holding and magnitude<=self.config.resume_error_m:
            return None
        if magnitude<=self.config.deadband_m:
            self.holding=True
            return None
        self.holding=False
        if self.best_error is None or self.best_error-magnitude>=self.config.min_improvement_m:
            self.best_error=magnitude
            self.no_progress=0
        elif self.history:
            self.no_progress+=1
        if self.no_progress>=self.config.no_progress_steps:
            self._fail('VISUAL_SERVO_NO_PROGRESS')
        j=self.config.jacobian_column if self.online is None else self.online.column
        dq=sum(a*b for a,b in zip(j,error))/sum(v*v for v in j)
        residual=math.sqrt(sum((e-v*dq)**2 for e,v in zip(error,j)))
        if residual>self.tolerance:
            self._fail('VISUAL_SERVO_UNCONTROLLABLE_TRANSLATION')
        delta=max(-self.config.max_step_m,min(self.config.max_step_m,self.config.gain*dq))
        if abs(delta)<self.config.min_step_m:
            return None
        if len(self.history)>=self.config.max_adjustments:
            self._fail('VISUAL_SERVO_ITERATION_LIMIT')
        if self.total_correction+abs(delta)>self.config.max_total_correction_m+1e-12:
            self._fail('VISUAL_SERVO_TOTAL_TRAVEL_LIMIT')
        proposed=target+delta
        self.check_target(proposed)
        self.total_correction+=abs(delta)
        self.history.append({'frame_id':sample.frame_id,'capture_s':sample.capture_s,
            'planned_at_s':now,'settled_since_s':settled_since,'filtered_error_m':magnitude,
            'filtered_error_vector_m':error,'from_m':target,'delta_m':delta,'to_m':proposed,
            'total_correction_m':self.total_correction,'no_progress_steps':self.no_progress,'jacobian_column':j})
        if self.online is not None:
            self.online.remember(paired_window,proposed)
        self.last_update=now
        self.window.clear()
        self.measurement_window.clear()
        return proposed

    def status(self):
        return {'enabled':True,'active':self.active,'fault':self.fault,'nominal_target_m':self.nominal,
            'adjustments':len(self.history),'total_correction_m':getattr(self,'total_correction',0.0),
            'filtered_error_m':getattr(self,'filtered_error',None),'history':list(self.history),
            'limits':asdict(self.config),'stop_requested':self.stop_requested,
            'stop_receipts':list(self.stop_receipts),
            'online_jacobian':{'enabled':False} if self.online is None else self.online.status()}
