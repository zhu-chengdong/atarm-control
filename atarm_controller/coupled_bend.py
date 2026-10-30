"""Three coordinated cable motors, TWO bend coordinates, independent camera.

Explicit 3x2 cable mixing and 3x2 translation Jacobian are required. Common
cable mode is constrained; no arbitrary three-axis or six-DOF arm is assumed.
Only standard-library small-matrix algebra is used. No hardware is opened.
"""
from collections import deque
from dataclasses import dataclass,asdict,replace
import math
from statistics import median
from .camera_feedback import CameraCompletionGate
from .online_jacobian import JacobianUpdateConfig


def length(v): return math.sqrt(sum(x*x for x in v))
def dot(a,b): return sum(x*y for x,y in zip(a,b))
def mv(a,x): return tuple(dot(row,x) for row in a)
def sub(a,b): return tuple(x-y for x,y in zip(a,b))
def gram(a):
    return (sum(r[0]*r[0] for r in a),sum(r[0]*r[1] for r in a),sum(r[1]*r[1] for r in a))
def singulars(a):
    x,y,z=gram(a); disc=math.sqrt((x-z)**2+4*y*y)
    return (math.sqrt(max(0,(x+z-disc)/2)),math.sqrt(max(0,(x+z+disc)/2)))
def solve2(a,b,c,r,s):
    det=a*c-b*b
    if not math.isfinite(det) or det<=1e-14: raise ValueError('BEND_SINGULAR_SYSTEM')
    return ((c*r-b*s)/det,(a*s-b*r)/det)
def inverse_action(a,v,damping=0.0):
    x,y,z=gram(a)
    return solve2(x+damping*damping,y,z+damping*damping,
        sum(r[0]*t for r,t in zip(a,v)),sum(r[1]*t for r,t in zip(a,v)))
def frobenius(a): return math.sqrt(sum(v*v for r in a for v in r))


@dataclass(frozen=True)
class BendServoConfig:
    cable_mix: tuple
    initial_jacobian: tuple
    min_lengths_m: tuple
    max_lengths_m: tuple
    online_update: object = None
    camera_noise_bound_m: float = 0.0001
    gain: float = 0.6
    damping: float = 0.05
    max_step_m: float = 0.0025
    max_offset_m: float = 0.04
    max_total_travel_m: float = 0.2
    speed_m_s: float = 0.02
    period_s: float = 0.02
    settle_s: float = 0.08
    extra_settle_s: float = 0.03
    timeout_s: float = 8.0
    motor_timeout_s: float = 1.0
    motor_progress_m: float = 0.0002
    motor_tolerance_m: float = 0.0004
    deploy_hold_tolerance_m: float = 0.0003
    mix_residual_tolerance_m: float = 0.0003
    min_improvement_m: float = 0.0002
    min_singular_value: float = 0.15
    max_condition: float = 10.0
    max_steps: int = 24
    no_progress_steps: int = 4
    filter_frames: int = 3

    def __post_init__(self):
        for name in ('cable_mix','initial_jacobian'):
            matrix=tuple(tuple(r) for r in getattr(self,name));object.__setattr__(self,name,matrix)
            if len(matrix)!=3 or any(len(r)!=2 for r in matrix) or not all(math.isfinite(v) for r in matrix for v in r):
                raise ValueError(name+' must be an explicit finite 3x2 matrix')
        for name in ('min_lengths_m','max_lengths_m'):
            values=tuple(getattr(self,name));object.__setattr__(self,name,values)
            if len(values)!=3 or not all(math.isfinite(v) for v in values): raise ValueError(name+' must contain 3 finite limits')
        if any(a>=b for a,b in zip(self.min_lengths_m,self.max_lengths_m)): raise ValueError('Ordered cable limits required')
        for name in ('gain','damping','max_step_m','max_offset_m','max_total_travel_m','speed_m_s','period_s',
            'settle_s','extra_settle_s','timeout_s','motor_timeout_s','motor_progress_m','motor_tolerance_m',
            'deploy_hold_tolerance_m','mix_residual_tolerance_m',
            'min_improvement_m','min_singular_value','max_condition','camera_noise_bound_m'):
            value=getattr(self,name)
            if not math.isfinite(value) or value<=0: raise ValueError(name+' must be finite and positive')
        if self.gain>1: raise ValueError('Invalid bend gain')
        for name in ('max_steps','no_progress_steps','filter_frames'):
            value=getattr(self,name)
            if not isinstance(value,int) or isinstance(value,bool) or value<1: raise ValueError(name+' must be a positive integer')
        if self.filter_frames<3 or self.filter_frames%2!=1: raise ValueError('Use an odd window of at least 3 frames')
        if any(abs(sum(r[k] for r in self.cable_mix))>1e-9 for k in (0,1)):
            raise ValueError('Cable mixing must constrain the common cable mode')
        if singulars(self.cable_mix)[0]<1e-6: raise ValueError('Rank-two cable mixing required')
        lo,hi=singulars(self.initial_jacobian)
        if lo<self.min_singular_value or hi/lo>self.max_condition: raise ValueError('Initial bend Jacobian is singular or ill-conditioned')
        if self.online_update is not None and not isinstance(self.online_update,JacobianUpdateConfig):
            raise ValueError('Explicit JacobianUpdateConfig required')
        if self.online_update is not None and self.online_update.position_noise_bound_m<self.camera_noise_bound_m:
            raise ValueError('Learning noise bound must cover the configured camera noise')


class CoupledJacobian:
    def __init__(self,cfg): self.cfg=cfg;self.reset()
    def reset(self):
        self.matrix=self.cfg.initial_jacobian;self.pending=None;self.events=[];self.reference=None
        self.excitation=[0.0,0.0,0.0]
    def new_request(self,key):
        if self.reference is not None and self.reference!=key: raise ValueError('BEND_JACOBIAN_REFERENCE_CHANGED')
        self.reference=key;self.pending=None
    def remember(self,window,target): self.pending={'before':window,'target':tuple(target)}
    def reject(self,event,reason):
        event.update({'accepted':False,'reason':reason,'after_matrix':self.matrix});self.events.append(event)
        return False
    def observe(self,after):
        if self.cfg.online_update is None or self.pending is None: return False
        pair=self.pending;self.pending=None;before=pair['before'];c=self.cfg.online_update
        dq=sub(after['q'],before['q']);du=inverse_action(self.cfg.cable_mix,dq)
        dp=sub(after['p'],before['p'])
        event={'before_matrix':self.matrix,'before':before,'after':after,'dq':dq,'du':du,'dp':dp,'target':pair['target']}
        if before['key']!=after['key'] or after['first_frame']<=before['last_frame'] or after['first_s']<=before['last_s']:
            return self.reject(event,'UNPAIRED_SOURCE_FRAMES')
        if after['last_s']-before['last_s']>c.max_pair_duration_s: return self.reject(event,'PAIR_TOO_OLD')
        if max(before['q_spread'],after['q_spread'])>c.motor_window_spread_m: return self.reject(event,'MOTORS_NOT_SETTLED')
        if max(before['pose_mad'],after['pose_mad'])>c.position_noise_bound_m: return self.reject(event,'NOISY_POSE_WINDOW')
        if abs(after['deploy']-before['deploy'])>self.cfg.deploy_hold_tolerance_m:
            return self.reject(event,'DEPLOY_ACTUATOR_MOVED')
        if max(abs(v) for v in sub(after['q'],pair['target']))>c.tracking_tolerance_m:
            return self.reject(event,'TARGET_NOT_TRACKED')
        if length(sub(dq,mv(self.cfg.cable_mix,du)))>self.cfg.mix_residual_tolerance_m:
            return self.reject(event,'ACTUATION_OUTSIDE_BEND_SUBSPACE')
        if length(du)<c.min_actuation_m: return self.reject(event,'INSUFFICIENT_EXCITATION')
        if length(dp)<max(c.min_translation_m,2*c.position_noise_bound_m*c.min_signal_to_noise):
            return self.reject(event,'NOISE_DOMINATED_TRANSLATION')
        response=length(dp)/length(du)
        if not c.min_norm<=response<=c.max_norm: return self.reject(event,'OBSERVED_GAIN_OUT_OF_BOUNDS')
        predicted=mv(self.matrix,du)
        if dot(predicted,dp)/(length(predicted)*length(dp))<math.cos(c.max_observation_angle_rad):
            return self.reject(event,'DIRECTION_INCONSISTENT')
        u=tuple(v/length(du) for v in du)
        self.excitation[0]+=u[0]*u[0];self.excitation[1]+=u[0]*u[1];self.excitation[2]+=u[1]*u[1]
        innovation=sub(dp,predicted)
        if length(innovation)<=2*c.position_noise_bound_m: return self.reject(event,'MODEL_CONSISTENT_WITH_NOISE')
        denom=dot(du,du)+c.regularization_m**2
        delta=tuple(tuple(c.learning_rate*e*v/denom for v in du) for e in innovation)
        size=frobenius(delta);cap=c.max_fractional_change*frobenius(self.matrix)
        if size>cap: delta=tuple(tuple(v*cap/size for v in r) for r in delta)
        candidate=tuple(tuple(a+b for a,b in zip(r,s)) for r,s in zip(self.matrix,delta))
        lo,hi=singulars(candidate)
        if lo<self.cfg.min_singular_value or hi/lo>self.cfg.max_condition or frobenius(candidate)>c.max_norm:
            return self.reject(event,'UPDATED_MODEL_ILL_CONDITIONED')
        newpred=mv(candidate,du)
        if dot(predicted,newpred)/(length(predicted)*length(newpred))<math.cos(c.max_update_angle_rad):
            return self.reject(event,'UPDATED_DIRECTION_TOO_LARGE')
        self.matrix=candidate
        event.update({'accepted':True,'reason':'ACCEPTED','after_matrix':candidate,'change_norm':frobenius(delta)})
        self.events.append(event);return True
    def status(self):
        x,y,z=self.excitation;disc=math.sqrt((x-z)**2+4*y*y)
        small=max(0,(x+z-disc)/2);large=max(0,(x+z+disc)/2)
        return {'enabled':self.cfg.online_update is not None,'matrix':self.matrix,'events':list(self.events),
            'accepted':sum(e['accepted'] for e in self.events),'rejected':sum(not e['accepted'] for e in self.events),
            'singular_values':singulars(self.matrix),'excitation_eigenvalues':(small,large),
            'excitation_rank':0 if large<1e-12 else 2 if small>0.05*large else 1,
            'full_matrix_calibration_claimed':False}


from .progress_window import GoalProgressWindow,PauseBudgetConfig

class CoupledBendServo:
    def __init__(self,cfg,camera_config,window_progress_config=None,pause_budget_config=None):
        self.cfg=cfg;self.gate=CameraCompletionGate(camera_config)
        self._base_camera_config=self.gate.config
        if pause_budget_config is not None and not isinstance(pause_budget_config,PauseBudgetConfig):
            raise ValueError('Explicit PauseBudgetConfig required')
        self.pause_budget=pause_budget_config
        if self.pause_budget is not None: window_progress_config=self.pause_budget.window(window_progress_config)
        self.inner_tolerance=self.gate.config.position_tolerance_m-cfg.camera_noise_bound_m
        if self.inner_tolerance<=0: raise ValueError('BEND_CAMERA_ACCURACY_INSUFFICIENT')
        self.progress_window=None if window_progress_config is None else GoalProgressWindow(window_progress_config)
        self.model=CoupledJacobian(cfg);self.reset()
    def reset(self):
        self.pause_budget_status={'enabled':False}
        if self.progress_window is not None: self.progress_window.reset()
        self.model.reset();self.gate.reset();self.state='IDLE';self.history=[]
        self.stop_requested=False;self.stop_receipts=[];self.total=0.0
        self.result={'success':False,'message':'No camera bend completed'}
        self.last_confirmed_goal=None;self.last_confirmed_q=None
    def finish(self,success,message):
        self.result={'success':success,'complete':success,'state':self.state,'message':message}
        return dict(self.result)
    def status(self):
        return {'enabled':True,'state':self.state,'result':dict(self.result),'steps':len(self.history),
            'history':list(self.history),'total_cable_travel_m':self.total,'model':self.model.status(),
            'camera':self.gate.status(),'stop_requested':self.stop_requested,'stop_receipts':list(self.stop_receipts),
            'limits':asdict(self.cfg),'short_window_progress':{'enabled':False} if self.progress_window is None else self.progress_window.status(),
            'pause_budget':dict(self.pause_budget_status)}
    def read_pose(self,arm):
        try: return arm.camera_feedback()
        except Exception as exc: raise ValueError('BEND_CAMERA_PROVIDER_ERROR: '+str(exc))
    def collect(self,entries,key):
        p=tuple(median(e['p'][i] for e in entries) for i in range(3))
        return {'p':p,'q':tuple(median(e['q'][i] for e in entries) for i in range(3)),
            'deploy':median(e['deploy'] for e in entries),'key':key,
            'first_frame':entries[0]['frame'],'last_frame':entries[-1]['frame'],
            'first_s':entries[0]['s'],'last_s':entries[-1]['s'],
            'q_spread':max(max(e['q'][i] for e in entries)-min(e['q'][i] for e in entries) for i in range(3)),
            'pose_mad':length(tuple(median(abs(e['p'][i]-p[i]) for e in entries) for i in range(3)))}
    def move(self,arm,goal):
        if arm._bend_motion_active or arm._visual_motion_active:
            return self.finish(False,'BEND_MOTION_BUSY')
        if not arm._reference_known or arm.deploy_state!='DEPLOYED' or arm.deployed is not True:
            return self.finish(False,'BEND_REQUIRES_CONFIRMED_DEPLOYMENT')
        cfg=self.cfg
        try:
            arm._cycle_checkpoint()
            self.gate.begin(goal,self.read_pose(arm),arm._clock())
            key=(goal.parent_frame,goal.child_frame,goal.map_epoch)
            self.model.new_request(key)
            if not arm._refresh_all(): raise ValueError(arm._feedback_failure_message())
            feedbacks=[m.get_current_status() for m in arm.motors]
            anchor=tuple(f['actuate_len'] for f in feedbacks[:3]);held=feedbacks[3]['actuate_len']
            if any(not lo<=q<=hi for lo,q,hi in zip(cfg.min_lengths_m,anchor,cfg.max_lengths_m)):
                raise ValueError('BEND_INITIAL_POSITION_OUT_OF_LIMIT')
            arm._bend_motion_active=True;self.state='RUNNING'
            self.history=[];self.total=0.0;self.stop_requested=False;self.stop_receipts=[]
            target=command=anchor;started=previous=arm._clock();settled_since=None
            motor_timeout=cfg.motor_timeout_s
            if self.progress_window is not None:
                policy=self.progress_window.cfg
                rate=policy.expected_bend_rate_m_s*min(1.0,cfg.speed_m_s/policy.reference_bend_speed_m_s,
                    cfg.max_step_m/policy.reference_bend_step_m)
                self.progress_window.begin(goal,self.gate.config.position_tolerance_m,started,rate)
                if self.pause_budget is not None:
                    minimum_rate=self.progress_window.rate
                    camera_per_motor=singulars(cfg.initial_jacobian)[1]/singulars(cfg.cable_mix)[0]
                    motor_timeout=self.pause_budget.timeout(cfg.motor_timeout_s,cfg.motor_progress_m,minimum_rate,camera_per_motor)
                    camera_timeout=self.pause_budget.timeout(self._base_camera_config.progress_timeout_s,
                        self._base_camera_config.progress_min_m,minimum_rate)
                    self.gate.config=replace(self._base_camera_config,progress_timeout_s=camera_timeout)
                    self.pause_budget_status={'enabled':True,**asdict(self.pause_budget),
                        'motor_timeout_s':motor_timeout,'camera_progress_timeout_s':camera_timeout,
                        'minimum_camera_rate_m_s':minimum_rate,'camera_per_motor':camera_per_motor,
                        'motor_progress_m':cfg.motor_progress_m,'window_s':policy.window_s}
            progress_mark=anchor;last_progress=started
            best_error=None;no_progress=0;last_frame=None
            holding=(self.last_confirmed_goal==goal and self.last_confirmed_q is not None and
                max(abs(v) for v in sub(anchor,self.last_confirmed_q))<=cfg.motor_tolerance_m)
            window=deque(maxlen=cfg.filter_frames)
            while True:
                arm._cycle_checkpoint()
                now=arm._clock()
                if now-started>=cfg.timeout_s: raise ValueError('BEND_OPERATION_TIMEOUT')
                delta=sub(target,command);largest=max(abs(v) for v in delta)
                step=cfg.speed_m_s*min(max(now-previous,0.0),2*cfg.period_s)
                alpha=1.0 if largest<=1e-12 else min(1.0,step/largest)
                command=tuple(q+alpha*d for q,d in zip(command,delta));previous=now
                for i in range(3):
                    if not arm.motors[i].set_target_actuation_length(command[i]):
                        raise ValueError('BEND_MOTOR_FEEDBACK_INVALID: '+arm._feedback_failure_message())
                    arm.last_commanded_lengths[i]=command[i]
                if not arm.motors[3].refresh_feedback(): raise ValueError('BEND_DEPLOY_FEEDBACK_INVALID')
                feedbacks=[m.get_current_status() for m in arm.motors]
                if not all(f['feedback_valid'] for f in feedbacks): raise ValueError('BEND_MOTOR_FEEDBACK_INVALID')
                q=tuple(f['actuate_len'] for f in feedbacks[:3]);deploy=feedbacks[3]['actuate_len']
                if abs(deploy-held)>cfg.deploy_hold_tolerance_m: raise ValueError('BEND_DEPLOY_ACTUATOR_MOVED')
                if any(not lo<=v<=hi for lo,v,hi in zip(cfg.min_lengths_m,q,cfg.max_lengths_m)):
                    raise ValueError('BEND_MEASURED_TRAVEL_LIMIT')
                actual_u=inverse_action(cfg.cable_mix,sub(q,anchor))
                if length(sub(sub(q,anchor),mv(cfg.cable_mix,actual_u)))>cfg.mix_residual_tolerance_m:
                    raise ValueError('BEND_COMMON_MODE_CHANGED')
                pose=self.read_pose(arm);arm._cycle_checkpoint();ready=self.gate.observe(pose,arm._clock())
                if self.progress_window is not None: self.progress_window.observe(pose)
                if length(sub(q,progress_mark))>=cfg.motor_progress_m: progress_mark=q;last_progress=now
                reached=(max(abs(v) for v in sub(command,target))<1e-9 and
                    max(abs(v) for v in sub(q,target))<=cfg.motor_tolerance_m and
                    all(abs(f['actuate_len_vel'])<=0.002 for f in feedbacks[:3]))
                if not reached:
                    settled_since=None;window.clear()
                    if now-last_progress>=motor_timeout: raise ValueError('BEND_MOTOR_PROGRESS_STALLED')
                else:
                    if settled_since is None: settled_since=now
                    eligible=(now-settled_since>=cfg.settle_s and pose.capture_s>=settled_since+cfg.extra_settle_s)
                    if eligible and pose.frame_id!=last_frame:
                        last_frame=pose.frame_id
                        window.append({'p':pose.position_m,'q':q,'deploy':deploy,'frame':pose.frame_id,'s':pose.capture_s})
                        if len(window)==cfg.filter_frames:
                            evidence=self.collect(list(window),key)
                            self.model.observe(evidence)
                            error=sub(goal.position_m,evidence['p']);magnitude=length(error)
                            if ready and magnitude<=self.inner_tolerance:
                                arm._bend_motion_active=False;self.state='SUCCEEDED'
                                if self.progress_window is not None: self.progress_window.finish(True)
                                self.last_confirmed_goal=goal;self.last_confirmed_q=q
                                return self.finish(True,'Camera bend goal confirmed')
                            # Once inside the task tolerance, hold still for independent
                            # stable-frame confirmation instead of chasing zero error.
                            if magnitude<=self.inner_tolerance: holding=True
                            elif holding and magnitude<=self.gate.config.position_tolerance_m: pass
                            else:
                                holding=False
                                if best_error is None or best_error-magnitude>=cfg.min_improvement_m:
                                    best_error=magnitude;no_progress=0
                                elif self.history: no_progress+=1
                                if no_progress>=cfg.no_progress_steps: raise ValueError('BEND_CAMERA_NO_PROGRESS')
                                lo,hi=singulars(self.model.matrix)
                                if lo<cfg.min_singular_value or hi/lo>cfg.max_condition: raise ValueError('BEND_MODEL_ILL_CONDITIONED')
                                du=inverse_action(self.model.matrix,error,cfg.damping)
                                projection=sub(error,mv(self.model.matrix,inverse_action(self.model.matrix,error)))
                                if length(projection)>self.gate.config.position_tolerance_m:
                                    raise ValueError('BEND_UNCONTROLLABLE_TRANSLATION')
                                dq=mv(cfg.cable_mix,tuple(cfg.gain*v for v in du))
                                scale=min(1.0,cfg.max_step_m/max(max(abs(v) for v in dq),1e-12))
                                dq=tuple(v*scale for v in dq)
                                proposed=tuple(a+b for a,b in zip(target,dq))
                                if any(not lo<=v<=hi for lo,v,hi in zip(cfg.min_lengths_m,proposed,cfg.max_lengths_m)):
                                    raise ValueError('BEND_TRAVEL_LIMIT')
                                if max(abs(v) for v in sub(proposed,anchor))>cfg.max_offset_m:
                                    raise ValueError('BEND_OFFSET_LIMIT')
                                if len(self.history)>=cfg.max_steps: raise ValueError('BEND_ITERATION_LIMIT')
                                travel=sum(abs(v) for v in dq)
                                if self.total+travel>cfg.max_total_travel_m: raise ValueError('BEND_TOTAL_TRAVEL_LIMIT')
                                self.total+=travel
                                self.history.append({'from_q':target,'to_q':proposed,'delta_q':dq,'error_m':magnitude,
                                    'frame':pose.frame_id,'capture_s':pose.capture_s,'settled_since_s':settled_since,
                                    'jacobian':self.model.matrix,'no_progress_steps':no_progress})
                                self.model.remember(evidence,proposed)
                                target=proposed;arm.final_target_actuation_lengths[:3]=target
                                settled_since=None;window.clear();progress_mark=q;last_progress=now
                arm._sleep(cfg.period_s)
        except Exception as exc:
            self.state='FAULT'
            arm._fault(str(exc))
            return self.finish(False,str(exc))
