"""Bounded, non-adaptive diagnostic motions before recovery grants retry."""
from copy import deepcopy
from dataclasses import dataclass,asdict,replace
import math
from .camera_feedback import CameraCompletionGate,CameraGateConfig


def norm(v): return math.sqrt(sum(x*x for x in v))
def sub(a,b): return tuple(x-y for x,y in zip(a,b))


@dataclass(frozen=True)
class ProbeWaypoint:
    name: str
    joint_offset_m: tuple
    camera_offset_m: tuple
    def __post_init__(self):
        for key,size in (('joint_offset_m',4),('camera_offset_m',3)):
            v=tuple(getattr(self,key));object.__setattr__(self,key,v)
            if len(v)!=size or not all(math.isfinite(x) for x in v): raise ValueError('Invalid probe '+key)
        if not isinstance(self.name,str) or not self.name: raise ValueError('Probe waypoint needs a name')


@dataclass(frozen=True)
class ProbeConfig:
    waypoints: tuple
    max_excursion_m: tuple
    max_total_joint_travel_m: float=0.02
    max_step_m: float=0.0002
    speed_m_s: float=0.005
    period_s: float=0.02
    settle_s: float=0.1
    timeout_s: float=5.0
    stage_timeout_s: float=1.0
    no_progress_s: float=0.3
    motor_tolerance_m: float=0.00008
    camera_tolerance_m: float=0.00025
    camera_noise_bound_m: float=0.00004
    min_camera_motion_m: float=0.0003
    max_pose_age_s: float=0.1
    direction_cos_min: float=0.7
    def __post_init__(self):
        object.__setattr__(self,'waypoints',tuple(self.waypoints))
        object.__setattr__(self,'max_excursion_m',tuple(self.max_excursion_m))
        if len(self.max_excursion_m)!=4 or not all(math.isfinite(x) and x>0 for x in self.max_excursion_m):
            raise ValueError('Four explicit probe excursion limits required')
        for k,v in asdict(self).items():
            if k in ('waypoints','max_excursion_m'): continue
            if isinstance(v,bool) or not math.isfinite(v) or v<=0: raise ValueError('Invalid probe '+k)
        if self.direction_cos_min>1 or self.camera_noise_bound_m>=self.camera_tolerance_m:
            raise ValueError('Invalid probe direction/noise budget')
        if not 2<=len(self.waypoints)<=8 or not all(isinstance(p,ProbeWaypoint) for p in self.waypoints):
            raise ValueError('Explicit bounded probe waypoints required')
        if len({p.name for p in self.waypoints})!=len(self.waypoints): raise ValueError('Probe names must be unique')
        previous=(0.0,)*4;camera=(0.0,)*3;travel=0.0
        for p in self.waypoints:
            if any(abs(x)>limit+1e-12 for x,limit in zip(p.joint_offset_m,self.max_excursion_m)):
                raise ValueError('Probe excursion exceeds configured limit')
            travel+=sum(abs(x) for x in sub(p.joint_offset_m,previous))
            if norm(sub(p.camera_offset_m,camera))<=self.min_camera_motion_m+2*self.camera_noise_bound_m:
                raise ValueError('Probe displacement is not distinguishable from configured noise')
            previous=p.joint_offset_m;camera=p.camera_offset_m
        if travel>self.max_total_joint_travel_m+1e-12: raise ValueError('Probe travel budget exceeded')
        if any(abs(v)>1e-12 for v in (*previous,*camera)): raise ValueError('Probe must return to its reference')
        if any(max(abs(p.joint_offset_m[i]) for p in self.waypoints)<=2*self.motor_tolerance_m for i in range(4)):
            raise ValueError('Probe must exercise all four actuators')


class RecoveryMotionProbe:
    def __init__(self,cfg):
        self.cfg=cfg;self.active=False;self.state='REQUIRED';self.phase='IDLE';self.stages=[]
        self.total_travel=0.0;self.result={'success':False,'complete':False};self.stop_receipts=[]
    def status(self):
        return {'enabled':True,'active':self.active,'state':self.state,'phase':self.phase,
            'result':dict(self.result),'stages':deepcopy(self.stages),'total_joint_travel_m':self.total_travel,
            'stop_receipts':deepcopy(self.stop_receipts),'limits':asdict(self.cfg)}
    def invalidate(self,message):
        if self.state!='FAULT': self.state='REQUIRED'
        self.result={'success':False,'complete':False,'message':message}
    def checkpoint(self,arm,recovery):
        recovery.checkpoint(arm)
        if arm._clock()-self.started>=self.cfg.timeout_s: raise ValueError('PROBE_TIME_LIMIT')
    def run(self,arm,recovery,home_goal,initial_pose):
        self.active=True;self.state='RUNNING';self.phase='PREFLIGHT';self.stages=[];self.stop_receipts=[]
        self.started=arm._clock();self.total_travel=0.0
        self.result={'success':False,'complete':False,'message':'Probe in progress'}
        try:
            anchor=tuple(m.get_current_status()['actuate_len'] for m in arm.motors)
            if not all(m.get_current_status()['feedback_valid'] for m in arm.motors):
                raise ValueError('PROBE_INITIAL_MOTOR_FEEDBACK_INVALID')
            command=anchor;previous_offset=(0.0,)*3;previous_pose=initial_pose
            for waypoint in self.cfg.waypoints:
                self.checkpoint(arm,recovery);self.phase=waypoint.name
                target=tuple(a+b for a,b in zip(anchor,waypoint.joint_offset_m))
                expected=replace(home_goal,position_m=tuple(a+b for a,b in zip(home_goal.position_m,waypoint.camera_offset_m)))
                expected_step=sub(waypoint.camera_offset_m,previous_offset)
                entry={'phase':waypoint.name,'started_s':arm._clock(),'verified':False,'anchor_q':anchor,
                    'target_q':target,'goal':asdict(expected),'samples':[]};self.stages.append(entry)
                command,previous_pose=self.stage(arm,recovery,entry,command,target,anchor,expected,expected_step,previous_pose)
                previous_offset=waypoint.camera_offset_m
                entry['ended_s']=arm._clock();entry['verified']=True
            self.state='SUCCEEDED';self.phase='DONE'
            self.result={'success':True,'complete':False,'message':'PROBE_PASSED_POSTCHECK_REQUIRED'}
            return self.status()
        except BaseException as exc:
            self.state='FAULT';self.result={'success':False,'complete':False,'message':str(exc) or type(exc).__name__}
            raise
        finally: self.active=False
    def stage(self,arm,recovery,entry,command,target,anchor,goal,expected_step,previous_pose):
        cfg=self.cfg;began=entry['started_s'];previous=arm._clock();done_at=None;settled_at=None
        gate=CameraCompletionGate(CameraGateConfig(max_age_s=cfg.max_pose_age_s,
            position_tolerance_m=cfg.camera_tolerance_m,settle_s=cfg.settle_s,
            progress_timeout_s=cfg.stage_timeout_s*2))
        first=True;start_pose=previous_pose
        while True:
            self.checkpoint(arm,recovery);now=arm._clock()
            if now-began>=cfg.stage_timeout_s: raise ValueError('PROBE_STAGE_TIME_LIMIT: '+self.phase)
            delta=sub(target,command);largest=max(abs(x) for x in delta)
            allowed=min(cfg.max_step_m,cfg.speed_m_s*max(0.0,now-previous));previous=now
            scale=min(1.0,allowed/largest) if largest>1e-12 else 1.0
            proposed=tuple(a+scale*d for a,d in zip(command,delta))
            travel=sum(abs(x) for x in sub(proposed,command))
            if self.total_travel+travel>cfg.max_total_joint_travel_m+1e-10: raise ValueError('PROBE_TRAVEL_LIMIT')
            if any(abs(q-a)>limit+1e-10 for q,a,limit in zip(proposed,anchor,cfg.max_excursion_m)):
                raise ValueError('PROBE_EXCURSION_LIMIT')
            for i,m in enumerate(arm.motors):
                if not m.set_target_actuation_length(proposed[i]):
                    raise ValueError('PROBE_MOTOR_FEEDBACK_INVALID: '+str(i)+': '+m.get_current_status()['feedback_error'])
            command=proposed;self.total_travel+=travel
            arm.last_commanded_lengths=list(command);arm.final_target_actuation_lengths=list(target)
            feedback=[m.get_current_status() for m in arm.motors]
            if not all(s['feedback_valid'] for s in feedback): raise ValueError('PROBE_MOTOR_FEEDBACK_INVALID')
            q=tuple(s['actuate_len'] for s in feedback)
            if max(abs(v) for v in sub(command,target))<1e-12:
                if done_at is None: done_at=arm._clock()
            reached=(done_at is not None and max(abs(v) for v in sub(q,target))<=cfg.motor_tolerance_m and
                all(abs(s['actuate_len_vel'])<=0.005 for s in feedback))
            if reached:
                if settled_at is None: settled_at=arm._clock()
            else:
                settled_at=None
                if done_at is not None and arm._clock()-done_at>=cfg.no_progress_s:
                    raise ValueError('PROBE_MOTOR_NOT_TRACKING: '+self.phase)
            try: pose=arm.camera_feedback()
            except Exception as exc: raise ValueError('PROBE_CAMERA_PROVIDER_ERROR: '+str(exc))
            self.checkpoint(arm,recovery)
            if (previous_pose is not None and getattr(pose,'capture_s',-1)>=began and
                isinstance(getattr(pose,'frame_id',None),int) and pose.frame_id<=previous_pose.frame_id):
                raise ValueError('PROBE_CAMERA_SOURCE_NOT_ADVANCED: '+self.phase)
            # Capture an independent reference before the first nonzero step.
            if start_pose is None: start_pose=pose
            ready=gate.begin(goal,pose,arm._clock()) if first else gate.observe(pose,arm._clock())
            first=False
            recovery.source_highwater=max(recovery.source_highwater,pose.frame_id)
            entry['samples'].append({'command_q':command,'measured_q':q,'pose':asdict(pose),
                'motor_frame_ids':[s['transport_frame_id'] for s in feedback],
                'motor_rx_ns':[s['rx_monotonic_ns'] for s in feedback],'settled_at_s':settled_at})
            if settled_at is None or pose.capture_s<settled_at:
                gate.stable_start=None;gate.stable_frames=0;ready=False
            if settled_at is not None and pose.capture_s>=settled_at+cfg.settle_s:
                observed=sub(pose.position_m,start_pose.position_m);length=norm(observed)
                lower=max(0.0,length-2*cfg.camera_noise_bound_m)
                if lower<cfg.min_camera_motion_m: raise ValueError('PROBE_CAMERA_NO_RESPONSE: '+self.phase)
                cosine=sum(a*b for a,b in zip(observed,expected_step))/(length*norm(expected_step))
                if cosine<cfg.direction_cos_min: raise ValueError('PROBE_CAMERA_DIRECTION_MISMATCH: '+self.phase)
                proof=gate.status()
                if proof['position_error_m']+cfg.camera_noise_bound_m>cfg.camera_tolerance_m:
                    raise ValueError('PROBE_CAMERA_RESPONSE_MISMATCH: '+self.phase)
                if ready and gate.stable_start>=settled_at:
                    entry.update({'proof':proof,'response_lower_bound_m':lower,'direction_cosine':cosine,
                        'settled_at_s':settled_at,'frame_id':pose.frame_id,'capture_s':pose.capture_s})
                    return command,pose
            arm._sleep(cfg.period_s)
