"""Fail-closed orchestration of deploy, camera bend, neutralize and retract.

No homing, automatic fault recovery or fault-cleanup retraction is performed.
All stages use the existing guarded controls; completion is rechecked using
fresh independent pose and motor evidence. The caller supplies calibrated goals.
"""
from copy import deepcopy
from dataclasses import dataclass,asdict
import math
import threading
from .camera_feedback import CameraGoal,CameraGateConfig,CameraCompletionGate,_valid_geometry


@dataclass(frozen=True)
class CyclePlan:
    neutral_goal: CameraGoal
    bend_goals: tuple
    def __post_init__(self): object.__setattr__(self,'bend_goals',tuple(self.bend_goals))


@dataclass(frozen=True)
class CycleConfig:
    timeout_s: float = 40.0
    verification_timeout_s: float = 1.0
    verification_period_s: float = 0.02
    position_tolerance_m: float = 0.001
    orientation_tolerance_rad: float = 0.05
    noise_bound_m: float = 0.0002
    max_pose_age_s: float = 0.1
    settle_s: float = 0.1
    motor_tolerance_m: float = 0.0005
    neutral_tolerance_m: float = 0.0008
    max_bend_goals: int = 8
    def __post_init__(self):
        for key,value in asdict(self).items():
            if not math.isfinite(value) or value<=0: raise ValueError(key+' must be finite and positive')
        if not isinstance(self.max_bend_goals,int) or isinstance(self.max_bend_goals,bool):
            raise ValueError('max_bend_goals must be an integer')
        if self.noise_bound_m>=self.position_tolerance_m: raise ValueError('CYCLE_CAMERA_ACCURACY_INSUFFICIENT')


class CameraArmCycle:
    def __init__(self,cfg=None):
        self.cfg=cfg or CycleConfig();self.run_id=0;self.cancelled=threading.Event()
        self.run_lock=threading.Lock();self.state_lock=threading.RLock();self.reset()
    def reset(self):
        self.active=False;self.owner=None;self.state='IDLE';self.phase='IDLE';self.events=[]
        self.stop_requested=False;self.stop_receipts=[];self.cancelled.clear()
        self.result={'success':False,'complete':False,'state':'IDLE','message':'No cycle completed'}
    def status(self):
        return {'active':self.active,'run_id':self.run_id,'state':self.state,'phase':self.phase,
            'result':dict(self.result),'stages':deepcopy(self.events),'stop_requested':self.stop_requested,
            'stop_receipts':deepcopy(self.stop_receipts),'cancel_requested':self.cancelled.is_set(),
            'limits':asdict(self.cfg)}
    def request_cancel(self):
        with self.state_lock:
            if not self.active or self.state!='RUNNING': return {'success':False,'message':'CYCLE_NOT_RUNNING'}
            self.cancelled.set();return {'success':True,'message':'CYCLE_CANCEL_REQUESTED'}
    def checkpoint(self,arm):
        if not self.active: return
        if self.cancelled.is_set(): raise ValueError('CYCLE_CANCELLED')
        if arm._clock()-self.started>=self.cfg.timeout_s: raise ValueError('CYCLE_TIME_LIMIT')
        if arm.deploy_state=='FAULT': raise ValueError('CYCLE_ARM_FAULT: '+arm.last_motion_result['message'])
    def finish(self,success,message):
        self.result={'success':success,'complete':success,'state':self.state,'phase':self.phase,
            'message':message,'run_id':self.run_id}
        return dict(self.result)
    def motor_summary(self,arm):
        result=[]
        for m in arm.motors:
            f=m.get_current_status();v=f['actuate_len']
            result.append({'valid':f['feedback_valid'],'length_m':v if math.isfinite(v) else None,
                'frame_id':f['transport_frame_id'],'age_s':f['feedback_age_s'],'error':f['feedback_error']})
        return result
    def validate(self,arm,plan):
        if not isinstance(plan,CyclePlan) or not 1<=len(plan.bend_goals)<=self.cfg.max_bend_goals:
            raise ValueError('CYCLE_INVALID_PLAN')
        if not arm.camera_required or not callable(arm.camera_feedback) or arm.bend_controller is None or arm.visual_servo is None:
            raise ValueError('CYCLE_CLOSED_LOOP_NOT_CONFIGURED')
        deployed=arm.camera_goals.get(True);retracted=arm.camera_goals.get(False)
        goals=(plan.neutral_goal,*plan.bend_goals,deployed,retracted)
        if any(not isinstance(g,CameraGoal) or not _valid_geometry(g) for g in goals):
            raise ValueError('CYCLE_INVALID_GOAL')
        keys=[(g.parent_frame,g.child_frame,g.map_epoch) for g in goals]
        if any(not all(key) or key!=keys[0] for key in keys): raise ValueError('CYCLE_GOAL_REFERENCE_MISMATCH')
        if plan.neutral_goal!=deployed: raise ValueError('CYCLE_NEUTRAL_REFERENCE_MISMATCH')
        if not arm.bend_controller.cfg.camera_noise_bound_m<self.cfg.neutral_tolerance_m<=arm.camera_gate.config.position_tolerance_m:
            raise ValueError('CYCLE_HANDOFF_PRECISION_INCONSISTENT')
        if not arm._reference_known or arm.deploy_state!='RETRACTED' or arm.deployed is not False:
            raise ValueError('CYCLE_REQUIRES_CONFIRMED_RETRACTED_START')
    def read_pose(self,arm):
        try: return arm.camera_feedback()
        except Exception as exc: raise ValueError('CYCLE_CAMERA_PROVIDER_ERROR: '+str(exc))
    def verify(self,arm,goal):
        cfg=self.cfg
        gate=CameraCompletionGate(CameraGateConfig(max_age_s=cfg.max_pose_age_s,
            position_tolerance_m=cfg.position_tolerance_m,orientation_tolerance_rad=cfg.orientation_tolerance_rad,
            settle_s=cfg.settle_s,progress_timeout_s=cfg.verification_timeout_s*2))
        began=arm._clock();gate.begin(goal,self.read_pose(arm),arm._clock())
        while True:
            self.checkpoint(arm)
            if arm._clock()-began>=cfg.verification_timeout_s: raise ValueError('CYCLE_FINAL_VERIFICATION_TIMEOUT')
            if not arm._refresh_all(): raise ValueError('CYCLE_VERIFY_MOTOR_FEEDBACK_INVALID')
            feedback=[m.get_current_status() for m in arm.motors]
            if not all(f['feedback_valid'] for f in feedback): raise ValueError('CYCLE_VERIFY_MOTOR_FEEDBACK_INVALID')
            if any(abs(f['actuate_len']-target)>cfg.motor_tolerance_m for f,target in zip(feedback,arm.last_commanded_lengths)):
                raise ValueError('CYCLE_VERIFY_MOTOR_NOT_TRACKING')
            pose=self.read_pose(arm);self.checkpoint(arm)
            ready=gate.observe(pose,arm._clock())
            error=gate.status().get('position_error_m',float('inf'))
            if ready and pose.capture_s>=began and error+cfg.noise_bound_m<=cfg.position_tolerance_m:
                return {'success':True,'complete':True,'message':'Independent pose and motor verification confirmed',
                    'evidence':gate.status(),'capture_s':pose.capture_s,'frame_id':pose.frame_id}
            arm._sleep(cfg.verification_period_s)
    def stage(self,arm,label,call,goal,kind):
        self.checkpoint(arm);self.phase=label
        entry={'phase':label,'started_s':arm._clock(),'verified':False,'response':None};self.events.append(entry)
        result=call()
        entry['ended_s']=arm._clock()
        if not isinstance(result,dict): raise ValueError('CYCLE_INVALID_STAGE_RESULT: '+label)
        entry['response']={k:result.get(k) for k in ('success','complete','message')}
        entry['motors']=self.motor_summary(arm)
        if kind=='bend':
            entry['controller']=deepcopy(arm.bend_status());gate=arm.bend_controller.gate
        elif kind=='deploy':
            entry['controller']={'camera':arm.camera_status(),'visual':arm.visual_servo_status(),
                'motion':dict(arm.last_motion_result)};gate=arm.camera_gate
        else: gate=None;entry['evidence']=deepcopy(result.get('evidence',{}))
        if result.get('success') is not True or result.get('complete') is not True:
            raise ValueError('CYCLE_STAGE_FAILED: '+label+': '+str(result.get('message')))
        self.checkpoint(arm)
        expected='RETRACTED' if label in ('VERIFY_START','RETRACT','VERIFY_END') else 'DEPLOYED'
        if arm.deploy_state!=expected or arm.deployed is not (expected=='DEPLOYED'):
            raise ValueError('CYCLE_STAGE_STATE_MISMATCH: '+label)
        if not all(m['valid'] for m in entry['motors']): raise ValueError('CYCLE_STAGE_FEEDBACK_INVALID: '+label)
        if gate is not None:
            proof=gate.status();pose=gate.last_sample
            if not proof.get('ready') or proof.get('goal')!=asdict(goal):
                raise ValueError('CYCLE_STAGE_POSE_NOT_CONFIRMED: '+label)
            if pose is None or pose.capture_s<entry['started_s'] or arm._clock()-pose.capture_s>self.cfg.max_pose_age_s:
                raise ValueError('CYCLE_STAGE_STALE_PROOF: '+label)
            if kind=='bend' and arm.bend_status()['state']!='SUCCEEDED':
                raise ValueError('CYCLE_STAGE_STATE_MISMATCH: '+label)
            entry['capture_s']=pose.capture_s;entry['frame_id']=pose.frame_id
            if label=='DEPLOY' and proof.get('position_error_m',float('inf'))+arm.bend_controller.cfg.camera_noise_bound_m>self.cfg.neutral_tolerance_m:
                raise ValueError('CYCLE_DEPLOY_PRECISION_INSUFFICIENT')
        else:
            entry['capture_s']=result['capture_s'];entry['frame_id']=result['frame_id']
        if self.last_stage_frame is not None and entry['frame_id']<=self.last_stage_frame:
            raise ValueError('CYCLE_STAGE_FRAME_NOT_ADVANCING: '+label)
        self.last_stage_frame=entry['frame_id'];entry['verified']=True
        return result
    def run(self,arm,plan):
        if not self.run_lock.acquire(blocking=False): return {'success':False,'complete':False,'message':'CYCLE_BUSY'}
        try: return self.run_owned(arm,plan)
        finally: self.run_lock.release()
    def run_owned(self,arm,plan):
        if self.active: return {'success':False,'complete':False,'message':'CYCLE_BUSY'}
        if arm.deploy_state=='FAULT':
            return {'success':False,'complete':False,'message':'CYCLE_EXPLICIT_RECOVERY_REQUIRED'}
        try: self.validate(arm,plan)
        except Exception as exc:
            self.state='REJECTED';self.phase='PREFLIGHT';self.events=[]
            return self.finish(False,str(exc))
        self.run_id+=1;self.active=True;self.owner=object();self.state='RUNNING';self.phase='PREFLIGHT'
        self.events=[];self.stop_requested=False;self.stop_receipts=[];self.cancelled.clear()
        self.started=arm._clock();self.last_stage_frame=None;arm._cycle_active=True
        bend_camera_config=arm.bend_controller.gate.config
        self.result={'success':False,'complete':False,'state':'RUNNING','message':'Cycle in progress'}
        try:
            self.stage(arm,'VERIFY_START',lambda:self.verify(arm,arm.camera_goals[False]),arm.camera_goals[False],'verify')
            self.stage(arm,'DEPLOY',lambda:arm.set_deploy(True,_cycle_token=self.owner),arm.camera_goals[True],'deploy')
            for index,goal in enumerate(plan.bend_goals,1):
                self.stage(arm,'BEND_'+str(index),lambda g=goal:arm.move_bend_camera(g,_cycle_token=self.owner),goal,'bend')
            # Return to the same configured reference more precisely than the
            # retract controller's total position tolerance, reserving a budget
            # for the later deployment-axis measurement error.
            policy=asdict(bend_camera_config);policy['position_tolerance_m']=self.cfg.neutral_tolerance_m
            arm.bend_controller.gate=CameraCompletionGate(CameraGateConfig(**policy))
            arm.bend_controller.inner_tolerance=self.cfg.neutral_tolerance_m-arm.bend_controller.cfg.camera_noise_bound_m
            self.stage(arm,'NEUTRALIZE',lambda:arm.move_bend_camera(plan.neutral_goal,_cycle_token=self.owner),plan.neutral_goal,'bend')
            self.stage(arm,'RETRACT',lambda:arm.set_deploy(False,_cycle_token=self.owner),arm.camera_goals[False],'deploy')
            self.stage(arm,'VERIFY_END',lambda:self.verify(arm,arm.camera_goals[False]),arm.camera_goals[False],'verify')
            with self.state_lock:
                self.checkpoint(arm)
                self.state='SUCCEEDED';self.phase='DONE'
                return self.finish(True,'Deploy, bend, neutralize, retract and final verification completed')
        except BaseException as exc:
            self.state='FAULT'
            reason=str(exc) if isinstance(exc,Exception) else 'CYCLE_INTERRUPTED: '+type(exc).__name__
            arm._fault(reason);result=self.finish(False,reason)
            if not isinstance(exc,Exception): raise
            return result
        finally:
            self.active=False;self.owner=None;arm._cycle_active=False
            arm.bend_controller.gate=CameraCompletionGate(bend_camera_config)
            arm.bend_controller.inner_tolerance=bend_camera_config.position_tolerance_m-arm.bend_controller.cfg.camera_noise_bound_m
