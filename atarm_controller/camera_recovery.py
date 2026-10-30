"""Explicit passive recovery. It does not home or diagnose a latent jam at rest."""
from copy import deepcopy
from dataclasses import dataclass,asdict
import math,threading
from .camera_feedback import CameraCompletionGate,CameraGateConfig,CameraGoal,_valid_geometry
from .serial_adapter import NativeSerialAdapter


@dataclass(frozen=True)
class RecoveryConfig:
    timeout_s: float=3.0
    period_s: float=0.02
    settle_s: float=0.1
    max_pose_age_s: float=0.1
    position_tolerance_m: float=0.0005
    noise_bound_m: float=0.0001
    orientation_tolerance_rad: float=0.05
    motor_stable_m: float=0.0001
    motor_velocity_m_s: float=0.005
    def __post_init__(self):
        for name,value in asdict(self).items():
            if isinstance(value,bool) or not math.isfinite(value) or value<=0: raise ValueError(name+' must be positive and finite')
        if self.noise_bound_m>=self.position_tolerance_m: raise ValueError('RECOVERY_CAMERA_ACCURACY_INSUFFICIENT')


class CameraRecovery:
    def __init__(self,cfg=None):
        self.cfg=cfg or RecoveryConfig();self.active=False;self.state='IDLE';self.attempt_id=0
        self.cancelled=threading.Event();self.state_lock=threading.RLock()
        self.history=[];self.stop_requested=False;self.stop_receipts=[]
        self.source_highwater=-1
        self.result={'success':False,'complete':False,'ready_for_retry':False,'message':'No recovery confirmed'}
    def status(self):
        return {'active':self.active,'state':self.state,'attempt_id':self.attempt_id,'result':dict(self.result),
            'history':deepcopy(self.history),'stop_requested':self.stop_requested,
            'stop_receipts':deepcopy(self.stop_receipts),'limits':asdict(self.cfg)}
    def mark_fault(self,message):
        if not self.active:
            self.state='REQUIRED'
            self.result={'success':False,'complete':False,'ready_for_retry':False,'message':message}
    def cancel(self):
        with self.state_lock:
            if not self.active or self.state not in ('CHECKING','PROBING'): return {'success':False,'message':'RECOVERY_NOT_RUNNING'}
            self.cancelled.set();return {'success':True,'message':'RECOVERY_CANCEL_REQUESTED'}
    def finish(self,success,message):
        self.result={'success':success,'complete':False,'ready_for_retry':success,
            'state':self.state,'attempt_id':self.attempt_id,'message':message}
        return dict(self.result)
    def checkpoint(self,arm):
        if self.cancelled.is_set(): raise ValueError('RECOVERY_CANCELLED')
        if arm._clock()-self.started>=self.cfg.timeout_s: raise ValueError('RECOVERY_TIME_LIMIT')
    def source_floor(self,arm):
        values=[]
        for gate in (arm.camera_gate,arm.bend_controller.gate):
            if gate.last_sample is not None: values.append(gate.last_sample.frame_id)
        for e in arm.cycle.status()['stages']:
            if isinstance(e.get('frame_id'),int): values.append(e['frame_id'])
            ctrl=e.get('controller',{})
            for proof in (ctrl.get('camera',{}),ctrl):
                if isinstance(proof.get('frame_id'),int): values.append(proof['frame_id'])
        return max([self.source_highwater,*values],default=-1)
    def bind_transports(self,arm,replacements):
        if replacements is None: return
        if not isinstance(replacements,(tuple,list)) or len(replacements)!=2:
            raise ValueError('RECOVERY_INVALID_TRANSPORT_REPLACEMENTS')
        old=(arm.serial_instance_active,arm.serial_instance_deploy)
        if replacements[0] is replacements[1] or any(
            not isinstance(p,NativeSerialAdapter) or not p._handle or p.trace or p.last_sample.status!='NO_REPLY' or
            any(p is o for o in old) for p in replacements):
            raise ValueError('RECOVERY_REQUIRES_NEW_EXPLICIT_TRANSPORTS')
        arm.serial_instance_active,arm.serial_instance_deploy=replacements
        for i,m in enumerate(arm.motors): m.rebind_passive_transport(replacements[1 if i==3 else 0])
        arm._serial_generation+=1
        for p in old: p.close()
    def run(self,arm,*,mechanically_confirmed=False,fault_cleared=False,replacement_transports=None):
        if not arm.cycle.run_lock.acquire(blocking=False):
            return {'success':False,'complete':False,'ready_for_retry':False,'message':'RECOVERY_OR_CYCLE_BUSY'}
        entry=None
        try:
            self.attempt_id+=1;self.stop_requested=False;self.stop_receipts=[]
            entry={'attempt_id':self.attempt_id,'started_s':arm._clock(),'prior_cycle':arm.cycle.status(),
                'prior_motion':dict(arm.last_motion_result),'mechanically_confirmed':mechanically_confirmed is True,
                'fault_cleared_ack':fault_cleared is True,'committed':False,'samples':[]}
            self.history.append(entry);self.history=self.history[-16:]
            if mechanically_confirmed is not True or fault_cleared is not True:
                self.state='REJECTED';return self.finish(False,'RECOVERY_REQUIRES_EXTERNAL_HOME_AND_FAULT_CLEAR_CONFIRMATION')
            if arm.deploy_state not in ('FAULT','UNKNOWN') or arm._visual_motion_active or arm._bend_motion_active:
                self.state='REJECTED';return self.finish(False,'RECOVERY_REQUIRES_FAULT_OR_UNKNOWN_STATE')
            if not arm.camera_required or not callable(arm.camera_feedback) or arm.bend_controller is None or arm.visual_servo is None:
                self.state='REJECTED';return self.finish(False,'RECOVERY_CLOSED_LOOP_NOT_CONFIGURED')
            goal=arm.camera_goals.get(False)
            if not isinstance(goal,CameraGoal) or not _valid_geometry(goal):
                self.state='REJECTED';return self.finish(False,'RECOVERY_INVALID_REFERENCE_GOAL')
            self.active=True;arm._recovery_active=True;self.state='CHECKING';self.cancelled.clear()
            self.started=arm._clock();minimum_frame=self.source_floor(arm)
            entry['minimum_camera_frame_id']=minimum_frame
            self.bind_transports(arm,replacement_transports)
            entry['serial_generation']=arm._serial_generation
            cfg=self.cfg;gate=CameraCompletionGate(CameraGateConfig(max_age_s=cfg.max_pose_age_s,
                position_tolerance_m=cfg.position_tolerance_m,orientation_tolerance_rad=cfg.orientation_tolerance_rad,
                settle_s=cfg.settle_s,progress_timeout_s=cfg.timeout_s*2))
            first=True;anchor=None;motor_since=None;phase_started=self.started;probe_done=False
            while True:
                self.checkpoint(arm)
                valid=[m.read_passive_feedback(rebaseline=first) for m in arm.motors]
                if not all(valid): raise ValueError('RECOVERY_MOTOR_FEEDBACK_INVALID: '+arm._feedback_failure_message())
                statuses=[m.get_current_status() for m in arm.motors]
                if any(s['rx_monotonic_ns']/1e9<self.started or not s['feedback_valid'] for s in statuses):
                    raise ValueError('RECOVERY_MOTOR_EVIDENCE_NOT_FRESH')
                current=tuple(s['actuate_len'] for s in statuses)
                if anchor is None: anchor=current;motor_since=arm._clock()
                if any(abs(a-b)>cfg.motor_stable_m for a,b in zip(anchor,current)) or any(
                    abs(s['actuate_len_vel'])>cfg.motor_velocity_m_s for s in statuses):
                    raise ValueError('RECOVERY_MOTOR_NOT_STATIONARY')
                try: pose=arm.camera_feedback()
                except Exception as exc: raise ValueError('RECOVERY_CAMERA_PROVIDER_ERROR: '+str(exc))
                self.checkpoint(arm)
                if (getattr(pose,'capture_s',-1)>=phase_started and
                    isinstance(getattr(pose,'frame_id',None),int) and pose.frame_id<=minimum_frame):
                    raise ValueError('RECOVERY_CAMERA_SOURCE_NOT_ADVANCED')
                ready=gate.begin(goal,pose,arm._clock()) if first else gate.observe(pose,arm._clock())
                first=False
                if pose.capture_s<phase_started: gate.stable_start=None;gate.stable_frames=0
                proof=gate.status()
                self.source_highwater=max(self.source_highwater,pose.frame_id)
                entry['samples'].append({'camera':proof,'motor_lengths_m':current,
                    'motor_frame_ids':[s['transport_frame_id'] for s in statuses],
                    'motor_rx_ns':[s['rx_monotonic_ns'] for s in statuses]})
                if proof['position_error_m']>cfg.position_tolerance_m or proof['orientation_error_rad']>cfg.orientation_tolerance_rad:
                    raise ValueError('RECOVERY_NOT_AT_RETRACTED_POSE')
                if (ready and gate.stable_start>=phase_started and pose.frame_id>minimum_frame and
                    arm._clock()-motor_since>=cfg.settle_s and
                    proof['position_error_m']+cfg.noise_bound_m<=cfg.position_tolerance_m):
                    if arm.motion_probe is not None and not probe_done:
                        self.state='PROBING'
                        entry['probe_started']=True
                        entry['probe']=arm.motion_probe.run(arm,self,goal,gate.last_sample)
                        probe_done=True;self.state='CHECKING'
                        phase_started=arm._clock();entry['post_probe_started_s']=phase_started
                        minimum_frame=self.source_highwater
                        gate=CameraCompletionGate(gate.config);first=True;anchor=None;motor_since=None
                        continue
                    with self.state_lock:
                        self.checkpoint(arm)
                        if not all(m.get_current_status()['feedback_valid'] for m in arm.motors):
                            raise ValueError('RECOVERY_MOTOR_EVIDENCE_EXPIRED')
                        arm._commit_retracted_reference()
                        arm.last_motion_result={'success':True,'complete':False,'state':'RETRACTED',
                            'message':'Recovery checked; a new cycle is still required'}
                        self.state='READY';entry['committed']=True;entry['proof']=deepcopy(proof)
                        return self.finish(True,'RECOVERY_READY_FOR_EXPLICIT_RETRY')
                arm._sleep(cfg.period_s)
        except BaseException as exc:
            self.state='FAILED'
            reason=str(exc) if isinstance(exc,Exception) else 'RECOVERY_INTERRUPTED: '+type(exc).__name__
            arm._fault(reason);result=self.finish(False,reason)
            if not isinstance(exc,Exception): raise
            return result
        finally:
            if entry is not None:
                entry['ended_s']=arm._clock();entry['result']=dict(self.result)
                entry['stop_requested']=self.stop_requested;entry['stop_receipts']=deepcopy(self.stop_receipts)
                if entry.get('probe_started'): entry['probe']=arm.motion_probe.status()
            self.active=False;arm._recovery_active=False
            arm.cycle.run_lock.release()
