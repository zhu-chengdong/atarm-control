"""Pulse motion at fake physical rotors; no injection state enters control."""
from dataclasses import replace
from copy import deepcopy
import math,time,threading
from recovery_cases import setup,counts
from probe_cases import config as probe_config
from cycle_cases import sequence_config
from servo_cases import config as deploy_config
from bend_cases import bend_config
from atarm_controller.camera_recovery import RecoveryConfig
from atarm_controller.online_jacobian import JacobianUpdateConfig
from late_jam_cases import verify_stop,RAW_TO_METRES


class PulseMotion:
    def __init__(self,rig,raw,phase,period_s,step_m,false_feedback=False,oscillate=False):
        self.rig=rig;self.raw=raw;self.phase=phase;self.period_s=period_s;self.step_m=step_m
        self.false_feedback=false_feedback;self.oscillate=oscillate;self.event=None;self.pulses=[]
        self.targets={};self.lock=threading.Lock();self.finished=False;self.osc_vector=None;self.forward=True
        self.observations=[];self.last_observed_s=-1
    def observe(self):
        arm=self.rig.arm
        if self.event is None:
            if not arm.cycle.active or arm.cycle.phase!=self.phase: return
            truth=self.raw.truth()
            progress=truth[2] if self.phase=='DEPLOY' else 0.8*truth[0]-0.6*truth[1]
            if progress<(0.01 if self.phase=='DEPLOY' else 0.002): return
            with self.lock:
                board=self.rig.deploy if self.phase=='DEPLOY' else self.rig.active
                self.board=board;self.ids=(2,) if self.phase=='DEPLOY' else (1,0,2)
                self.original_modes=dict(board.motor_modes);self.anchor=dict(board.position)
                self.targets=dict(board.position);self.next_pulse=time.monotonic()+self.period_s
                for mid in self.ids: board.motor_modes[mid]='coherent_false' if self.false_feedback else 'stall'
                self.event={'injected_s':time.monotonic(),'phase':self.phase,'physical_progress_m':progress,
                    'period_s':self.period_s,'step_m':self.step_m,'false_feedback':self.false_feedback,
                    'oscillate':self.oscillate,'position_raw_at_onset':dict(board.position)}
        now=time.monotonic()
        if self.event and arm.cycle.phase==self.phase and now-self.last_observed_s>=0.002:
            self.last_observed_s=now
            if self.phase=='DEPLOY':
                p=arm.motion_progress or {}
                self.observations.append({'s':now,'best_m':p.get('best_m'),
                    'seconds_without_progress':p.get('seconds_without_progress'),
                    'corrections':len(arm.visual_servo.history)})
            else:
                b=arm.bend_controller
                self.observations.append({'s':now,'corrections':len(b.history),
                    'no_progress_steps':b.history[-1]['no_progress_steps'] if b.history else None})
    def before_command(self,board,motor_id,target):
        with self.lock:
            if self.event is None or board is not getattr(self,'board',None) or self.finished: return
            arm=self.rig.arm
            if not arm.cycle.active or arm.cycle.phase!=self.phase:
                board.motor_modes=dict(self.original_modes);self.finished=True;return
            old_targets=dict(self.targets);self.targets[motor_id]=target
            if motor_id!=self.ids[0] or time.monotonic()<self.next_pulse: return
            now=time.monotonic();self.next_pulse=now+self.period_s
            old=tuple(board.position[mid] for mid in self.ids)
            target_vector=tuple(old_targets[mid] for mid in self.ids)
            if self.oscillate:
                if self.osc_vector is None:
                    d=tuple(target_vector[i]-self.anchor[mid] for i,mid in enumerate(self.ids))
                    largest=max(abs(v) for v in d)
                    if largest<1: return
                    scale=min(1.0,self.step_m/RAW_TO_METRES/largest)
                    self.osc_vector=tuple(round(v*scale) for v in d)
                updated=tuple(self.anchor[mid]+(self.osc_vector[i] if self.forward else 0)
                    for i,mid in enumerate(self.ids));self.forward=not self.forward
            else:
                d=tuple(b-a for a,b in zip(old,target_vector));largest=max(abs(v) for v in d)
                scale=min(1.0,self.step_m/RAW_TO_METRES/largest) if largest else 0.0
                updated=tuple(round(a+scale*delta) for a,delta in zip(old,d))
            for mid,value in zip(self.ids,updated): board.position[mid]=value
            movement=max(abs(b-a) for a,b in zip(old,updated))*RAW_TO_METRES
            if movement>0:
                self.pulses.append({'s':now,'old_raw':old,'new_raw':updated,'max_move_m':movement,
                    'rotor_displacement_norm_m':math.sqrt(sum(((b-a)*RAW_TO_METRES)**2 for a,b in zip(old,updated)))})


class PulseCamera:
    def __init__(self,raw,motion): self.raw=raw;self.motion=motion
    def __call__(self):
        pose=self.raw();self.motion.observe();return pose


def options(profile):
    extra={'probe_config':probe_config(),
        'recovery_config':RecoveryConfig(timeout_s=2.0,period_s=0.002,settle_s=0.025,max_pose_age_s=0.07)}
    dc=deploy_config(jacobian_column=(0.0,0.0,1.0),deadband_m=0.00004,resume_error_m=0.00008,
        min_step_m=0.00001,min_improvement_m=0.00003,max_duration_s=1.4,max_adjustments=24)
    dc=replace(dc,online_update=JacobianUpdateConfig(position_noise_bound_m=0.00005,min_norm=0.2,max_norm=3.0,min_actuation_m=0.001))
    bc=bend_config(0.0001)
    if profile in ('deploy_time','deploy_iterations','deploy_travel'):
        dc=replace(dc,max_step_m=0.0001,max_adjustments=1000,max_duration_s=0.55)
        if profile=='deploy_iterations': dc=replace(dc,max_adjustments=4,max_duration_s=1.4)
        if profile=='deploy_travel': dc=replace(dc,max_total_correction_m=0.00035,max_duration_s=1.4)
    if profile in ('bend_time','bend_travel','cycle_time'):
        bc=replace(bc,max_step_m=0.0004,max_steps=1000,timeout_s=0.65,min_improvement_m=0.00005)
        if profile=='bend_travel': bc=replace(bc,max_total_travel_m=0.012,timeout_s=2.0)
    if profile=='cycle_time':
        extra['cycle_config']=sequence_config(timeout_s=1.0)
        bc=replace(bc,timeout_s=2.0)
    extra.update({'visual_servo_config':dc,'bend_servo_config':bc})
    return extra


def exercise(base,name,phase='DEPLOY',period_s=0.04,step_m=0.00065,false_feedback=False,
             oscillate=False,profile='current',noise=0.0,healthy=False):
    rig,raw,plan,ignored=setup(base,extra_options=options(profile))
    try:
        raw.fault_phase=None;raw.fault=None;raw.last=None;raw.noise=noise
        motion=PulseMotion(rig,raw,phase,period_s,step_m,false_feedback,oscillate)
        rig.arm.camera_feedback=PulseCamera(raw,motion)
        rig.arm._fault('SIMULATED_FAULT_BEFORE_CHECKED_RECOVERY')
        ready=rig.arm.recover_camera_cycle(mechanically_confirmed=True,fault_cleared=True)
        assert ready['success'] and not ready['complete'],ready
        passed=rig.arm.probe_status();assert passed['state']=='SUCCEEDED'
        rig.active.late_jam=rig.deploy.late_jam=motion
        result=rig.arm.run_camera_cycle(plan);returned=time.monotonic();cycle=rig.arm.cycle_status()
        goal=rig.arm.camera_goals[False] if healthy else plan.neutral_goal if phase=='DEPLOY' else plan.bend_goals[0]
        truth=raw.truth();error=math.sqrt(sum((a-b)**2 for a,b in zip(truth,goal.position_m)))
        controller=next(s for s in cycle['stages'] if s['phase']==phase).get('controller',{})
        servo=controller.get('visual',{}) if phase=='DEPLOY' else controller
        detail={'profile':profile,'phase':phase,'recovery_result':ready,'passed_probe':passed,
            'result':result,'cycle':cycle,'injection':motion.event,'pulses':motion.pulses,
            'observations':motion.observations,'controller':servo,'true_camera_p':truth,'true_error_m':error,
            'noise_per_axis_m':noise,'root_state':rig.arm.deploy_state}
        base.save_rig(name,rig,detail)
        assert motion.event and len(motion.pulses)>=2,(name,len(motion.pulses),result)
        if healthy:
            assert result['success'] and result['complete'] and cycle['state']=='SUCCEEDED',result
            assert error<=0.001
        else:
            assert not result['success'] and not result['complete'] and cycle['state']=='FAULT',result
            assert rig.arm.deploy_state=='FAULT' and error>0.0015,error
            allowed=('PROGRESS_STALLED','NO_PROGRESS','TIMEOUT','TIME_LIMIT','ITERATION_LIMIT','TOTAL_TRAVEL_LIMIT','OFFSET_LIMIT','TRAVEL_LIMIT')
            assert any(code in result['message'] for code in allowed),result
            assert cycle['phase']==phase
            assert not any(s['phase'] in ('NEUTRALIZE','RETRACT','VERIFY_END') for s in cycle['stages'])
            detail['stop_timing']=verify_stop(rig,cycle,motion.event['injected_s'])
            detail['onset_to_failure_ms']=(returned-motion.event['injected_s'])*1000
            assert detail['onset_to_failure_ms']<2300
            mark=counts(rig);assert not rig.arm.run_camera_cycle(plan)['success'];rig.arm.step()
            assert counts(rig)==mark;detail['further_commands_blocked']=True
        history=servo.get('history',[])
        detail['correction_count']=len(history)
        late_history=[h for h in history if h.get('planned_at_s',h.get('capture_s',-1))>=motion.event['injected_s']]
        detail['corrections_after_onset']=len(late_history)
        detail['zero_no_progress_corrections']=sum(h.get('no_progress_steps')==0 for h in late_history)
        if phase=='DEPLOY':
            obs=motion.observations
            detail['observed_motor_timer_resets']=sum(
                a['seconds_without_progress'] is not None and b['seconds_without_progress'] is not None and
                b['seconds_without_progress']+0.001<a['seconds_without_progress'] and
                (b['best_m'] or 0)>(a['best_m'] or 0)
                for a,b in zip(obs,obs[1:]))
        if profile in ('deploy_time','deploy_iterations','deploy_travel','bend_time','bend_travel','cycle_time'):
            expected={'deploy_time':('VISUAL_SERVO_TIMEOUT',),'deploy_iterations':('VISUAL_SERVO_ITERATION_LIMIT',),
                'deploy_travel':('VISUAL_SERVO_TOTAL_TRAVEL_LIMIT',),'bend_time':('BEND_OPERATION_TIMEOUT',),
                'bend_travel':('BEND_TOTAL_TRAVEL_LIMIT',),'cycle_time':('CYCLE_TIME_LIMIT',)}[profile]
            assert any(code in result['message'] for code in expected),result
            assert detail['zero_no_progress_corrections']>=3,detail['zero_no_progress_corrections']
        base.save_rig(name,rig,detail)
    finally: rig.close()


def cases(base):
    result=[]
    def add(name,**kw): result.append((name,lambda n=name,k=kw:exercise(base,n,**k)))
    add('intermittent_deploy_actual_current')
    add('intermittent_deploy_false_current',false_feedback=True,period_s=0.025,step_m=0.00012)
    for profile in ('deploy_time','deploy_iterations','deploy_travel'):
        add('intermittent_'+profile,false_feedback=True,period_s=0.025,step_m=0.00012,profile=profile)
    add('intermittent_deploy_oscillate_actual',oscillate=True)
    add('intermittent_deploy_oscillate_false',false_feedback=True,period_s=0.025,step_m=0.00012,oscillate=True)
    add('intermittent_bend_actual_current',phase='BEND_1',period_s=0.08,step_m=0.00025)
    add('intermittent_bend_false_current',phase='BEND_1',period_s=0.025,step_m=0.00012,false_feedback=True)
    for profile in ('bend_time','bend_travel','cycle_time'):
        add('intermittent_'+profile,phase='BEND_1',period_s=0.025,step_m=0.00012,false_feedback=True,profile=profile)
    add('intermittent_bend_oscillate_actual',phase='BEND_1',period_s=0.08,step_m=0.00025,oscillate=True)
    add('intermittent_deploy_healthy',period_s=0.02,step_m=0.003,healthy=True)
    add('intermittent_bend_healthy',phase='BEND_1',period_s=0.025,step_m=0.0015,healthy=True)
    add('intermittent_deploy_noise',false_feedback=True,period_s=0.025,step_m=0.00012,profile='deploy_time',noise=0.00002)
    add('intermittent_bend_noise',phase='BEND_1',period_s=0.025,step_m=0.00012,false_feedback=True,profile='bend_time',noise=0.00002)
    return result
