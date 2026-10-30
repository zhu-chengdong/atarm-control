"""Schedule an extra physical pause only after severe primary evidence is seen."""
import importlib,json,math,random,threading,time
from dataclasses import replace
from recovery_cases import setup,counts
from intermittent_cases import options,PulseCamera
from late_jam_cases import verify_stop,RAW_TO_METRES
from early_progress_cases import paired as creep_pair


class ConfirmPauseMotion:
    def __init__(self,rig,raw,phase,delay_s,normal_rate):
        self.rig=rig;self.raw=raw;self.phase=phase;self.delay_s=delay_s;self.normal_rate=normal_rate
        self.lock=threading.Lock();self.event=None;self.trigger=None;self.steps=[];self.finished=False
        self.period_s=0.08;self.step_m=0.00020 if phase=='BEND_1' else 0.00065
        self.resumed_s=None;self.pause_samples=0
    def guard(self):
        return self.rig.arm.bend_controller.progress_window if self.phase=='BEND_1' else self.rig.arm.deploy_progress_window
    def observe(self):
        arm=self.rig.arm
        if not arm.cycle.active or arm.cycle.phase!=self.phase:return
        if self.event is None:
            p=self.raw.truth();progress=0.8*p[0]-0.6*p[1] if self.phase=='BEND_1' else 0.05-p[2] if self.phase=='RETRACT' else p[2]
            if progress<(0.002 if self.phase=='BEND_1' else 0.01):return
            with self.lock:
                self.board=self.rig.active if self.phase=='BEND_1' else self.rig.deploy
                self.ids=(1,0,2) if self.phase=='BEND_1' else (2,)
                self.original_modes=dict(self.board.motor_modes);self.targets=dict(self.board.position)
                for mid in self.ids:self.board.motor_modes[mid]='stall'
                now=time.monotonic();self.last_s=now;self.next_pulse=now+self.period_s
                self.event={'injected_s':now,'phase':self.phase,'pulse_period_s':self.period_s,'pulse_step_m':self.step_m}
        if self.trigger is None:
            g=self.guard()
            if not g.events:return
            primary=g.events[-1]
            lower=primary.get('progress_lower_bound_m',primary['progress_upper_bound_m'])
            if not primary['bad'] or lower>0.6*primary['required_reduction_m']:return
            now=time.monotonic();pending=g.severe_pending
            self.trigger={'armed_s':primary['last_capture_s'],'armed_frame':primary['last_frame'],
                'observed_s':now,'switch_s':now,'primary_ratio':primary['progress_upper_bound_m']/primary['required_reduction_m'],
                'actual_confirmation_pending':pending is not None,
                'primary_lower_m':lower,'primary_threshold_m':0.6*primary['required_reduction_m'],
                'pause_start_s':None if self.delay_s is None else now+self.delay_s,
                'pause_end_s':None if self.delay_s is None else now+self.delay_s+0.08,
                'normal_rate_m_s':self.normal_rate}
    def before_command(self,board,motor_id,target):
        with self.lock:
            if self.event is None or board is not self.board or self.finished:return
            arm=self.rig.arm
            if not arm.cycle.active or arm.cycle.phase!=self.phase:
                board.motor_modes=dict(self.original_modes);self.finished=True;return
            old_targets=dict(self.targets);self.targets[motor_id]=target
            if motor_id!=self.ids[0]:return
            now=time.monotonic();previous=self.last_s;self.last_s=now
            old=tuple(board.position[mid] for mid in self.ids)
            delta=tuple(old_targets[mid]-q for mid,q in zip(self.ids,old));largest=max(abs(v) for v in delta)
            active=paused=0.0;mode='initial_pulse_hold';limit=0.0
            if self.trigger is None:
                if now>=self.next_pulse:
                    self.next_pulse=now+self.period_s;limit=self.step_m;mode='initial_pulse'
            else:
                t=self.trigger;start=max(previous,t['switch_s']);dt=max(0,now-start)
                if t['pause_start_s'] is not None:paused=max(0,min(now,t['pause_end_s'])-max(start,t['pause_start_s']))
                active=max(0,dt-paused);limit=self.normal_rate*active;mode='recovery'
            scale=min(1.0,limit/RAW_TO_METRES/largest) if largest else 0.0
            updated=tuple(round(q+scale*d) for q,d in zip(old,delta))
            for mid,q in zip(self.ids,updated):board.position[mid]=q
            moved=max(abs(a-b) for a,b in zip(old,updated))*RAW_TO_METRES
            if self.trigger is not None and self.trigger['pause_start_s'] is not None:
                t=self.trigger
                if previous>=t['pause_start_s'] and now<=t['pause_end_s']:
                    self.pause_samples+=1;assert moved==0
                if now>=t['pause_end_s'] and moved>0 and self.resumed_s is None:self.resumed_s=now
            self.steps.append({'s':now,'previous_s':previous,'mode':mode,'active_dt_s':active,'paused_dt_s':paused,
                'limit_m':limit,'old_raw':old,'new_raw':updated,'max_move_m':moved})


def run(base,name,phase,delay_s,rate_kind,noise,fraction):
    lib=importlib.import_module('atarm_controller.arm_controller_lib');extra=options('current')
    extra['window_progress_config']=lib.WindowProgressConfig(window_s=0.4,severe_fraction=fraction)
    extra['pause_budget_config']=lib.PauseBudgetConfig()
    rig,raw,plan,ignored=setup(base,extra_options=extra)
    try:
        raw.fault_phase=None;raw.fault=None;raw.last=None;raw.noise=noise;raw.random=random.Random(53)
        rate=(0.03 if rate_kind=='fast' else 0.014) if phase=='BEND_1' else (0.16 if rate_kind=='fast' else 0.05)
        motion=ConfirmPauseMotion(rig,raw,phase,delay_s,rate);rig.arm.camera_feedback=PulseCamera(raw,motion)
        rig.arm._fault('SIMULATED_FAULT_BEFORE_CONFIRMATION_PAUSE')
        recovery=rig.arm.recover_camera_cycle(mechanically_confirmed=True,fault_cleared=True)
        probe=rig.arm.probe_status();assert recovery['success'] and probe['state']=='SUCCEEDED'
        rig.active.late_jam=rig.deploy.late_jam=motion
        result=rig.arm.run_camera_cycle(plan);returned=time.monotonic();cycle=rig.arm.cycle_status()
        stage=next(s for s in cycle['stages'] if s['phase']==phase)
        ctrl=stage['controller'] if phase=='BEND_1' else stage['controller']['visual']
        goal=rig.arm.camera_goals[False] if result['success'] or phase=='RETRACT' else plan.bend_goals[0] if phase=='BEND_1' else plan.neutral_goal
        error=math.sqrt(sum((a-b)**2 for a,b in zip(raw.truth(),goal.position_m)))
        d={'phase':phase,'delay_s':delay_s,'rate_kind':rate_kind,'noise_m':noise,'severe_fraction':fraction,
            'recovery':recovery,'probe':probe,'result':result,'cycle':cycle,'controller':ctrl,'injection':motion.event,
            'confirmation_trigger':motion.trigger,'steps':motion.steps,'pause_samples':motion.pause_samples,
            'resumed_s':motion.resumed_s,'root_state':rig.arm.deploy_state,'true_error_m':error,
            'phase_elapsed_s':stage['ended_s']-stage['started_s'],'returned_s':returned}
        base.save_rig(name,rig,d)
        assert motion.trigger is not None and motion.trigger['primary_lower_m']<=motion.trigger['primary_threshold_m']
        assert motion.trigger['actual_confirmation_pending']==bool(fraction)
        if delay_s is not None:
            assert motion.pause_samples>0
            assert motion.resumed_s is not None and motion.resumed_s>=motion.trigger['pause_end_s']
        if result['success']:
            assert result['complete'] and cycle['state']=='SUCCEEDED' and error<=0.001
        else:
            assert not result['complete'] and cycle['state']=='FAULT' and rig.arm.deploy_state=='FAULT'
            assert cycle['phase']==phase and error>0.0015
            d['stop_timing']=verify_stop(rig,cycle,motion.event['injected_s'])
            mark=counts(rig);assert not rig.arm.run_camera_cycle(plan)['success'];rig.arm.step()
            assert counts(rig)==mark;d['later_commands_blocked']=True
        base.save_rig(name,rig,d);return d
    finally:rig.close()


def pair(base,name,phase,delay_s,rate_kind,noise=0.0):
    off=run(base,name+'_original400',phase,delay_s,rate_kind,noise,0.0)
    on=run(base,name+'_confirmation',phase,delay_s,rate_kind,noise,0.6)
    assert off['result']['success'],off['result']
    d={'kind':'recovery','phase':phase,'delay_s':delay_s,'rate_kind':rate_kind,'noise_m':noise,
        'reference_success':off['result']['success'],'confirmation_success':on['result']['success'],
        'reference_phase_ms':off['phase_elapsed_s']*1000,'confirmation_phase_ms':on['phase_elapsed_s']*1000,
        'resumed_before_stop':on['resumed_s'] is not None,'reason':on['result']['message'],
        'trigger_mode':on['controller']['short_window_progress']['trigger_mode']}
    base.DETAILS[name]=d;(base.TRACE_ROOT/(name+'.json')).write_text(json.dumps(d,indent=2)+'\n')


def source_cases(base):
    from atarm_controller.progress_window import WindowProgressConfig,PauseBudgetConfig,GoalProgressWindow
    from atarm_controller.camera_feedback import CameraGoal,CameraPose
    rows=[]
    for mode,expected,fast,slow in [('deploy',0.2,0.16,0.05),('bend',0.04,0.03,0.01)]:
        for rate_kind,rate in [('fast',fast),('slow',slow)]:
            for delay in (None,0.02,0.06,0.1):
                for noise in (0.0,0.00002):
                    for fraction in (0.0,0.6):
                        cfg=PauseBudgetConfig().window(WindowProgressConfig(window_s=0.4,severe_fraction=fraction))
                        g=GoalProgressWindow(cfg);goal=CameraGoal('sim','tip','epoch',(0.3,0.0,0.0))
                        g.begin(goal,0.0005,0.0,expected);rng=random.Random(53);x=0.0;trigger=None;failure=None
                        for i in range(121):
                            t=i*0.01;previous=max(0,t-0.01)
                            if trigger is not None:
                                overlap=0 if delay is None else max(0,min(t,trigger+delay+0.08)-max(previous,trigger+delay))
                                x+=rate*max(0,t-previous-overlap)
                            p=(x+rng.uniform(-noise,noise),rng.uniform(-noise,noise),rng.uniform(-noise,noise))
                            try:g.observe(CameraPose(i+1,t,'sim','tip','epoch',True,p))
                            except ValueError as exc:failure={'s':t,'reason':str(exc)};break
                            if trigger is None and g.events and g.events[-1]['bad']:
                                e=g.events[-1]
                                if e['progress_upper_bound_m']<=0.6*e['required_reduction_m']:trigger=t
                        assert trigger is not None
                        if fraction==0 or delay is None or rate_kind=='fast':assert failure is None
                        rows.append({'mode':mode,'rate_kind':rate_kind,'delay_s':delay,'noise_m':noise,'severe_fraction':fraction,
                            'trigger_s':trigger,'failure':failure,'window':g.status()})
    d={'scope':'source-time window replay only','rows':rows}
    base.DETAILS['confirm_source_matrix']=d;(base.TRACE_ROOT/'confirm_source_matrix.json').write_text(json.dumps(d,indent=2)+'\n')


def cases(base):
    result=[]
    def add(name,**kw):result.append((name,lambda n=name,k=kw:pair(base,n,**k)))
    for phase,label in [('DEPLOY','deploy'),('BEND_1','bend'),('RETRACT','retract')]:
        for rate_kind in ('fast','slow'):
            for delay,tail in [(None,'no_pause'),(0.02,'start'),(0.06,'middle'),(0.1,'late')]:
                add('confirm_'+label+'_'+rate_kind+'_'+tail,phase=phase,delay_s=delay,rate_kind=rate_kind)
        add('confirm_'+label+'_slow_noise',phase=phase,delay_s=0.06,rate_kind='slow',noise=0.00002)
    for phase,label in [('DEPLOY','deploy'),('BEND_1','bend')]:
        for noise,tail in [(0.0,''),(0.00002,'_noise')]:
            name='confirm_creep_'+label+tail
            result.append((name,lambda n=name,p=phase,v=noise:creep_pair(base,n,p,'pulse',noise=v)))
    result.append(('confirm_source_matrix',lambda:source_cases(base)))
    return result
