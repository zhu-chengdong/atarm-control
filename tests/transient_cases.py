"""Continuous physical speed profiles; no profile flag reaches the controller."""
from dataclasses import replace
from copy import deepcopy
import importlib,json,math,random,threading,time
from recovery_cases import setup,counts
from intermittent_cases import options,PulseCamera
from late_jam_cases import verify_stop,RAW_TO_METRES


class TransientMotion:
    def __init__(self,rig,raw,phase,kind,pause_s=0.04,false_feedback=False):
        self.rig=rig;self.raw=raw;self.phase=phase;self.kind=kind;self.pause_s=pause_s
        self.false_feedback=false_feedback;self.event=None;self.finished=False;self.targets={}
        self.steps=[];self.resumed_s=None;self.lock=threading.Lock()
        self.initial_rate=0.05 if phase=='BEND_1' else 0.16
        self.final_rate=(0.012 if phase=='BEND_1' else 0.035) if kind=='ramp_ok' else 0.001 if kind=='ramp_creep' else 0.0
        self.ramp_s=0.3 if kind=='ramp_ok' else 0.2
    def observe(self):
        arm=self.rig.arm
        if self.event is not None or not arm.cycle.active or arm.cycle.phase!=self.phase: return
        p=self.raw.truth()
        progress=(0.05-p[2] if self.phase=='RETRACT' else p[2]) if self.phase!='BEND_1' else 0.8*p[0]-0.6*p[1]
        if progress<(0.002 if self.phase=='BEND_1' else 0.01): return
        with self.lock:
            self.board=self.rig.active if self.phase=='BEND_1' else self.rig.deploy
            self.ids=(1,0,2) if self.phase=='BEND_1' else (2,)
            self.original_modes=dict(self.board.motor_modes);self.targets=dict(self.board.position)
            now=time.monotonic();self.last_s=now
            for mid in self.ids: self.board.motor_modes[mid]='coherent_false' if self.false_feedback else 'stall'
            self.event={'injected_s':now,'phase':self.phase,'kind':self.kind,'pause_s':self.pause_s,
                'initial_rate_m_s':self.initial_rate,'final_rate_m_s':self.final_rate,'ramp_s':self.ramp_s,
                'false_feedback':self.false_feedback,'position_raw_at_onset':dict(self.board.position),
                'scheduled_resume_s':now+self.pause_s if self.kind=='pause' else None,
                'scheduled_ramp_end_s':now+self.ramp_s if self.kind!='pause' else None}
    def rate(self,elapsed):
        if self.kind=='jam': return 0.0
        if self.kind=='pause': return 0.0 if elapsed<self.pause_s else self.initial_rate
        fraction=min(1.0,max(0.0,elapsed/self.ramp_s))
        return self.initial_rate+(self.final_rate-self.initial_rate)*fraction
    def before_command(self,board,motor_id,target):
        with self.lock:
            if self.event is None or board is not getattr(self,'board',None) or self.finished: return
            arm=self.rig.arm
            if not arm.cycle.active or arm.cycle.phase!=self.phase:
                board.motor_modes=dict(self.original_modes);self.finished=True;return
            old_targets=dict(self.targets);self.targets[motor_id]=target
            if motor_id!=self.ids[0]: return
            now=time.monotonic();dt=max(0.0,now-self.last_s);self.last_s=now
            elapsed=now-self.event['injected_s'];rate=self.rate(elapsed)
            old=tuple(board.position[mid] for mid in self.ids)
            delta=tuple(old_targets[mid]-q for mid,q in zip(self.ids,old))
            largest=max(abs(v) for v in delta)
            scale=min(1.0,rate*dt/RAW_TO_METRES/largest) if largest else 0.0
            updated=tuple(round(q+scale*d) for q,d in zip(old,delta))
            for mid,q in zip(self.ids,updated): board.position[mid]=q
            move=max(abs(a-b) for a,b in zip(old,updated))*RAW_TO_METRES
            if self.kind=='pause' and elapsed>=self.pause_s and move>0 and self.resumed_s is None: self.resumed_s=now
            self.steps.append({'s':now,'dt_s':dt,'rate_m_s':rate,'old_raw':old,'new_raw':updated,'max_move_m':move})


def run(base,name,phase='DEPLOY',kind='pause',pause_s=0.04,noise=0.0,fake=False,enabled=True,expected=True,coordinated=False,window_s=None,severe_fraction=0.0):
    extra=options('current');lib=importlib.import_module('atarm_controller.arm_controller_lib')
    if enabled: extra['window_progress_config']=lib.WindowProgressConfig(window_s=0.12 if window_s is None else window_s,severe_fraction=severe_fraction)
    if coordinated: extra['pause_budget_config']=lib.PauseBudgetConfig()
    rig,raw,plan,ignored=setup(base,extra_options=extra)
    try:
        raw.fault_phase=None;raw.fault=None;raw.last=None;raw.noise=noise;raw.random=random.Random(31)
        motion=TransientMotion(rig,raw,phase,kind,pause_s,fake)
        rig.arm.camera_feedback=PulseCamera(raw,motion)
        rig.arm._fault('SIMULATED_FAULT_BEFORE_TRANSIENT_TEST')
        recovery=rig.arm.recover_camera_cycle(mechanically_confirmed=True,fault_cleared=True)
        assert recovery['success'] and not recovery['complete'],recovery
        probe=rig.arm.probe_status();assert probe['state']=='SUCCEEDED'
        rig.active.late_jam=rig.deploy.late_jam=motion
        result=rig.arm.run_camera_cycle(plan);returned=time.monotonic();cycle=rig.arm.cycle_status()
        stage=next(s for s in cycle['stages'] if s['phase']==phase)
        ctrl=stage['controller'] if phase=='BEND_1' else stage['controller']['visual']
        guard=ctrl.get('short_window_progress',{'enabled':False})
        goal=rig.arm.camera_goals[False] if result['success'] or phase=='RETRACT' else plan.bend_goals[0] if phase=='BEND_1' else plan.neutral_goal
        error=math.sqrt(sum((a-b)**2 for a,b in zip(raw.truth(),goal.position_m)))
        detail={'phase':phase,'kind':kind,'expected_success':expected,'enabled':enabled,'noise_m':noise,
            'coordinated':coordinated,'pause_budget':ctrl.get('pause_budget',{'enabled':False}),
            'recovery':recovery,'probe':probe,'result':result,'cycle':cycle,'window':guard,
            'profile':motion.event,'steps':motion.steps,'resumed_s':motion.resumed_s,'true_error_m':error,
            'root_state':rig.arm.deploy_state,'classification':'completed' if result['success'] else
                'stopped_scheduled_recovery' if expected is None else 'fault_stopped',
            'onset_to_return_ms':None if motion.event is None else (returned-motion.event['injected_s'])*1000}
        base.save_rig(name,rig,detail)
        assert motion.event and len(motion.steps)>=2,(name,result)
        assert any(s['max_move_m']>0 for s in motion.steps) or kind in ('pause','jam')
        if result['success']:
            assert result['complete'] and cycle['state']=='SUCCEEDED' and error<=0.001,result
            if enabled: assert guard['state']=='COMPLETED' and not guard['fault']
            if kind=='pause': assert motion.resumed_s is not None
        else:
            assert not result['complete'] and cycle['state']=='FAULT' and rig.arm.deploy_state=='FAULT'
            assert error>0.0015 and cycle['phase']==phase,(error,result)
            assert not cycle['stages'][-1]['verified'] and cycle['stages'][-1]['phase']==phase
            detail['stop_timing']=verify_stop(rig,cycle,motion.event['injected_s'])
            mark=counts(rig);assert not rig.arm.run_camera_cycle(plan)['success'];rig.arm.step()
            assert counts(rig)==mark;detail['later_commands_blocked']=True
            assert detail['onset_to_return_ms']<(850 if enabled else 2300),result
        base.save_rig(name,rig,detail)
        if expected is not None: assert result['success']==expected,result
        return detail
    finally: rig.close()


def pair(base,name,phase):
    off=run(base,name+'_baseline',phase=phase,kind='ramp_creep',fake=True,enabled=False,expected=False)
    on=run(base,name+'_enabled',phase=phase,kind='ramp_creep',fake=True,enabled=True,expected=False)
    comparison={'baseline_ms':off['onset_to_return_ms'],'enabled_ms':on['onset_to_return_ms'],
        'baseline_reason':off['result']['message'],'enabled_reason':on['result']['message']}
    base.DETAILS[name]=comparison
    (base.TRACE_ROOT/(name+'.json')).write_text(json.dumps(comparison,indent=2)+'\n')


def source_grid(base):
    from atarm_controller.progress_window import WindowProgressConfig,GoalProgressWindow
    from atarm_controller.camera_feedback import CameraGoal,CameraPose
    rows=[]
    for kind in ('pause','ramp_ok','ramp_zero'):
        for duration in ((0.04,0.08,0.12,0.18,0.3) if kind=='pause' else (0.3,)):
            for onset in (0.17,0.205,0.24):
                for noise in (0.0,0.00002):
                    guard=GoalProgressWindow(WindowProgressConfig());goal=CameraGoal('sim','tip','epoch',(0.3,0.0,0.0))
                    guard.begin(goal,0.0005,0.0,0.2);rng=random.Random(31);x=0.0;failed=None
                    for i in range(101):
                        t=i*0.01;elapsed=max(0.0,t-onset)
                        if t<onset: rate=0.12
                        elif kind=='pause': rate=0.0 if elapsed<duration else 0.12
                        else: rate=0.12+((0.035 if kind=='ramp_ok' else 0.0)-0.12)*min(1.0,elapsed/duration)
                        if i: x+=rate*0.01
                        p=(x+rng.uniform(-noise,noise),rng.uniform(-noise,noise),rng.uniform(-noise,noise))
                        try: guard.observe(CameraPose(i+1,t,'sim','tip','epoch',True,p))
                        except ValueError as exc: failed={'s':t,'reason':str(exc)};break
                    row={'kind':kind,'pause_or_ramp_s':duration,'onset_s':onset,'noise_m':noise,'failed':failed,'window':guard.status()}
                    if kind=='ramp_ok' or kind=='pause' and duration<=0.04: assert failed is None,row
                    if kind=='ramp_zero' or kind=='pause' and duration>=0.3:
                        assert failed and failed['reason']=='WINDOW_PROGRESS_TOO_SMALL' and failed['s']-onset<0.6,row
                    rows.append(row)
    detail={'scope':'exact source-time replay of the unchanged window policy only, not motors or full cycle',
        'rows':rows,'normal_pause_stops':sum(r['failed'] is not None for r in rows if r['kind']=='pause' and r['pause_or_ramp_s']<0.3)}
    base.DETAILS['transient_source_grid']=detail
    (base.TRACE_ROOT/'transient_source_grid.json').write_text(json.dumps(detail,indent=2)+'\n')


def cases(base):
    result=[]
    def add(name,**kw): result.append((name,lambda n=name,k=kw:run(base,n,**k)))
    for phase,label in (('DEPLOY','deploy'),('BEND_1','bend'),('RETRACT','retract')):
        for duration in (0.03,0.04): add('transient_'+label+'_pause_'+str(round(duration*1000)),phase=phase,pause_s=duration)
        add('transient_'+label+'_pause_noise',phase=phase,noise=0.00002)
        add('transient_'+label+'_decelerate_ok',phase=phase,kind='ramp_ok')
        add('transient_'+label+'_pause80_boundary',phase=phase,pause_s=0.08,expected=None)
        add('transient_'+label+'_pause300_fault',phase=phase,pause_s=0.3,expected=False)
        add('transient_'+label+'_decelerate_zero',phase=phase,kind='ramp_zero',expected=False)
        add('transient_'+label+'_decelerate_creep',phase=phase,kind='ramp_creep',expected=False)
        name='transient_'+label+'_false_pair'
        result.append((name,lambda n=name,p=phase:pair(base,n,p)))
    result.append(('transient_source_grid',lambda:source_grid(base)))
    return result
