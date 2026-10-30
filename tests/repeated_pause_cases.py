"""Repeated physical pauses with exact active-time integration, no controller changes."""
from dataclasses import replace
import importlib,json,math,random,time
from transient_cases import TransientMotion
from intermittent_cases import options,PulseCamera
from recovery_cases import setup,counts
from cycle_cases import sequence_config
from late_jam_cases import verify_stop,RAW_TO_METRES


class RepeatedMotion(TransientMotion):
    def __init__(self,rig,raw,phase,repetitions,move_s,normal_rate):
        super().__init__(rig,raw,phase,'repeated_pause',0.08)
        self.repetitions=repetitions;self.move_s=move_s;self.initial_rate=normal_rate;self.pauses=[]
    def observe(self):
        was=self.event is not None;super().observe()
        if not was and self.event is not None:
            self.event.update({'repetitions':self.repetitions,'move_s':self.move_s,'normal_rate_m_s':self.initial_rate})
            start=self.event['injected_s'];period=self.pause_s+self.move_s
            self.pauses=[{'index':j,'start_s':start+j*period,'end_s':start+j*period+self.pause_s,
                'holding_samples':0,'resumed_sample_s':None} for j in range(self.repetitions)]
    def before_command(self,board,motor_id,target):
        with self.lock:
            if self.event is None or board is not getattr(self,'board',None) or self.finished: return
            arm=self.rig.arm
            if not arm.cycle.active or arm.cycle.phase!=self.phase:
                board.motor_modes=dict(self.original_modes);self.finished=True;return
            old_targets=dict(self.targets);self.targets[motor_id]=target
            if motor_id!=self.ids[0]: return
            now=time.monotonic();previous=self.last_s;self.last_s=now;dt=now-previous
            overlap=sum(max(0.0,min(now,p['end_s'])-max(previous,p['start_s'])) for p in self.pauses)
            active=max(0.0,dt-overlap)
            old=tuple(board.position[mid] for mid in self.ids)
            delta=tuple(old_targets[mid]-q for mid,q in zip(self.ids,old));largest=max(abs(v) for v in delta)
            fraction=min(1.0,self.initial_rate*active/RAW_TO_METRES/largest) if largest else 0.0
            updated=tuple(round(q+fraction*d) for q,d in zip(old,delta))
            for mid,q in zip(self.ids,updated): board.position[mid]=q
            moved=max(abs(a-b) for a,b in zip(old,updated))*RAW_TO_METRES
            for j,p in enumerate(self.pauses):
                if previous>=p['start_s'] and now<=p['end_s']:
                    p['holding_samples']+=1;assert moved==0
                next_start=self.pauses[j+1]['start_s'] if j+1<len(self.pauses) else float('inf')
                if moved>0 and now>=p['end_s'] and previous<next_start and p['resumed_sample_s'] is None:
                    if min(now,next_start)>max(previous,p['end_s']): p['resumed_sample_s']=now
            self.steps.append({'s':now,'previous_s':previous,'dt_s':dt,'active_dt_s':active,'paused_dt_s':overlap,
                'normal_rate_m_s':self.initial_rate,'old_raw':old,'new_raw':updated,'max_move_m':moved})


def run(base,name,phase='DEPLOY',repetitions=6,move_s=0.06,noise=0.0,
        dense=False,wide=False,deadline=False,cycle_deadline=False,severe_fraction=0.0):
    lib=importlib.import_module('atarm_controller.arm_controller_lib');extra=options('current')
    extra['window_progress_config']=lib.WindowProgressConfig(window_s=0.4 if wide else 0.12,severe_fraction=severe_fraction)
    extra['pause_budget_config']=lib.PauseBudgetConfig()
    if cycle_deadline: extra['cycle_config']=sequence_config(timeout_s=1.0)
    rig,raw,plan,ignored=setup(base,extra_options=extra)
    try:
        raw.fault_phase=None;raw.fault=None;raw.last=None;raw.noise=noise;raw.random=random.Random(53)
        if deadline or cycle_deadline:
            if phase=='BEND_1': rig.arm.bend_controller.cfg=replace(rig.arm.bend_controller.cfg,speed_m_s=0.06)
            else: rig.arm.deploy_speed_m_s=0.1
        normal_rate=(0.015 if phase=='BEND_1' else 0.16) if deadline or cycle_deadline else (
            (0.03 if phase=='BEND_1' else 0.16) if dense else (0.02 if phase=='BEND_1' else 0.08))
        motion=RepeatedMotion(rig,raw,phase,repetitions,move_s,normal_rate)
        rig.arm.camera_feedback=PulseCamera(raw,motion)
        rig.arm._fault('SIMULATED_FAULT_BEFORE_REPEATED_PAUSE')
        recovery=rig.arm.recover_camera_cycle(mechanically_confirmed=True,fault_cleared=True)
        probe=rig.arm.probe_status();assert recovery['success'] and probe['state']=='SUCCEEDED'
        rig.active.late_jam=rig.deploy.late_jam=motion
        result=rig.arm.run_camera_cycle(plan);returned=time.monotonic();cycle=rig.arm.cycle_status()
        stage=next(s for s in cycle['stages'] if s['phase']==phase)
        # A whole-cycle deadline can unwind the stage call before its normal
        # controller snapshot is assigned. Preserve that fact and read the
        # actual post-stop status; do not fabricate a returned stage reply.
        snapshot_after_exception='controller' not in stage
        if snapshot_after_exception:
            ctrl=rig.arm.bend_status() if phase=='BEND_1' else rig.arm.visual_servo_status()
        else:
            ctrl=stage['controller'] if phase=='BEND_1' else stage['controller']['visual']
        goal=rig.arm.camera_goals[False] if result['success'] or phase=='RETRACT' else plan.bend_goals[0] if phase=='BEND_1' else plan.neutral_goal
        error=math.sqrt(sum((a-b)**2 for a,b in zip(raw.truth(),goal.position_m)))
        resumed=[p for p in motion.pauses if p['resumed_sample_s'] is not None]
        detail={'phase':phase,'dense':dense,'wide':wide,'deadline':deadline,'cycle_deadline':cycle_deadline,
            'noise_m':noise,'recovery':recovery,'probe':probe,'result':result,'cycle':cycle,
            'controller':ctrl,'profile':motion.event,'pauses':motion.pauses,'steps':motion.steps,
            'controller_snapshot_after_exception':snapshot_after_exception,
            'resumed_count':len(resumed),'completed_pause_s':len(resumed)*0.08,
            'true_error_m':error,'root_state':rig.arm.deploy_state,'returned_s':returned,
            'cycle_started_s':rig.arm.cycle.started,'phase_started_s':stage['started_s'],
            'cycle_elapsed_s':returned-rig.arm.cycle.started,
            'phase_ended_s':stage.get('ended_s',returned),
            'phase_elapsed_s':stage.get('ended_s',returned)-stage['started_s'],
            'action_limit_s':2.0 if phase=='BEND_1' else 1.4}
        base.save_rig(name,rig,detail)
        assert motion.event and len(resumed)>=2,(len(resumed),result)
        assert all(p['holding_samples']>0 for p in resumed)
        if result['success']:
            assert result['complete'] and cycle['state']=='SUCCEEDED' and error<=0.001
            assert len(resumed)==repetitions,(len(resumed),repetitions)
            assert not deadline and not cycle_deadline,result
        else:
            assert not result['complete'] and cycle['state']=='FAULT' and rig.arm.deploy_state=='FAULT'
            assert cycle['phase']==phase and not cycle['stages'][-1]['verified']
            assert error>0.0015,error
            detail['stop_timing']=verify_stop(rig,cycle,motion.event['injected_s'])
            mark=counts(rig);assert not rig.arm.run_camera_cycle(plan)['success'];rig.arm.step()
            assert counts(rig)==mark;detail['later_commands_blocked']=True
        base.save_rig(name,rig,detail)
        if cycle_deadline:
            assert 'CYCLE_TIME_LIMIT' in result['message'],result
            assert 1.0<=detail['cycle_elapsed_s']<1.12,detail['cycle_elapsed_s']
        elif deadline:
            code='BEND_OPERATION_TIMEOUT' if phase=='BEND_1' else 'VISUAL_SERVO_TIMEOUT'
            assert code in result['message'],result
            assert detail['action_limit_s']<=detail['phase_elapsed_s']<detail['action_limit_s']+0.12
        elif wide or not dense:
            assert result['success'],result
        else:
            assert result['success'] or 'WINDOW_PROGRESS_TOO_SMALL' in result['message'],result
        return detail
    finally: rig.close()


def source_grid(base):
    from atarm_controller.progress_window import WindowProgressConfig,PauseBudgetConfig,GoalProgressWindow
    from atarm_controller.camera_feedback import CameraGoal,CameraPose
    rows=[]
    for mode,width,gap,rate,count in [('regular',0.12,0.06,0.08,6),('dense',0.12,0.01,0.16,8),('dense_wide',0.4,0.01,0.16,8)]:
        for onset in (0.17,0.205,0.24):
            for noise in (0.0,0.00002):
                cfg=PauseBudgetConfig().window(WindowProgressConfig(window_s=width))
                guard=GoalProgressWindow(cfg);goal=CameraGoal('sim','tip','epoch',(0.3,0.0,0.0))
                guard.begin(goal,0.0005,0.0,0.2);rng=random.Random(53);x=0.0;failure=None
                for i in range(161):
                    t=i*0.01;previous=max(0.0,t-0.01)
                    overlap=sum(max(0.0,min(t,onset+j*(0.08+gap)+0.08)-max(previous,onset+j*(0.08+gap))) for j in range(count))
                    if i: x+=rate*max(0.0,t-previous-overlap)
                    p=(x+rng.uniform(-noise,noise),rng.uniform(-noise,noise),rng.uniform(-noise,noise))
                    try: guard.observe(CameraPose(i+1,t,'sim','tip','epoch',True,p))
                    except ValueError as exc: failure={'s':t,'reason':str(exc)};break
                row={'mode':mode,'window_s':cfg.window_s,'onset_s':onset,'noise_m':noise,'failure':failure,'window':guard.status()}
                rows.append(row)
                if mode!='dense': assert failure is None,row
    assert any(r['failure'] is not None for r in rows if r['mode']=='dense')
    detail={'scope':'separate exact-source-time replay, not the motor/camera full cycle','rows':rows}
    base.DETAILS['repeat_source_grid']=detail
    (base.TRACE_ROOT/'repeat_source_grid.json').write_text(json.dumps(detail,indent=2)+'\n')


def cases(base):
    result=[]
    def add(name,**kw): result.append((name,lambda n=name,k=kw:run(base,n,**k)))
    for phase,label in (('DEPLOY','deploy'),('BEND_1','bend'),('RETRACT','retract')):
        for count in (3,6): add('repeat_'+label+'_'+str(count),phase=phase,repetitions=count)
        add('repeat_'+label+'_noise',phase=phase,noise=0.00002)
        add('repeat_'+label+'_dense',phase=phase,repetitions=8,move_s=0.01,dense=True)
        add('repeat_'+label+'_dense_wide',phase=phase,repetitions=8,move_s=0.01,dense=True,wide=True)
        add('repeat_'+label+'_action_limit',phase=phase,repetitions=100,move_s=0.015,deadline=True)
    add('repeat_cycle_limit',repetitions=100,move_s=0.015,cycle_deadline=True)
    result.append(('repeat_source_grid',lambda:source_grid(base)))
    return result
