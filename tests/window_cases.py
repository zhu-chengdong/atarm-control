"""Paired window-policy tests and legitimate slow-motion controls."""
from dataclasses import replace
import importlib,json,math,random,time,types
from recovery_cases import setup,counts
from intermittent_cases import PulseMotion,PulseCamera,options
from late_jam_cases import verify_stop
from atarm_controller.camera_cycle import CyclePlan


def run(base,name,phase='DEPLOY',fake=False,osc=False,healthy=False,pulse=True,enabled=True,
        period=0.04,step=0.00065,noise=0.0,planned=None,near=False,seed=19,node=False,coordinated=False,
        window_s=None,require_window_fault=True,severe_fraction=0.0):
    extra=options('current');lib=importlib.import_module('atarm_controller.arm_controller_lib')
    if enabled: extra['window_progress_config']=lib.WindowProgressConfig(window_s=0.12 if window_s is None else window_s,severe_fraction=severe_fraction)
    if coordinated: extra['pause_budget_config']=lib.PauseBudgetConfig()
    rig,raw,plan,ignored=setup(base,extra_options=extra)
    try:
        raw.fault_phase=None;raw.fault=None;raw.last=None;raw.noise=noise;raw.random=random.Random(seed)
        if planned=='deploy':
            rig.arm.deploy_speed_m_s=0.04;rig.arm.deploy_delta_length=0.012
            rig.arm.camera_goals[True]=replace(rig.arm.camera_goals[True],position_m=(0.0,0.0,0.012))
            neutral=rig.arm.camera_goals[True]
            plan=CyclePlan(neutral,(replace(neutral,position_m=(0.0024,-0.0018,0.012)),))
        if planned=='bend': rig.arm.bend_controller.cfg=replace(rig.arm.bend_controller.cfg,speed_m_s=0.15)
        if planned=='small_step': rig.arm.bend_controller.cfg=replace(rig.arm.bend_controller.cfg,max_step_m=0.0008)
        if near: plan=CyclePlan(plan.neutral_goal,(replace(plan.neutral_goal,position_m=(0.0012,0.0,plan.neutral_goal.position_m[2])),))
        raw.plan=plan
        motion=PulseMotion(rig,raw,phase,period,step,fake,osc)
        rig.arm.camera_feedback=PulseCamera(raw,motion) if pulse else raw
        rig.arm._fault('SIMULATED_FAULT_BEFORE_WINDOW_TEST')
        recovery=rig.arm.recover_camera_cycle(mechanically_confirmed=True,fault_cleared=True)
        assert recovery['success'] and not recovery['complete'],recovery
        passed_probe=rig.arm.probe_status()
        if pulse: rig.active.late_jam=rig.deploy.late_jam=motion
        if node:
            module=importlib.import_module('atarm_controller.arm_controller_node')
            obj=object.__new__(module.UnitreeTensegrityControllerNode);obj.arm=rig.arm;obj.cycle_plan=plan
            response=types.SimpleNamespace(success=False,message='')
            obj._cycle_service(None,response);result=rig.arm.cycle_status()['result'];assert response.success==result['success']
        else: result=rig.arm.run_camera_cycle(plan)
        returned=time.monotonic();cycle=rig.arm.cycle_status()
        stage=next(s for s in cycle['stages'] if s['phase']==phase)
        ctrl=stage['controller'].get('visual',{}) if phase=='DEPLOY' else stage['controller']
        guard=ctrl.get('short_window_progress',{'enabled':False})
        goal=rig.arm.camera_goals[False] if healthy else rig.arm.camera_goals[True] if phase=='DEPLOY' else plan.bend_goals[0]
        error=math.sqrt(sum((a-b)**2 for a,b in zip(raw.truth(),goal.position_m)))
        detail={'enabled':enabled,'phase':phase,'healthy':healthy,'planned':planned,'seed':seed,'noise_m':noise,
            'recovery':recovery,'passed_probe':passed_probe,'result':result,'cycle':cycle,'window':guard,
            'injection':motion.event,'pulses':motion.pulses,'true_error_m':error,'root_state':rig.arm.deploy_state}
        base.save_rig(name,rig,detail)
        if healthy:
            assert result['success'] and result['complete'] and error<=0.001,result
            assert guard.get('fault') is None,guard
            if enabled: assert guard['state']=='COMPLETED'
        else:
            assert motion.event and len(motion.pulses)>=1,(name,result)
            assert not result['success'] and not result['complete'] and rig.arm.deploy_state=='FAULT',result
            assert error>0.0015,error
            if enabled and require_window_fault and not (osc and phase=='DEPLOY'):
                assert 'WINDOW_PROGRESS_TOO_SMALL' in result['message'],result
                assert guard['state']=='FAULT' and guard['bad_count']==guard['limits']['bad_windows']
            detail['stop_timing']=verify_stop(rig,cycle,motion.event['injected_s'])
            detail['onset_to_failure_ms']=(returned-motion.event['injected_s'])*1000
            assert not any(s['phase'] in ('NEUTRALIZE','RETRACT','VERIFY_END') for s in cycle['stages'])
            mark=counts(rig);assert not rig.arm.run_camera_cycle(plan)['success'];rig.arm.step()
            assert counts(rig)==mark;detail['later_commands_blocked']=True
        base.save_rig(name,rig,detail);return detail
    finally: rig.close()


def paired(base,name,**kw):
    off=run(base,name+'_baseline',enabled=False,**kw)
    on=run(base,name+'_enabled',enabled=True,**kw)
    comparison={'baseline_ms':off['onset_to_failure_ms'],'enabled_ms':on['onset_to_failure_ms'],
        'baseline_reason':off['result']['message'],'enabled_reason':on['result']['message']}
    assert comparison['enabled_ms']<comparison['baseline_ms']-20,comparison
    base.DETAILS[name]=comparison
    (base.TRACE_ROOT/(name+'.json')).write_text(json.dumps(comparison,indent=2)+'\n')


def unit_checks(base):
    from atarm_controller.progress_window import WindowProgressConfig,GoalProgressWindow
    from atarm_controller.camera_feedback import CameraGoal,CameraPose
    goal=CameraGoal('sim','tip','epoch',(0.05,0.0,0.0));cfg=WindowProgressConfig()
    g=GoalProgressWindow(cfg);g.begin(goal,0.001,0.0,0.2)
    p=CameraPose(1,0.03,'sim','tip','epoch',True,(0.0,0.0,0.0));g.observe(p)
    for _ in range(1000): g.observe(p)
    assert not g.events and g.bad_count==0
    tiny=GoalProgressWindow(cfg);tiny.begin(goal,0.001,0.0,0.000001)
    for i in range(65): tiny.observe(replace(p,frame_id=i+1,capture_s=0.03+i*0.01))
    assert tiny.state in ('UNRESOLVED','RESOLUTION_WAIT') and not tiny.fault
    assert tiny.events and not any(e['resolved'] for e in tiny.events)
    base.DETAILS['window_source_and_resolution']={'cached_poll_events':len(g.events),'low_signal':tiny.status()}
    (base.TRACE_ROOT/'window_source_and_resolution.json').write_text(json.dumps(base.DETAILS['window_source_and_resolution'],indent=2)+'\n')


def cases(base):
    result=[]
    def add(name,**kw): result.append((name,lambda n=name,k=kw:run(base,n,**k)))
    for phase,label in (('DEPLOY','deploy'),('BEND_1','bend')):
        for fake in (False,True):
            name='window_pair_'+label+('_false' if fake else '_actual')
            kw={'phase':phase,'fake':fake,'period':0.025 if fake else 0.04 if phase=='DEPLOY' else 0.08,
                'step':0.00012 if fake else 0.00065 if phase=='DEPLOY' else 0.00025}
            result.append((name,lambda n=name,k=kw:paired(base,n,**k)))
    result.append(('window_pair_bend_oscillate',lambda:paired(base,'window_pair_bend_oscillate',phase='BEND_1',period=0.08,step=0.00025,osc=True)))
    add('window_deploy_oscillate',osc=True)
    add('window_deploy_noise',fake=True,period=0.025,step=0.00012,noise=0.00002)
    add('window_bend_noise',phase='BEND_1',fake=True,period=0.025,step=0.00012,noise=0.00002)
    add('window_normal',healthy=True,pulse=False)
    add('window_slow_deploy',healthy=True,period=0.04,step=0.0015)
    add('window_slow_bend',phase='BEND_1',healthy=True,period=0.04,step=0.0008)
    add('window_planned_deploy',healthy=True,pulse=False,planned='deploy')
    add('window_planned_bend',phase='BEND_1',healthy=True,pulse=False,planned='bend')
    add('window_small_steps',phase='BEND_1',healthy=True,pulse=False,planned='small_step')
    add('window_near_goal_noise',phase='BEND_1',healthy=True,pulse=False,near=True,noise=0.00002)
    for seed in (7,41): add('window_normal_noise_'+str(seed),phase='BEND_1',healthy=True,pulse=False,noise=0.00002,seed=seed)
    add('window_node',healthy=True,pulse=False,node=True)
    result.append(('window_source_and_resolution',lambda:unit_checks(base)))
    return result
