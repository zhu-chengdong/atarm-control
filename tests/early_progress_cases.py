"""Matched early-confirmation tests plus source-phase/noise robustness controls."""
from dataclasses import replace
import json,math,random
import transient_cases,window_cases,repeated_pause_cases
from wide400_cases import metrics
from atarm_controller.progress_window import WindowProgressConfig,PauseBudgetConfig,GoalProgressWindow
from atarm_controller.camera_feedback import CameraGoal,CameraPose


def paired(base,name,phase,family,noise=0.0,fake=False):
    results=[]
    for fraction,suffix in ((0.0,'_baseline'),(0.6,'_optimized')):
        raw_name=name+suffix
        if family=='dense':
            d=repeated_pause_cases.run(base,raw_name,phase=phase,repetitions=8,move_s=0.01,
                dense=True,wide=True,noise=noise,severe_fraction=fraction)
            assert d['result']['success'] and d['resumed_count']==8
        elif family in ('action','cycle'):
            d=repeated_pause_cases.run(base,raw_name,phase=phase,repetitions=100,move_s=0.015,
                wide=True,deadline=family=='action',cycle_deadline=family=='cycle',severe_fraction=fraction)
        elif family=='pulse':
            d=window_cases.run(base,raw_name,phase=phase,period=0.08,step=0.00065 if phase=='DEPLOY' else 0.00025,
                coordinated=True,window_s=0.4,require_window_fault=False,severe_fraction=fraction,noise=noise)
        else:
            d=transient_cases.run(base,raw_name,phase=phase,kind=family,fake=fake,expected=False,
                coordinated=True,window_s=0.4,severe_fraction=fraction)
        ctrl,elapsed,basis=metrics(d,family);g=ctrl['short_window_progress']
        assert g['limits']['window_s']==0.4 and g['limits']['severe_fraction']==fraction
        results.append((d,ctrl,elapsed,basis))
    before,a,old_ms,basis=results[0];after,b,new_ms,new_basis=results[1]
    assert basis==new_basis
    x=dict(a['short_window_progress']['limits']);y=dict(b['short_window_progress']['limits'])
    x.pop('severe_fraction');y.pop('severe_fraction');assert x==y
    assert a['pause_budget']==b['pause_budget'] and a['limits']==b['limits']
    assert before['cycle']['limits']==after['cycle']['limits']
    if family=='pulse':
        assert new_ms<old_ms-100,(old_ms,new_ms)
        assert new_ms<(650 if phase=='DEPLOY' else 1000),(phase,new_ms)
        assert b['short_window_progress']['trigger_mode']=='SEVERE_THEN_LOW_RECOVERY'
        assert b['short_window_progress']['severe_events'][-1]['decision']=='STOP'
    summary={'phase':phase,'family':family,'noise_m':noise,'false_motor_feedback':fake,'metric_basis':basis,
        'old_success':before['result']['success'],'new_success':after['result']['success'],
        'old_elapsed_ms':old_ms,'new_elapsed_ms':new_ms,'saved_ms':old_ms-new_ms,
        'old_reason':before['result']['message'],'new_reason':after['result']['message'],
        'new_trigger_mode':b['short_window_progress']['trigger_mode'],'other_guard_configuration_equal':True}
    base.DETAILS[name]=summary
    (base.TRACE_ROOT/(name+'.json')).write_text(json.dumps(summary,indent=2)+'\n')


def source_replays(base):
    rows=[]
    modes=[('deploy_regular',0.08,0.2,0.06,6),('deploy_dense',0.16,0.2,0.01,8),
        ('bend_regular',0.02,0.04,0.06,6),('bend_dense',0.03,0.04,0.01,8)]
    for mode,rate,expected,gap,count in modes:
        for onset in (0.17,0.205,0.24):
            for noise in (0.0,0.00002):
                for seed in (7,19,53):
                    for fraction in (0.0,0.6):
                        cfg=PauseBudgetConfig().window(WindowProgressConfig(window_s=0.4,severe_fraction=fraction))
                        g=GoalProgressWindow(cfg);goal=CameraGoal('sim','tip','epoch',(0.3,0.0,0.0))
                        g.begin(goal,0.0005,0.0,expected);rng=random.Random(seed);x=0.0;failure=None
                        for i in range(161):
                            t=i*0.01;previous=max(0.0,t-0.01)
                            overlap=sum(max(0,min(t,onset+j*(0.08+gap)+0.08)-max(previous,onset+j*(0.08+gap))) for j in range(count))
                            if i: x+=rate*max(0,t-previous-overlap)
                            p=(x+rng.uniform(-noise,noise),rng.uniform(-noise,noise),rng.uniform(-noise,noise))
                            try:g.observe(CameraPose(i+1,t,'sim','tip','epoch',True,p))
                            except ValueError as exc:failure={'s':t,'reason':str(exc)};break
                        row={'mode':mode,'onset_s':onset,'noise_m':noise,'seed':seed,'severe_fraction':fraction,
                            'failure':failure,'window':g.status()};rows.append(row)
                        assert failure is None,row
    detail={'scope':'separate source-time replay with a distant goal; not physical full cycles','rows':rows}
    base.DETAILS['early_source_replays']=detail
    (base.TRACE_ROOT/'early_source_replays.json').write_text(json.dumps(detail,indent=2)+'\n')


def source_safeguards(base):
    cfg=PauseBudgetConfig().window(WindowProgressConfig(window_s=0.4,severe_fraction=0.6))
    goal=CameraGoal('sim','tip','epoch',(0.3,0.0,0.0))
    def pose(i,x=0.0):return CameraPose(i+1,i*0.01,'sim','tip','epoch',True,(x,0.0,0.0))
    g=GoalProgressWindow(cfg);g.begin(goal,0.0005,0.0,0.2)
    for i in range(44):g.observe(pose(i))
    assert g.severe_pending is not None
    for _ in range(1000):g.observe(pose(43))
    assert not g.severe_events and g.fault is None
    for i in range(44,85):
        try:g.observe(pose(i))
        except ValueError:break
    assert g.trigger_mode=='SEVERE_THEN_LOW_RECOVERY'
    recovered=GoalProgressWindow(cfg);recovered.begin(goal,0.0005,0.0,0.2)
    for i in range(90):recovered.observe(pose(i,max(0,i*0.01-0.45)*0.3))
    assert not recovered.fault and any(e['decision']=='RECOVERED' for e in recovered.severe_events)
    tiny=GoalProgressWindow(cfg);tiny.begin(goal,0.0005,0.0,0.000001)
    for i in range(110):tiny.observe(pose(i))
    assert not tiny.fault and not tiny.severe_events
    detail={'cached_poll_count':1000,'stalled':g.status(),'recovered':recovered.status(),'unresolved':tiny.status()}
    base.DETAILS['early_source_safeguards']=detail
    (base.TRACE_ROOT/'early_source_safeguards.json').write_text(json.dumps(detail,indent=2)+'\n')


def cases(base):
    result=[]
    def add(name,**kw):result.append((name,lambda n=name,k=kw:paired(base,n,**k)))
    for phase,label in (('DEPLOY','deploy'),('BEND_1','bend'),('RETRACT','retract')):
        add('early_'+label+'_dense',phase=phase,family='dense')
        add('early_'+label+'_dense_noise',phase=phase,family='dense',noise=0.00002)
        for family in ('jam','ramp_creep'):
            for fake in (False,True):add('early_'+label+'_'+family+('_false' if fake else '_actual'),phase=phase,family=family,fake=fake)
        add('early_'+label+'_ramp_zero',phase=phase,family='ramp_zero')
        add('early_'+label+'_action_limit',phase=phase,family='action')
    add('early_cycle_limit',phase='DEPLOY',family='cycle')
    for phase,label in (('DEPLOY','deploy'),('BEND_1','bend')):
        add('early_pulse_'+label,phase=phase,family='pulse')
        add('early_pulse_'+label+'_noise',phase=phase,family='pulse',noise=0.00002)
    result.append(('early_source_replays',lambda:source_replays(base)))
    result.append(('early_source_safeguards',lambda:source_safeguards(base)))
    return result
