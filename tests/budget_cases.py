"""Pause-budget regressions; clocks and observations come from simulated sources."""
import json
from dataclasses import replace
import confirm_pause_cases,early_progress_cases,window_cases
from wide400_cases import metrics
from atarm_controller.progress_window import WindowProgressConfig,PauseBudgetConfig,GoalProgressWindow
from atarm_controller.camera_feedback import CameraGoal,CameraPose

def save(base,name,d):
    base.DETAILS[name]=d;(base.TRACE_ROOT/(name+'.json')).write_text(json.dumps(d,indent=2)+'\n')

def recovery_source(base):
    confirm_pause_cases.source_cases(base)
    d=base.DETAILS['confirm_source_matrix'];save(base,'budget_recovery_source',d)
    assert len(d['rows'])==64 and not any(r['failure'] for r in d['rows'])

def boundary_source(base):
    cfg=PauseBudgetConfig().window(WindowProgressConfig(window_s=.4,severe_fraction=.6))
    g=GoalProgressWindow(cfg);goal=CameraGoal('sim','tip','epoch',(.006,0.,0.));g.begin(goal,.001,0.,.04)
    failure=None;first=None;pending_at_first=None
    for i in range(121):
        t=i*.01
        try:g.observe(CameraPose(i+1,t,'sim','tip','epoch',True,(.00184*t,0.,0.)))
        except ValueError as exc:failure={'s':t,'reason':str(exc)};break
        if first is None and g.events and g.events[-1]['bad']:
            first=dict(g.events[-1]);pending_at_first=g.severe_pending is not None
    d={'first_bad':first,'pending_at_first_bad':pending_at_first,'failure':failure,'window':g.status()}
    save(base,'budget_boundary_source',d)
    assert .6<first['progress_upper_bound_m']/first['required_reduction_m']<.75
    assert pending_at_first and failure and failure['s']<.67
    assert g.trigger_mode=='SEVERE_THEN_LOW_RECOVERY'

def safeguards(base):
    early_progress_cases.source_safeguards(base)
    cfg=PauseBudgetConfig().window(WindowProgressConfig(window_s=.4,severe_fraction=.6))
    goal=CameraGoal('sim','tip','epoch',(.3,0.,0.));g=GoalProgressWindow(cfg);g.begin(goal,.0005,0.,.04)
    for i in range(44):g.observe(CameraPose(i+1,i*.01,'sim','tip','epoch',True,(0.,0.,0.)))
    armed=g.severe_pending['armed_capture_s']
    for _ in range(1000):g.observe(CameraPose(44,.43,'sim','tip','epoch',True,(0.,0.,0.)))
    assert g.severe_pending['armed_capture_s']==armed
    failure=None
    for i in range(44,100):
        try:g.observe(CameraPose(i+1,i*.01,'sim','tip','epoch',True,((i-43)*.000001,0.,0.)))
        except ValueError as exc:failure={'s':i*.01,'reason':str(exc)};break
        assert g.severe_pending['armed_capture_s']==armed
    rejected=False
    try:replace(cfg,severe_confirm_s=cfg.allowed_pause_s)
    except ValueError:rejected=True
    d={'cached_polls':1000,'original_armed_s':armed,'failure':failure,'window':g.status(),
       'zero_effective_confirm_time_rejected':rejected,'existing_safeguards':base.DETAILS['early_source_safeguards']}
    save(base,'budget_safeguards',d)
    assert failure and failure['s']<=.66 and rejected

def recovery(base,name,**kw):
    confirm_pause_cases.pair(base,name,**kw)
    assert base.DETAILS[name]['confirmation_success'],base.DETAILS[name]

def creep(base,name,phase,noise=0.,seed=19):
    runs=[]
    for fraction,suffix in ((0.,'_baseline'),(.6,'_optimized')):
        d=window_cases.run(base,name+suffix,phase=phase,period=.08,step=.00065 if phase=='DEPLOY' else .00025,
            coordinated=True,window_s=.4,require_window_fault=False,severe_fraction=fraction,noise=noise,seed=seed)
        ctrl,ms,basis=metrics(d,'pulse');runs.append((d,ctrl,ms))
    before,a,old_ms=runs[0];after,b,new_ms=runs[1]
    x=dict(a['short_window_progress']['limits']);y=dict(b['short_window_progress']['limits'])
    x.pop('severe_fraction');y.pop('severe_fraction');assert x==y
    assert a['pause_budget']==b['pause_budget'] and a['limits']==b['limits'] and before['cycle']['limits']==after['cycle']['limits']
    d={'family':'pulse','phase':phase,'noise_m':noise,'seed':seed,'other_guard_configuration_equal':True,
        'old_success':before['result']['success'],'new_success':after['result']['success'],
        'old_elapsed_ms':old_ms,'new_elapsed_ms':new_ms,'saved_ms':old_ms-new_ms,
        'old_reason':before['result']['message'],'new_reason':after['result']['message'],
        'new_trigger_mode':b['short_window_progress']['trigger_mode'],
        'rapid_target_met':new_ms<(650 if phase=='DEPLOY' else 1000),'saved_100_ms_target_met':new_ms<old_ms-100}
    save(base,name,d)
    assert d['rapid_target_met'] and d['saved_100_ms_target_met'],d
    assert d['new_trigger_mode']=='SEVERE_THEN_LOW_RECOVERY'

def cases(base):
    result=[]
    for phase,label in [('DEPLOY','deploy'),('BEND_1','bend'),('RETRACT','retract')]:
        for rate_kind in ('fast','slow'):
            for delay,tail in [(None,'no_pause'),(.02,'start'),(.06,'middle'),(.1,'late')]:
                name='budget_'+label+'_'+rate_kind+'_'+tail
                result.append((name,lambda n=name,p=phase,r=rate_kind,d=delay:recovery(base,n,phase=p,delay_s=d,rate_kind=r)))
        name='budget_'+label+'_slow_noise'
        result.append((name,lambda n=name,p=phase:recovery(base,n,phase=p,delay_s=.06,rate_kind='slow',noise=.00002)))
    for phase,label in [('DEPLOY','deploy'),('BEND_1','bend')]:
        for noise,tail in [(0.,''),(.00002,'_noise')]:
            name='budget_creep_'+label+tail
            result.append((name,lambda n=name,p=phase,v=noise:creep(base,n,p,v)))
    for seed in (7,41,53):
        for repeat in range(2):
            name='budget_bend_noise_'+str(seed)+'_repeat'+str(repeat)
            result.append((name,lambda n=name,s=seed:creep(base,n,'BEND_1',.00002,s)))
    result += [('budget_recovery_source',lambda:recovery_source(base)),('budget_boundary_source',lambda:boundary_source(base)),
               ('budget_safeguards',lambda:safeguards(base))]
    return result
