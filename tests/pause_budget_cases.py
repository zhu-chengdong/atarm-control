"""Shared pause-budget acceptance; original and coordinated profiles coexist."""
from dataclasses import replace
import json,math
import transient_cases
import window_cases
from atarm_controller.progress_window import WindowProgressConfig,PauseBudgetConfig


def run(base,name,**kw):
    d=transient_cases.run(base,name,coordinated=True,**kw)
    p=d['pause_budget'];g=d['window']
    assert p['enabled'] and p['pause_s']==g['limits']['allowed_pause_s']==0.08
    assert g['limits']['window_s']==p['window_s']==0.2
    assert p['motor_timeout_s']>=p['pause_s']+p['sampling_slack_s']+p['motor_progress_m']*p['camera_per_motor']/p['minimum_camera_rate_m_s']-1e-12
    if d['expected_success'] is True and d['kind']=='pause':
        assert d['resumed_s']>=d['profile']['scheduled_resume_s']
    return d


def pair(base,name,phase):
    before=transient_cases.run(base,name+'_baseline',phase=phase,pause_s=0.08,expected=None)
    after=run(base,name+'_coordinated',phase=phase,pause_s=0.08)
    if phase!='BEND_1':
        assert not before['result']['success'] and before['resumed_s'] is None
        assert 'MOTION_PROGRESS_STALLED' in before['result']['message']
    assert after['result']['success'] and after['result']['complete']
    comparison={'phase':phase,'baseline_success':before['result']['success'],
        'baseline_reason':before['result']['message'],'baseline_ms':before['onset_to_return_ms'],
        'coordinated_success':after['result']['success'],'coordinated_final_error_m':after['true_error_m'],
        'resumed_after_ms':(after['resumed_s']-after['profile']['injected_s'])*1000,
        'pause_budget':after['pause_budget']}
    base.DETAILS[name]=comparison
    (base.TRACE_ROOT/(name+'.json')).write_text(json.dumps(comparison,indent=2)+'\n')


def planned(base,name,mode):
    d=window_cases.run(base,name,healthy=True,pulse=False,coordinated=True,planned=mode,
        phase='DEPLOY' if mode=='deploy' else 'BEND_1')
    phase='DEPLOY' if mode=='deploy' else 'BEND_1'
    stage=next(s for s in d['cycle']['stages'] if s['phase']==phase)
    ctrl=stage['controller']['visual'] if mode=='deploy' else stage['controller']
    assert ctrl['pause_budget']['enabled']
    assert math.isclose(ctrl['pause_budget']['minimum_camera_rate_m_s'],d['window']['required_rate_m_s'])


def pulses(base,name,phase):
    d=window_cases.run(base,name,coordinated=True,phase=phase,period=0.08,
        step=0.00065 if phase=='DEPLOY' else 0.00025)
    assert d['window']['fault']=='WINDOW_PROGRESS_TOO_SMALL'
    assert len(d['pulses'])>=2


def config_checks(base):
    cfg=WindowProgressConfig();p=PauseBudgetConfig();resolved=p.window(cfg)
    assert cfg.allowed_pause_s==0.04 and cfg.window_s==0.12
    assert resolved.allowed_pause_s==0.08 and resolved.window_s==0.2
    assert math.isclose(p.timeout(0.06,0.0005,0.04),0.1125)
    assert p.timeout(1.0,0.0005,0.04)==1.0
    assert math.isclose(p.timeout(0.06,0.0005,0.0016),0.4125)
    rejected=0
    for values in ({'pause_s':0},{'pause_s':float('nan')},{'sampling_slack_s':-1},{'pause_s':True}):
        try: PauseBudgetConfig(**values)
        except ValueError: rejected+=1
        else: raise AssertionError('Invalid pause budget accepted')
    try: p.window(None)
    except ValueError: rejected+=1
    else: raise AssertionError('Pause budget requires independent short-window evidence')
    assert rejected==5
    detail={'original_window_unchanged':True,'pause_s':resolved.allowed_pause_s,'window_s':resolved.window_s,
        'motor_timeout_normal_s':p.timeout(0.06,0.0005,0.04),'motor_timeout_planned_s':p.timeout(0.06,0.0005,0.0016),
        'invalid_configurations_rejected':rejected,'limits_preserved':'overall operation time, cycle time, iteration, travel and pose tolerances'}
    base.DETAILS['budget_configuration']=detail
    (base.TRACE_ROOT/'budget_configuration.json').write_text(json.dumps(detail,indent=2)+'\n')


def cases(base):
    result=[]
    def add(name,**kw): result.append((name,lambda n=name,k=kw:run(base,n,**k)))
    for phase,label in (('DEPLOY','deploy'),('BEND_1','bend'),('RETRACT','retract')):
        name='budget_'+label+'_pause80_pair'
        result.append((name,lambda n=name,p=phase:pair(base,n,p)))
        add('budget_'+label+'_pause80_noise',phase=phase,pause_s=0.08,noise=0.00002)
        add('budget_'+label+'_decelerate_ok',phase=phase,kind='ramp_ok')
        add('budget_'+label+'_pause300_fault',phase=phase,pause_s=0.3,expected=False)
        add('budget_'+label+'_decelerate_zero',phase=phase,kind='ramp_zero',expected=False)
        add('budget_'+label+'_decelerate_creep',phase=phase,kind='ramp_creep',expected=False)
        add('budget_'+label+'_false_creep',phase=phase,kind='ramp_creep',fake=True,expected=False)
    for mode in ('deploy','bend','small_step'):
        name='budget_planned_'+mode
        result.append((name,lambda n=name,m=mode:planned(base,n,m)))
    for phase,label in (('DEPLOY','deploy'),('BEND_1','bend')):
        name='budget_pulse_'+label
        result.append((name,lambda n=name,p=phase:pulses(base,n,p)))
    result.append(('budget_configuration',lambda:config_checks(base)))
    return result
