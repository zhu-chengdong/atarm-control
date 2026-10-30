"""Matched 200/400 ms configurations across recovery, real faults and deadlines."""
import json,math
import transient_cases,window_cases,repeated_pause_cases


def metrics(d,family):
    if family in ('dense','action','cycle'):
        ctrl=d['controller'];elapsed=d['cycle_elapsed_s'] if family=='cycle' else d['phase_elapsed_s']
        basis='cycle_start' if family=='cycle' else 'phase_start'
        return ctrl,elapsed*1000,basis
    if family=='pulse':
        stage=next(s for s in d['cycle']['stages'] if s['phase']==d['phase'])
        ctrl=stage['controller']['visual'] if d['phase']=='DEPLOY' else stage['controller']
        return ctrl,d['onset_to_failure_ms'],'fault_injection'
    stage=next(s for s in d['cycle']['stages'] if s['phase']==d['phase'])
    ctrl=stage['controller']['visual'] if d['phase']!='BEND_1' else stage['controller']
    return ctrl,d['onset_to_return_ms'],'fault_injection'


def paired(base,name,phase,family,noise=0.0,fake=False):
    results=[]
    for width in (0.2,0.4):
        raw_name=name+('_200' if width==0.2 else '_400')
        if family=='dense':
            d=repeated_pause_cases.run(base,raw_name,phase=phase,repetitions=8,move_s=0.01,
                dense=True,wide=width==0.4,noise=noise)
            if width==0.4: assert d['result']['success'] and d['resumed_count']==8
        elif family in ('action','cycle'):
            d=repeated_pause_cases.run(base,raw_name,phase=phase,repetitions=100,move_s=0.015,
                wide=width==0.4,deadline=family=='action',cycle_deadline=family=='cycle')
        elif family=='pulse':
            d=window_cases.run(base,raw_name,phase=phase,period=0.08,
                step=0.00065 if phase=='DEPLOY' else 0.00025,coordinated=True,
                window_s=width,require_window_fault=False)
            assert not d['result']['success'] and not d['result']['complete']
            assert d['onset_to_failure_ms']<(1450 if phase=='DEPLOY' else 2050)
        else:
            d=transient_cases.run(base,raw_name,phase=phase,kind=family,fake=fake,
                expected=False,coordinated=True,window_s=width)
        ctrl,elapsed,basis=metrics(d,family);g=ctrl['short_window_progress'];p=ctrl['pause_budget']
        assert g['limits']['window_s']==width and p['pause_s']==g['limits']['allowed_pause_s']==0.08
        assert p['window_s']==width
        if family=='jam': assert all(s['max_move_m']==0 for s in d['steps'])
        results.append((d,ctrl,elapsed,basis))
    before,a,old_ms,basis=results[0];after,b,new_ms,new_basis=results[1]
    assert basis==new_basis
    old_window=dict(a['short_window_progress']['limits']);new_window=dict(b['short_window_progress']['limits'])
    old_window.pop('window_s');new_window.pop('window_s');assert old_window==new_window
    old_budget=dict(a['pause_budget']);new_budget=dict(b['pause_budget'])
    old_budget.pop('window_s');new_budget.pop('window_s');assert old_budget==new_budget
    assert a['limits']==b['limits'] and before['cycle']['limits']==after['cycle']['limits']
    summary={'phase':phase,'family':family,'noise_m':noise,'false_motor_feedback':fake,'metric_basis':basis,
        'old_success':before['result']['success'],'new_success':after['result']['success'],
        'old_elapsed_ms':old_ms,'new_elapsed_ms':new_ms,'delta_ms':new_ms-old_ms,
        'old_reason':before['result']['message'],'new_reason':after['result']['message'],
        'old_window_state':a['short_window_progress']['state'],'new_window_state':b['short_window_progress']['state'],
        'other_guard_configuration_equal':True}
    base.DETAILS[name]=summary
    (base.TRACE_ROOT/(name+'.json')).write_text(json.dumps(summary,indent=2)+'\n')


def cases(base):
    result=[]
    def add(name,**kw): result.append((name,lambda n=name,k=kw:paired(base,n,**k)))
    for phase,label in (('DEPLOY','deploy'),('BEND_1','bend'),('RETRACT','retract')):
        add('w400_'+label+'_dense',phase=phase,family='dense')
        add('w400_'+label+'_dense_noise',phase=phase,family='dense',noise=0.00002)
        for family in ('jam','ramp_creep'):
            for fake in (False,True):
                add('w400_'+label+'_'+family+('_false' if fake else '_actual'),phase=phase,family=family,fake=fake)
        add('w400_'+label+'_ramp_zero',phase=phase,family='ramp_zero')
        add('w400_'+label+'_action_limit',phase=phase,family='action')
    add('w400_cycle_limit',phase='DEPLOY',family='cycle')
    for phase,label in (('DEPLOY','deploy'),('BEND_1','bend')):
        add('w400_'+label+'_pulse',phase=phase,family='pulse')
    return result
