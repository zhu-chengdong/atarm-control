"""Changing physical plant, guarded estimator checks and paired fixed-model runs."""
from dataclasses import replace
import math
import time
from atarm_controller.online_jacobian import OnlineJacobian,JacobianUpdateConfig
from atarm_controller.camera_feedback import CameraGateConfig
from atarm_controller.visual_servo import VisualServoConfig
from servo_cases import PlantCamera,config
from camera_cases import goals


def update_config(noise=0.0001):
    return JacobianUpdateConfig(position_noise_bound_m=noise,min_norm=0.2,max_norm=3.0,
        learning_rate=0.5,min_actuation_m=0.001,max_fractional_change=0.25)


class VaryingCamera(PlantCamera):
    def __init__(self,rig,plant,noise,seed,outliers=False,period=0.01):
        super().__init__(rig,noise=noise,seed=seed,mode='outliers' if outliers else 'normal',period=period)
        self.plant=plant

    def position(self,q):
        if self.plant=='smooth': return 0.55*q+4*q*q
        if self.plant=='increase': return 0.65*q if q<=0.054 else 0.65*0.054+1.2*(q-0.054)
        if self.plant=='decrease': return 0.85*q if q<=0.054 else 0.85*0.054+0.45*(q-0.054)
        if self.plant=='reversal': return 0.85*q if q<=0.054 else 0.85*0.054-0.6*(q-0.054)
        if self.plant=='jammed': return 0.0
        return 0.8*q

    def truth(self):
        return self.position(self.rig.physical_mm()/1000)


def run_plant(base,name,plant='smooth',noise=0.0,seed=1,adaptive=True,outliers=False,
              expected=True,period=0.01,repeated=0,motor_mode='normal'):
    rig=base.Rig()
    try:
        camera=VaryingCamera(rig,plant,noise,seed,outliers,period)
        cfg=config(jacobian_column=(1.6,0.0,0.0),max_step_m=0.003,
            max_length_m=0.11,max_offset_m=0.055,max_total_correction_m=0.08,
            max_adjustments=26,max_duration_s=2.4,no_progress_steps=4)
        if adaptive and 'online_update' in VisualServoConfig.__dataclass_fields__:
            cfg=replace(cfg,online_update=update_config(max(noise,0.0001)))
        camcfg=CameraGateConfig(max_age_s=max(0.08,2*period),position_tolerance_m=0.001,
            settle_s=0.025,progress_min_m=0.0003,progress_timeout_s=0.7,
            max_speed_m_s=4.0,step_margin_m=0.004)
        rig.construct(control_options={'camera_required':True,'camera_feedback':camera,
            'camera_goals':goals(),'camera_config':camcfg,'visual_servo_config':cfg},deploy_timeout_s=2.6)
        rig.home(); rig.deploy.mode=motor_mode
        began=time.monotonic(); response=rig.call(True); elapsed=time.monotonic()-began
        servo=rig.arm.visual_servo_status(); online=servo.get('online_jacobian',{'enabled':False})
        q=rig.physical_mm()/1000
        secants=[]
        for event in online.get('events',[]):
            b=event['before']['actuation_m']; a=event['after']['actuation_m']
            secants.append({'accepted':event['accepted'],'reason':event['reason'],
                'actual_secant':None if abs(a-b)<1e-12 else (camera.position(a)-camera.position(b))/(a-b),
                'model_after':event['after_column'][0]})
        detail={'plant':plant,'noise_bound_m':noise,'seed':seed,'response':response,
            'elapsed_s':elapsed,'state':rig.arm.deploy_state,'final_motor_m':q,
            'true_error_m':abs(camera.truth()-0.05),'camera_samples':camera.logs,
            'visual_servo':servo,'online_jacobian':online,'independent_true_secants':secants,
            'camera_evidence':rig.arm.camera_status()}
        base.save_rig(name,rig,detail)
        assert response['success']==expected,detail
        if adaptive:
            assert online['enabled'],detail
            assert all(0.2<=math.sqrt(sum(v*v for v in e['after_column']))<=3.0 for e in online['events']),detail
            for event in online['events']:
                if event['accepted']:
                    change=math.sqrt(sum((a-b)**2 for a,b in zip(event['after_column'],event['before_column'])))
                    assert change<=0.25*math.sqrt(sum(v*v for v in event['before_column']))+1e-12,detail
                else:
                    assert event['after_column']==event['before_column'],detail
        if expected:
            assert detail['true_error_m']<=0.001+noise,detail
            if adaptive:
                assert online['accepted_updates']>=1,detail
                latest=[s for s in secants if s['accepted']][-1]
                initial_error=abs(1.6-latest['actual_secant'])
                learned_error=abs(latest['model_after']-latest['actual_secant'])
                assert learned_error<initial_error,detail
                detail['last_secant_initial_error']=initial_error
                detail['last_secant_learned_error']=learned_error
            signs=[math.copysign(1,s['delta_m']) for s in servo['history']]
            detail['correction_reversals']=sum(a!=b for a,b in zip(signs,signs[1:]))
            assert detail['correction_reversals']<=1,detail
            if noise: assert detail['correction_reversals']==0,detail
            model_before=online.get('column'); before_q=q
            repeat_models=[]
            for _ in range(repeated):
                assert rig.call(True)['success']
                repeat_models.append(rig.arm.visual_servo_status()['online_jacobian']['column'])
                assert rig.arm.visual_servo_status()['adjustments']==0
                assert abs(rig.physical_mm()/1000-before_q)<0.00005
            if repeated:
                assert all(j==model_before for j in repeat_models),detail
                detail['model_unchanged_without_new_motion']=True
        else:
            assert rig.arm.deploy_state=='FAULT' and servo['stop_requested'],detail
            assert elapsed<2.9,detail
            mark=(len(rig.active.logs),len(rig.deploy.logs))
            assert not rig.call(True)['success']; rig.arm.step()
            assert mark==(len(rig.active.logs),len(rig.deploy.logs)),detail
            detail['no_commands_after_fault']=True
            if adaptive and plant=='jammed':
                assert online['accepted_updates']==0 and online['column']==online['initial_column'],detail
        base.save_rig(name,rig,detail)
        return detail
    finally:
        rig.close()


def paired(base,name,plant):
    fixed=run_plant(base,name+'_fixed',plant=plant,adaptive=False)
    learned=run_plant(base,name+'_online',plant=plant,adaptive=True)
    base.DETAILS[name]={'plant':plant,'fixed_steps':fixed['visual_servo']['adjustments'],
        'online_steps':learned['visual_servo']['adjustments'],
        'fixed_error_m':fixed['true_error_m'],'online_error_m':learned['true_error_m']}


def window(q,p,first=1):
    return {'actuation_m':q,'position_m':(p,0.0,0.0),'first_frame':first,'last_frame':first+2,
        'capture_start_s':first*0.01,'capture_end_s':(first+2)*0.01,
        'motor_spread_m':0.0,'other_spread_m':0.0,'pose_mad_m':0.0,'other_lengths_m':(0.0,0.0,0.0),
        'reference':('base','tip','map-1')}


def guard_case(base,name,kind):
    est=OnlineJacobian((1.0,0.0,0.0),update_config(0.0001 if kind=='accepted' else 0.0004))
    before=window(0.0,0.0); after=window(0.003,0.0024,6); target=0.003
    expected=None
    if kind=='small': after=window(0.0001,0.0001,6); target=0.0001; expected='INSUFFICIENT_EXCITATION'
    if kind=='noise': after=window(0.003,0.0003,6); expected='NOISE_DOMINATED_TRANSLATION'
    if kind=='huge': after=window(0.003,0.02,6); expected='OBSERVED_GAIN_OUT_OF_BOUNDS'
    if kind=='direction': after=window(0.003,-0.0024,6); expected='OBSERVED_DIRECTION_INCONSISTENT'
    if kind=='unstable': after['motor_spread_m']=0.001; expected='MOTOR_NOT_SETTLED'
    if kind=='other': after['other_lengths_m']=(0.001,0.0,0.0); expected='OTHER_ACTUATOR_MOVED'
    if kind=='other_window': after['other_spread_m']=0.001; expected='OTHER_ACTUATOR_NOT_SETTLED'
    if kind=='replay': after=window(0.003,0.0024,2); expected='UNPAIRED_SOURCE_FRAMES'
    if kind=='old': after['capture_end_s']=2.0; expected='PAIR_TOO_OLD'
    if kind=='untracked': target=0.005; expected='TARGET_NOT_TRACKED'
    if kind=='mad': after['pose_mad_m']=0.002; expected='NOISY_POSE_WINDOW'
    if kind=='consistent_noise': after=window(0.003,0.0031,6); expected='MODEL_CONSISTENT_WITH_NOISE'
    if kind=='no_pending':
        assert not est.observe(after) and est.status()['accepted_updates']==0
    else:
        est.remember(before,target)
        accepted=est.observe(after)
        if expected:
            assert not accepted and est.events[-1]['reason']==expected
            assert est.column==(1.0,0.0,0.0)
        else:
            assert accepted and abs(est.column[0]-0.9004424778761062)<1e-9
            saved=est.column; count=len(est.events)
            assert not est.observe(after) and len(est.events)==count and est.column==saved
            est.new_request(); assert est.column==saved
            est.reset(); assert est.column==(1.0,0.0,0.0)
    base.DETAILS[name]=est.status()


def noise_only_innovation(base):
    est=OnlineJacobian((1.0,0.0,0.0),update_config(0.0004))
    for i in range(100):
        q=0.003*i
        b=window(q,q+(0.0003 if i%2 else -0.0003),10*i+1)
        a=window(q+0.003,q+0.003+(-0.0003 if i%2 else 0.0003),10*i+6)
        est.remember(b,q+0.003)
        assert not est.observe(a)
        assert est.events[-1]['reason']=='MODEL_CONSISTENT_WITH_NOISE'
    assert est.column==(1.0,0.0,0.0) and est.status()['accepted_updates']==0
    base.DETAILS['jacobian_100_noise_pairs']=est.status()


def reference_reset(base):
    est=OnlineJacobian((1.0,0.0,0.0),update_config(0.0001))
    before=window(0.0,0.0); after=window(0.003,0.0024,6)
    est.remember(before,0.003); assert est.observe(after)
    learned=est.column; est.new_request(); assert est.column==learned and not est.pending
    changed=window(0.006,0.005,11); changed['reference']=('base','tip','map-2')
    try:
        est.observe(changed)
    except ValueError as exc:
        assert 'JACOBIAN_REFERENCE_CHANGED' in str(exc)
    else:
        raise AssertionError('Map change must require explicit reference recovery')
    assert est.column==learned
    est.reset(); assert est.column==(1.0,0.0,0.0) and est.reference is None
    base.DETAILS['jacobian_reference_reset']=est.status()


def cases(base):
    result=[]
    def add(name,**kwargs): result.append((name,lambda n=name,k=kwargs:run_plant(base,n,**k)))
    add('adaptive_smooth',repeated=2)
    add('adaptive_increase',plant='increase')
    add('adaptive_decrease',plant='decrease')
    for seed in (7,19,41):
        add('adaptive_noise_'+str(seed),noise=0.0004,seed=seed,repeated=2)
    add('adaptive_outliers',noise=0.0002,outliers=True)
    add('adaptive_30hz',period=1/30)
    add('adaptive_reversal',plant='reversal',expected=False)
    add('adaptive_jammed',plant='jammed',noise=0.0004,expected=False)
    add('adaptive_false_encoder',plant='jammed',noise=0.0004,motor_mode='coherent_false',expected=False)
    for plant in ('smooth','increase','decrease'):
        name='adaptive_paired_'+plant
        result.append((name,lambda n=name,p=plant:paired(base,n,p)))
    for kind in ('accepted','small','noise','huge','direction','unstable','other','other_window','replay','old','untracked','mad','no_pending','consistent_noise'):
        name='jacobian_guard_'+kind
        result.append((name,lambda n=name,k=kind:guard_case(base,n,k)))
    result.append(('jacobian_100_noise_pairs',lambda:noise_only_innovation(base)))
    result.append(('jacobian_reference_reset',lambda:reference_reset(base)))
    return result
