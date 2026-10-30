"""Real component sequence through PTYs; stage faults use independent physical state."""
from dataclasses import asdict,replace
import math,time,types
from atarm_controller.camera_cycle import CyclePlan,CycleConfig
from atarm_controller.camera_feedback import CameraGoal,CameraGateConfig
from atarm_controller.online_jacobian import JacobianUpdateConfig
from servo_cases import config as deploy_config
from bend_cases import BendCamera,bend_config,camera_goals
from atarm_controller.coupled_bend import length,sub


class CycleCamera(BendCamera):
    def __init__(self,rig,fault_phase=None,fault=None,plant='coupled',noise=0.0,gain=1.0):
        super().__init__(rig,plant=plant,noise=noise,seed=19)
        self.fault_phase=fault_phase;self.fault=fault;self.deploy_gain=gain
        self.phase_counts={};self.seen_frame=None;self.injected=False;self.noise=noise
        self.plan=None;self.ownership_checked=False
    def truth(self):
        p=super().truth();return (p[0],p[1],self.deploy_gain*p[2])
    def __call__(self):
        sample=super().__call__()
        cycle=getattr(self.rig.arm,'cycle',None);phase=getattr(cycle,'phase','LEGACY')
        if sample.frame_id!=self.seen_frame:
            self.seen_frame=sample.frame_id;self.phase_counts[phase]=self.phase_counts.get(phase,0)+1
        if phase==self.fault_phase and self.phase_counts.get(phase,0)>=2:
            self.injected=True
            if self.fault=='lost': sample=replace(sample,tracking_ok=False)
            elif self.fault=='stale': sample=replace(sample,capture_s=sample.capture_s-1.0)
            elif self.fault=='epoch': sample=replace(sample,map_epoch='reset-map')
            elif self.fault=='false_encoder': self.rig.deploy.mode='coherent_false'
            elif self.fault=='motor_crc': self.rig.active.motor_modes[0]='bad_crc'
            elif self.fault=='deploy_timeout': self.rig.deploy.mode='timeout'
            elif self.fault=='cancel': self.rig.arm.cancel_camera_cycle()
            elif self.fault=='ownership' and not self.ownership_checked:
                arm=self.rig.arm;mark=(len(self.rig.active.logs),len(self.rig.deploy.logs))
                assert not arm.run_camera_cycle(self.plan)['success']
                assert not arm.set_deploy(False)['success']
                assert not arm.move_bend_camera(self.plan.bend_goals[0])['success']
                assert not arm.confirm_retracted_reference(mechanically_confirmed=True)['success']
                assert not arm.step()
                assert mark==(len(self.rig.active.logs),len(self.rig.deploy.logs))
                self.ownership_checked=True
        self.last=sample
        return sample


def sequence_config(**changes):
    values=dict(timeout_s=6.0,verification_timeout_s=0.4,verification_period_s=0.002,
        position_tolerance_m=0.001,noise_bound_m=0.0001,max_pose_age_s=0.07,
        settle_s=0.025,neutral_tolerance_m=0.0003)
    values.update(changes);return CycleConfig(**values)


def exercise(base,name,fault_phase=None,fault=None,expected=True,expected_reason=None,
             noise=0.0,gain=1.0,plant='coupled',multiple=False,repeated=False,changes=None,
             forged=None,no_home=False,invalid=None,use_node=False,expect_rejected=False,second_fault=False):
    rig=base.Rig()
    try:
        camera=CycleCamera(rig,fault_phase,fault,plant,noise,gain)
        bc=bend_config(max(noise*math.sqrt(3),0.00005))
        dc=deploy_config(
                jacobian_column=(0.0,0.0,1.0),deadband_m=0.00004,resume_error_m=0.00008,
                min_step_m=0.00001,min_improvement_m=0.00003,max_duration_s=1.4,max_adjustments=24)
        dc=replace(dc,online_update=JacobianUpdateConfig(position_noise_bound_m=max(noise*math.sqrt(3),0.00005),
            min_norm=0.2,max_norm=3.0,min_actuation_m=0.001))
        deploy_cc=CameraGateConfig(max_age_s=0.07,position_tolerance_m=0.0005,settle_s=0.025,
            progress_min_m=0.0001,progress_timeout_s=0.5,max_speed_m_s=4.0,step_margin_m=0.004)
        bend_cc=replace(deploy_cc,position_tolerance_m=0.001,progress_timeout_s=0.7)
        cfg=sequence_config(**(changes or {}))
        options={'camera_required':True,'camera_feedback':camera,'camera_goals':camera_goals(),
            'camera_config':deploy_cc,'visual_servo_config':dc,'bend_servo_config':bc}
        lib=__import__('atarm_controller.arm_controller_lib',fromlist=['*'])
        if hasattr(lib,'CameraArmCycle'):
            options.update({'cycle_config':cfg,'bend_camera_config':bend_cc})
        rig.construct(control_options=options,deploy_timeout_s=1.6)
        if not no_home: rig.home()
        neutral=camera_goals()[True]
        goals=(replace(neutral,position_m=(0.008,-0.006,0.05)),)
        if multiple: goals+= (replace(neutral,position_m=(-0.004,0.006,0.05)),)
        plan=CyclePlan(neutral,goals)
        if invalid=='frame': plan=CyclePlan(neutral,(replace(goals[0],map_epoch='other-map'),))
        if invalid=='empty': plan=CyclePlan(neutral,())
        if invalid=='neutral': plan=CyclePlan(replace(neutral,position_m=(0.001,0.0,0.05)),goals)
        camera.plan=plan
        if forged:
            if forged in ('deploy','incomplete','exception','interrupt'):
                original=rig.arm.set_deploy
                def changed(deploy,**kwargs):
                    if deploy:
                        if forged=='exception': raise RuntimeError('INJECTED_STAGE_EXCEPTION')
                        if forged=='interrupt': raise KeyboardInterrupt()
                        if forged=='deploy': return {'success':True,'complete':True,'message':'Injected false success'}
                        result=original(deploy,**kwargs);result['complete']=False;return result
                    return original(deploy,**kwargs)
                rig.arm.set_deploy=changed
            elif forged=='bend':
                rig.arm.move_bend_camera=lambda goal,**kwargs:{'success':True,'complete':True,'message':'Injected false success'}
            elif forged=='retract':
                original=rig.arm.set_deploy
                def changed(deploy,**kwargs):
                    if not deploy:
                        rig.arm.deploy_state='RETRACTED';rig.arm.deployed=False
                        return {'success':True,'complete':True,'message':'Injected false success'}
                    return original(deploy,**kwargs)
                rig.arm.set_deploy=changed
        mark=(len(rig.active.logs),len(rig.deploy.logs));started=time.monotonic()
        if hasattr(rig.arm,'run_camera_cycle'):
            if use_node:
                module=__import__('atarm_controller.arm_controller_node',fromlist=['*'])
                node=object.__new__(module.UnitreeTensegrityControllerNode);node.arm=rig.arm;node.cycle_plan=plan
                response_obj=types.SimpleNamespace(success=False,message='')
                node._cycle_service(None,response_obj);response=rig.arm.cycle_status()['result']
                assert response_obj.success==response['success']
            else:
                try: response=rig.arm.run_camera_cycle(plan)
                except KeyboardInterrupt:
                    assert forged=='interrupt'
                    response=rig.arm.cycle_status()['result']
            status=rig.arm.cycle_status()
        else:
            # Pre-integration contrast: straight calls omit camera-confirmed neutralization.
            response=rig.arm.set_deploy(True)
            if response['success']: response=rig.arm.move_bend_camera(goals[0])
            if response['success']: response=rig.arm.set_deploy(False)
            status={'enabled':False}
        elapsed=time.monotonic()-started
        detail={'response':response,'elapsed_s':elapsed,'workflow':status,'state':rig.arm.deploy_state,
            'physical_q':camera.physical_q(),'physical_deploy_mm':rig.physical_mm(),'true_p':camera.truth(),
            'final_true_error_m':length(camera.truth()),'fault_phase':fault_phase,'fault':fault,
            'injected':camera.injected,'noise_per_axis_m':noise,'deploy_gain':gain,'camera_samples':camera.logs}
        base.save_rig(name,rig,detail)
        assert response['success']==expected,detail
        if expected:
            assert response['complete'] is True and status['state']=='SUCCEEDED',detail
            assert rig.arm.deploy_state=='RETRACTED' and rig.arm.deployed is False,detail
            assert detail['final_true_error_m']<=cfg.position_tolerance_m+0.00001,detail
            phases=[s['phase'] for s in status['stages']]
            wanted=['VERIFY_START','DEPLOY']+['BEND_'+str(i) for i in range(1,len(goals)+1)]+['NEUTRALIZE','RETRACT','VERIFY_END']
            assert phases==wanted and all(s['verified'] for s in status['stages']),detail
            frames=[s['frame_id'] for s in status['stages']]
            assert all(b>a for a,b in zip(frames,frames[1:])),detail
            if repeated:
                second_reply=rig.arm.run_camera_cycle(plan)
                detail['second_cycle']=rig.arm.cycle_status()
                base.save_rig(name,rig,detail)
                assert second_reply['success'],second_reply
                assert detail['second_cycle']['run_id']>status['run_id']
            if fault=='ownership': assert camera.ownership_checked
            if second_fault:
                camera.fault_phase='BEND_1';camera.fault='lost';camera.phase_counts={}
                later=rig.arm.run_camera_cycle(plan);detail['second_cycle']=rig.arm.cycle_status()
                assert not later['success'] and not later['complete']
                assert detail['second_cycle']['run_id']>status['run_id']
                assert not any(s['phase']=='VERIFY_END' for s in detail['second_cycle']['stages'])
        else:
            assert response.get('complete') is not True,detail
            assert expected_reason and expected_reason in response['message'],detail
            if no_home or invalid or expect_rejected:
                assert status['state']=='REJECTED',detail
                assert mark==(len(rig.active.logs),len(rig.deploy.logs)),detail
            else:
                assert status['state']=='FAULT' and rig.arm.deploy_state=='FAULT',detail
                assert status['stop_requested'] and len(status['stop_receipts'])==4,detail
                assert elapsed<cfg.timeout_s+0.5,detail
                failed=[s for s in status['stages'] if not s['verified']]
                assert failed and failed[-1]['phase']==status['phase'],detail
                assert not any(s['phase']=='VERIFY_END' and s['verified'] for s in status['stages']),detail
                tail=(len(rig.active.logs),len(rig.deploy.logs))
                assert not rig.arm.run_camera_cycle(plan)['success'];rig.arm.step()
                assert tail==(len(rig.active.logs),len(rig.deploy.logs)),detail
                detail['no_commands_after_failure']=True
                if fault_phase in ('DEPLOY','BEND_1','NEUTRALIZE') or forged in ('deploy','bend','incomplete','exception','interrupt'):
                    assert not any(s['phase']=='RETRACT' for s in status['stages']),detail
        base.save_rig(name,rig,detail)
    finally: rig.close()


def cases(base):
    result=[]
    def add(name,**kwargs): result.append((name,lambda n=name,k=kwargs:exercise(base,n,**k)))
    add('cycle_normal',repeated=True)
    add('cycle_waypoints',multiple=True)
    add('cycle_nonlinear',plant='nonlinear')
    add('cycle_deploy_bias',gain=0.85)
    add('cycle_noise',noise=0.00002)
    add('cycle_node_service',use_node=True)
    add('cycle_motion_ownership',fault_phase='BEND_1',fault='ownership')
    add('cycle_second_run_fault',second_fault=True)
    for phase in ('DEPLOY','BEND_1','NEUTRALIZE','RETRACT','VERIFY_END'):
        add('cycle_lost_'+phase.lower(),fault_phase=phase,fault='lost',expected=False,expected_reason='CAMERA_TRACKING_LOST')
    add('cycle_deploy_false_encoder',fault_phase='DEPLOY',fault='false_encoder',expected=False,expected_reason='VISUAL_SERVO_NO_PROGRESS')
    add('cycle_bend_crc',fault_phase='BEND_1',fault='motor_crc',expected=False,expected_reason='BAD_CRC')
    add('cycle_retract_timeout',fault_phase='RETRACT',fault='deploy_timeout',expected=False,expected_reason='TIMEOUT')
    add('cycle_final_stale',fault_phase='VERIFY_END',fault='stale',expected=False,expected_reason='CAMERA_FEEDBACK_STALE')
    add('cycle_cancel_bend',fault_phase='BEND_1',fault='cancel',expected=False,expected_reason='CYCLE_CANCELLED')
    add('cycle_cancel_final_verify',fault_phase='VERIFY_END',fault='cancel',expected=False,expected_reason='CYCLE_CANCELLED')
    add('cycle_overall_timeout',changes={'timeout_s':0.15},expected=False,expected_reason='CYCLE_TIME_LIMIT')
    for kind in ('deploy','bend','retract'):
        add('cycle_false_'+kind,forged=kind,expected=False,expected_reason='CYCLE_STAGE_')
    add('cycle_incomplete_reply',forged='incomplete',expected=False,expected_reason='CYCLE_STAGE_FAILED')
    add('cycle_stage_exception',forged='exception',expected=False,expected_reason='INJECTED_STAGE_EXCEPTION')
    add('cycle_interrupt',forged='interrupt',expected=False,expected_reason='CYCLE_INTERRUPTED')
    add('cycle_precision_rejected',noise=0.0002,expected=False,expect_rejected=True,expected_reason='CYCLE_HANDOFF_PRECISION_INCONSISTENT')
    add('cycle_unknown_start',no_home=True,expected=False,expected_reason='CYCLE_REQUIRES_CONFIRMED_RETRACTED_START')
    for invalid in ('frame','empty','neutral'):
        add('cycle_invalid_'+invalid,invalid=invalid,expected=False,expected_reason='CYCLE_')
    return result


def repeated_endpoint_hold(base):
    from camera_cases import PhysicalCamera,goals
    rig=base.Rig()
    try:
        raw=PhysicalCamera(lambda:rig.physical_mm()/1000);bias=[0.0]
        def camera():
            sample=raw()
            return replace(sample,position_m=(sample.position_m[0]+bias[0],0.0,0.0))
        cc=CameraGateConfig(max_age_s=0.06,position_tolerance_m=0.001,settle_s=0.025,
            progress_timeout_s=0.5,max_speed_m_s=4.0,step_margin_m=0.004)
        rig.construct(control_options={'camera_required':True,'camera_feedback':camera,'camera_goals':goals(),
            'camera_config':cc,'visual_servo_config':deploy_config()},deploy_timeout_s=0.6)
        rig.home();assert rig.call(True)['success'];before=rig.physical_mm()
        bias[0]=0.0009;raw.last=None
        reply=rig.call(True)
        assert reply['success'] and rig.arm.visual_servo_status()['adjustments']==0
        assert abs(rig.physical_mm()-before)<0.01
        base.save_rig('cycle_repeated_scalar_hold',rig,{'response':reply,'before_mm':before,
            'after_mm':rig.physical_mm(),'visual_servo':rig.arm.visual_servo_status()})
    finally: rig.close()
