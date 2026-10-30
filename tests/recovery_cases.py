"""Fault recovery uses passive motor exchanges and independent camera evidence."""
from copy import deepcopy
from dataclasses import replace,asdict
import importlib,time
from cycle_cases import CycleCamera,sequence_config
from bend_cases import bend_config,camera_goals
from servo_cases import config as deploy_config
from atarm_controller.camera_cycle import CyclePlan
from atarm_controller.camera_feedback import CameraGateConfig
from atarm_controller.online_jacobian import JacobianUpdateConfig


class RecoveryCamera(CycleCamera):
    mode='normal';held=None;checks=0
    def __call__(self):
        if self.mode=='frozen': return self.held
        pose=super().__call__()
        if self.mode=='lost': pose=replace(pose,tracking_ok=False)
        if self.mode=='stale': pose=replace(pose,capture_s=pose.capture_s-1)
        if self.mode=='map': pose=replace(pose,map_epoch='replacement-map')
        if self.mode=='replay': pose=replace(pose,frame_id=1)
        if self.mode=='margin': pose=replace(pose,position_m=(0.00045,0.0,0.0))
        recovery=getattr(self.rig.arm,'recovery',None)
        if recovery and recovery.active:
            self.checks+=1
            if self.mode=='cancel' and self.checks>=2: self.rig.arm.cancel_camera_recovery()
            if self.mode=='busy' and self.checks==2:
                arm=self.rig.arm;mark=counts(self.rig)
                assert not arm.recover_camera_cycle(mechanically_confirmed=True,fault_cleared=True)['success']
                assert not arm.run_camera_cycle(self.plan)['success']
                assert not arm.set_deploy(False)['success']
                assert not arm.move_bend_camera(self.plan.bend_goals[0])['success']
                assert not arm.confirm_retracted_reference(mechanically_confirmed=True)['success']
                assert not arm.step();assert mark==counts(self.rig)
        return pose


def counts(rig): return (len(rig.active.logs),len(rig.deploy.logs))
def passive_since(rig,mark):
    import struct
    records=rig.active.logs[mark[0]:]+rig.deploy.logs[mark[1]:]
    assert records
    for r in records:
        raw=bytes.fromhex(r['request_hex'])
        assert all(v==0 for v in struct.unpack_from('<hh',raw,20)),r
        assert struct.unpack_from('<h',raw,12)[0]==0,r
        assert struct.unpack_from('<h',raw,14)[0]==0,r
    return len(records)


def setup(base,phase='BEND_1',fault='lost',extra_options=None):
    rig=base.Rig();camera=RecoveryCamera(rig,phase,fault)
    cc=CameraGateConfig(max_age_s=0.07,position_tolerance_m=0.0005,settle_s=0.025,
        progress_min_m=0.0001,progress_timeout_s=0.5,max_speed_m_s=4.0,step_margin_m=0.004)
    dc=deploy_config(jacobian_column=(0.0,0.0,1.0),deadband_m=0.00004,resume_error_m=0.00008,
        min_step_m=0.00001,min_improvement_m=0.00003,max_duration_s=1.4,max_adjustments=24)
    dc=replace(dc,online_update=JacobianUpdateConfig(position_noise_bound_m=0.00005,
        min_norm=0.2,max_norm=3.0,min_actuation_m=0.001))
    options={'camera_required':True,'camera_feedback':camera,'camera_goals':camera_goals(),'camera_config':cc,
        'visual_servo_config':dc,'bend_servo_config':bend_config(0.0001),
        'bend_camera_config':replace(cc,position_tolerance_m=0.001,progress_timeout_s=0.7),
        'cycle_config':sequence_config()}
    lib=importlib.import_module('atarm_controller.arm_controller_lib')
    if hasattr(lib,'CameraRecovery'):
        from atarm_controller.camera_recovery import RecoveryConfig
        options['recovery_config']=RecoveryConfig(timeout_s=0.3,period_s=0.002,settle_s=0.025,max_pose_age_s=0.07)
    options.update(extra_options or {})
    rig.construct(control_options=options,deploy_timeout_s=1.6);rig.home()
    neutral=camera_goals()[True];plan=CyclePlan(neutral,(replace(neutral,position_m=(0.008,-0.006,0.05)),))
    camera.plan=plan
    return rig,camera,plan,options


def external_home(rig,camera):
    # Test fixture changes the synthetic world, not the production controller.
    rig.active.position={0:0,1:0,2:0};rig.deploy.position={0:0,1:0,2:0}
    camera.last=None


def exercise(base,name,kind='success',phase='BEND_1',fault='lost'):
    rig,camera,plan,options=setup(base,phase,fault)
    try:
        failed=rig.arm.run_camera_cycle(plan);assert not failed['success']
        prior=deepcopy(rig.arm.cycle_status());assert prior['state']=='FAULT'
        mark=counts(rig)
        assert not rig.arm.run_camera_cycle(plan)['success'];assert mark==counts(rig)
        camera.fault_phase=None;camera.fault=None;camera.last=None
        rig.active.mode='normal';rig.active.motor_modes.clear();rig.deploy.mode='normal'
        if kind!='not_home': external_home(rig,camera)
        kwargs={'mechanically_confirmed':kind!='no_confirmation','fault_cleared':kind!='no_ack'}
        reason=None
        if kind in ('lost','stale','map','replay','margin','cancel','busy'): camera.mode=kind
        if kind=='fail_then_fix': camera.mode='lost'
        if kind=='frozen': camera.held=camera();camera.mode='frozen'
        if kind=='motor_error': rig.active.motor_modes[0]='motor_error'
        if kind=='motor_drift': rig.active.motor_modes[0]='drift_idle'
        if kind=='hidden_jam': rig.deploy.mode='stall'
        prior_traces={'active':deepcopy(rig.active.adapter.trace),'deploy':deepcopy(rig.deploy.adapter.trace)}
        if kind in ('reconnect','persistent_crc'):
            # New explicitly supplied PTY transports; never clear native quarantine in place.
            rig.active.reconnect_adapter();rig.deploy.reconnect_adapter()
            kwargs['replacement_transports']=(rig.active.adapter,rig.deploy.adapter)
            if kind=='persistent_crc': rig.active.motor_modes[0]='bad_crc'
        if kind=='shortcut':
            result=rig.arm.confirm_retracted_reference(mechanically_confirmed=True)
        elif kind=='node':
            import types
            node=object.__new__(importlib.import_module('atarm_controller.arm_controller_node').UnitreeTensegrityControllerNode)
            node.arm=rig.arm;reply=types.SimpleNamespace(success=False,message='')
            node._recover_service(None,reply);result=rig.arm.recovery_status()['result']
            assert reply.success==result['success']
        elif hasattr(rig.arm,'recover_camera_cycle'):
            result=rig.arm.recover_camera_cycle(**kwargs)
        else:
            result=rig.arm.confirm_retracted_reference(mechanically_confirmed=kwargs['mechanically_confirmed'])
        status=rig.arm.recovery_status() if hasattr(rig.arm,'recovery_status') else {'state':'UNIMPLEMENTED'}
        detail={'initial_failure':failed,'prior_cycle':prior,'recovery_result':result,'recovery':status,
            'root_state':rig.arm.deploy_state,'prior_native_trace':prior_traces,'kind':kind,
            'external_mock_home':kind!='not_home','physical_q':camera.physical_q(),'physical_deploy_mm':rig.physical_mm()}
        base.save_rig(name,rig,detail)
        ok=kind in ('success','reconnect','busy','second_fault','hidden_jam','retry_source_lost','node')
        assert result['success']==ok,(name,result)
        if ok:
            assert not result.get('complete'),result
            assert rig.arm.deploy_state=='RETRACTED' and status['state']=='READY'
            assert rig.arm.cycle_status()['state']=='IDLE'
            assert rig.arm.visual_endpoint_lengths=={} and rig.arm.bend_status()['model']['accepted']==0
            if kind=='reconnect':
                assert status['history'][-1]['serial_generation']==1
                assert max(status['history'][-1]['samples'][0]['motor_frame_ids'])<=3
            if kind=='retry_source_lost': camera.mode='lost'
            if kind=='second_fault': camera.fault_phase='BEND_1';camera.fault='lost';camera.phase_counts={}
            retry=rig.arm.run_camera_cycle(plan);detail['retry']=retry;detail['retry_cycle']=rig.arm.cycle_status()
            assert retry['success']==(kind not in ('hidden_jam','second_fault','retry_source_lost')),(name,retry)
            assert detail['retry_cycle']['run_id']>prior['run_id']
            if retry['success']:
                assert retry['complete'] and rig.arm.deploy_state=='RETRACTED'
                assert sum(v*v for v in camera.truth())**0.5<=0.001
            else:
                assert not retry['complete'] and rig.arm.deploy_state=='FAULT'
                assert detail['retry_cycle']['stop_requested']
                wanted={'hidden_jam':'MOTION_PROGRESS_STALLED','second_fault':'CAMERA_TRACKING_LOST',
                    'retry_source_lost':'CAMERA_TRACKING_LOST'}[kind]
                assert wanted in retry['message'],retry
        else:
            assert not result.get('complete') and rig.arm.deploy_state=='FAULT'
            reasons={'lost':'CAMERA_TRACKING_LOST','fail_then_fix':'CAMERA_TRACKING_LOST',
                'stale':'CAMERA_FEEDBACK_STALE','map':'CAMERA_REFERENCE_MISMATCH',
                'replay':'RECOVERY_CAMERA_SOURCE_NOT_ADVANCED','frozen':'CAMERA_FEEDBACK_STALE',
                'margin':'RECOVERY_TIME_LIMIT','cancel':'RECOVERY_CANCELLED',
                'motor_error':'RECOVERY_MOTOR_FEEDBACK_INVALID','motor_drift':'RECOVERY_MOTOR_NOT_STATIONARY',
                'no_confirmation':'RECOVERY_REQUIRES_EXTERNAL_HOME_AND_FAULT_CLEAR_CONFIRMATION',
                'no_ack':'RECOVERY_REQUIRES_EXTERNAL_HOME_AND_FAULT_CLEAR_CONFIRMATION',
                'not_home':'RECOVERY_NOT_AT_RETRACTED_POSE','shortcut':'RECOVERY_API_REQUIRED',
                'quarantined':'QUARANTINED','persistent_crc':'BAD_CRC'}
            assert reasons[kind] in result['message'],(kind,result)
            assert rig.arm.cycle_status()==prior
            mark2=counts(rig)
            assert not rig.arm.run_camera_cycle(plan)['success'];assert not rig.arm.set_deploy(True)['success']
            rig.arm.step();assert mark2==counts(rig)
            if kind not in ('no_confirmation','no_ack','shortcut'):
                detail['passive_exchanges']=passive_since(rig,mark)
            else: assert mark==counts(rig)
            if kind=='fail_then_fix':
                camera.mode='normal';camera.last=None
                again=rig.arm.recover_camera_cycle(mechanically_confirmed=True,fault_cleared=True)
                detail['second_recovery']=rig.arm.recovery_status()
                assert again['success'] and not again['complete'],again
                assert detail['second_recovery']['attempt_id']==status['attempt_id']+1
                assert detail['second_recovery']['history'][0]['result']['success'] is False
                retry=rig.arm.run_camera_cycle(plan);detail['retry']=retry;detail['retry_cycle']=rig.arm.cycle_status()
                assert retry['success'] and retry['complete'],retry
        base.save_rig(name,rig,detail)
    finally: rig.close()


def cases(base):
    result=[]
    def add(name,**kw): result.append((name,lambda n=name,k=kw:exercise(base,n,**k)))
    add('recovery_persistent_camera',kind='lost')
    for phase in ('DEPLOY','BEND_1','NEUTRALIZE','RETRACT','VERIFY_END'):
        add('recovery_from_'+phase.lower(),phase=phase)
    for kind in ('stale','map','replay','frozen','margin','cancel','busy','motor_error','motor_drift',
                 'no_confirmation','no_ack','not_home','shortcut','second_fault','hidden_jam','retry_source_lost','fail_then_fix','node'):
        add('recovery_'+kind,kind=kind)
    add('recovery_quarantined_not_reopened',phase='BEND_1',fault='motor_crc',kind='quarantined')
    add('recovery_reconnected',phase='BEND_1',fault='motor_crc',kind='reconnect')
    add('recovery_crc_persists_after_reconnect',phase='BEND_1',fault='motor_crc',kind='persistent_crc')
    add('recovery_from_cancel',fault='cancel')
    result.append(('recovery_process_restart',lambda:process_restart(base)))
    return result


def process_restart(base):
    rig,camera,plan,options=setup(base)
    try:
        before=rig.arm.run_camera_cycle(plan);assert not before['success']
        old_cycle=rig.arm.cycle_status();assert rig.physical_mm()>40
        camera.fault_phase=None;camera.fault=None;camera.last=None
        rig.arm.close();rig.active.reconnect_adapter();rig.deploy.reconnect_adapter()
        rig.construct(control_options=options,deploy_timeout_s=1.6)
        assert rig.arm.deploy_state=='UNKNOWN' and rig.arm.deployed is None
        mark=counts(rig);restart=rig.arm.run_camera_cycle(plan)
        assert not restart['success'] and 'CONFIRMED_RETRACTED_START' in restart['message']
        assert counts(rig)==mark
        attempt=rig.arm.recover_camera_cycle(mechanically_confirmed=True,fault_cleared=True)
        assert not attempt['success'] and 'RECOVERY_NOT_AT_RETRACTED_POSE' in attempt['message'],attempt
        passive_since(rig,mark)
        external_home(rig,camera)
        recovered=rig.arm.recover_camera_cycle(mechanically_confirmed=True,fault_cleared=True)
        assert recovered['success'] and not recovered['complete'],recovered
        retry=rig.arm.run_camera_cycle(plan);assert retry['success'] and retry['complete'],retry
        base.save_rig('recovery_process_restart',rig,{'initial_failure':before,'old_cycle':old_cycle,
            'restart_without_reference':restart,'recovery_while_still_extended':attempt,
            'recovery_result':recovered,'recovery':rig.arm.recovery_status(),
            'retry':retry,'retry_cycle':rig.arm.cycle_status(),'root_state':rig.arm.deploy_state})
    finally: rig.close()
