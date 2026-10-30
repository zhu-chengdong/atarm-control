"""Physical-state camera fixture for bounded deployment visual correction.

Camera truth derives from simulated physical rotor travel through an independent
plant map, never from command or reported q. All numbers are synthetic metres.
"""
from dataclasses import asdict, replace
import math
import random
import struct
import time
from atarm_controller.camera_feedback import CameraPose,CameraGateConfig
from atarm_controller.visual_servo import VisualServoConfig
from camera_cases import goals


class PlantCamera:
    def __init__(self,rig,gain=1.0,bias=0.0,noise=0.0,seed=1,mode='normal',period=0.01):
        self.rig=rig; self.gain=gain; self.bias=bias; self.noise=noise
        self.mode=mode; self.period=period; self.random=random.Random(seed)
        self.last=None; self.counter=0; self.logs=[]

    def truth(self):
        x=self.gain*self.rig.physical_mm()/1000+self.bias
        if self.mode=='jammed': x=0.0
        if self.mode=='saturated': x=min(x,0.035)
        return x

    def __call__(self):
        now=time.monotonic()
        if self.last and now-self.last.capture_s<self.period: return self.last
        self.counter+=1
        physical=self.rig.physical_mm()/1000
        truth=self.truth()
        measured=truth+self.random.uniform(-self.noise,self.noise)
        if self.mode=='outliers' and self.counter%5==0: measured+=0.0015
        sample=CameraPose(self.counter,now,'sim_base_fixed','sim_tip_camera','sim-run-1',True,
            (measured,0.008 if self.mode=='off_axis' else 0.0,0.0))
        if self.mode=='lost_correction' and physical>0.051: sample=replace(sample,tracking_ok=False)
        if self.mode=='stale_correction' and physical>0.051: sample=replace(sample,capture_s=now-1.0)
        if self.mode=='motor_stall' and physical>0.0509: self.rig.deploy.mode='stall'
        if self.mode=='stop_link_failure' and physical>0.051:
            sample=replace(sample,tracking_ok=False)
            self.rig.deploy.mode='timeout'
        self.last=sample
        self.logs.append({'physical_motor_m':physical,'true_camera_x_m':truth,'pose':asdict(sample)})
        return sample


def config(**changes):
    values=dict(jacobian_column=(1.0,0.0,0.0),min_length_m=-0.012,max_length_m=0.075,
        max_offset_m=0.02,max_total_correction_m=0.04,max_step_m=0.002,gain=0.6,
        deadband_m=0.0005,resume_error_m=0.0008,min_step_m=0.00002,
        settle_wait_s=0.004,update_period_s=0.005,max_duration_s=1.2,
        min_improvement_m=0.0003,max_adjustments=16,no_progress_steps=3,filter_frames=3)
    values.update(changes)
    return VisualServoConfig(**values)


def exercise(base,name,gain=0.85,bias=0.0,noise=0.0,seed=1,mode='normal',deploy=True,
             expected=True,expected_code=None,changes=None,repeated=0,period=0.01,motor_mode='normal',arm_timeout_s=1.4):
    rig=base.Rig()
    try:
        camera=PlantCamera(rig,gain,bias,noise,seed,mode,period)
        cfg=config(**(changes or {}))
        camcfg=CameraGateConfig(max_age_s=max(0.06,2*period),position_tolerance_m=0.001,
            orientation_tolerance_rad=0.05,settle_s=0.025,progress_min_m=0.0003,
            progress_timeout_s=0.5,max_speed_m_s=4.0,step_margin_m=0.004)
        options={'camera_required':True,'camera_feedback':camera,'camera_goals':goals(),'camera_config':camcfg}
        lib=__import__('atarm_controller.arm_controller_lib',fromlist=['*'])
        if hasattr(lib,'BoundedVisualServo'): options['visual_servo_config']=cfg
        rig.construct(control_options=options,deploy_timeout_s=arm_timeout_s)
        rig.home()
        rig.deploy.mode=motor_mode
        if not deploy:
            assert rig.call(True)['success']
        before=rig.physical_mm()
        before_truth=camera.truth()
        started=time.monotonic()
        response=rig.call(deploy)
        elapsed=time.monotonic()-started
        evidence=getattr(rig.arm,'visual_servo_status',lambda:{'integration_missing':True})()
        detail={'response':response,'state':rig.arm.deploy_state,'elapsed_s':elapsed,
            'gain':gain,'bias_m':bias,'noise_bound_m':noise,'seed':seed,'camera_mode':mode,
            'before_motor_mm':before,'final_motor_mm':rig.physical_mm(),'before_true_camera_m':before_truth,
            'final_true_camera_m':camera.truth(),'true_error_m':abs(camera.truth()-(0.05 if deploy else 0.0)),
            'camera_evidence':rig.arm.camera_status(),'visual_servo':evidence,'camera_samples':camera.logs,
            'motor_feedback':rig.arm.motors[3].get_current_status()}
        base.save_rig(name,rig,detail)
        assert response['success']==expected,detail
        if expected:
            assert detail['true_error_m']<=0.001+noise,detail
            assert evidence['adjustments']>0,detail
            history=evidence['history']
            assert all(abs(p['delta_m'])<=cfg.max_step_m+1e-12 for p in history),detail
            assert all(p['capture_s']>=p['settled_since_s']+cfg.settle_wait_s for p in history),detail
            signs=[math.copysign(1,p['delta_m']) for p in history]
            reversals=sum(a!=b for a,b in zip(signs,signs[1:]))
            detail['correction_direction_reversals']=reversals
            assert reversals<=1,detail
            if noise:
                assert reversals==0,detail
            positions=[rig.physical_mm()]
            repeats=[]
            for _ in range(repeated):
                time.sleep(0.012)
                reply=rig.call(deploy)
                assert reply['success'],(detail,reply)
                positions.append(rig.physical_mm())
                repeats.append(rig.arm.visual_servo_status())
            detail['repeated_request_motor_mm']=positions
            detail['repeated_request_servo']=repeats
            assert max(positions)-min(positions)<=0.05,detail
            assert all(p['adjustments']==0 for p in repeats),detail
        else:
            assert rig.arm.deploy_state=='FAULT',detail
            if expected_code: assert expected_code in response['message'],detail
            assert elapsed<=min(cfg.max_duration_s,arm_timeout_s)+0.45,detail
            assert evidence.get('stop_requested'),detail
            assert len(evidence['stop_receipts'])==4,detail
            assert all(m.cmd.kp==m.cmd.kd==m.cmd.dq==m.cmd.tau==0.0 for m in rig.arm.motors),detail
            # Native wire bytes show explicit zero-effort requests, not just a Boolean stop flag.
            for board in (rig.active,rig.deploy):
                raw=bytes.fromhex(board.logs[-1]['request_hex'])
                assert all(struct.unpack_from('<h',raw,offset)[0]==0 for offset in (12,14,20,22)),detail
            mark=(len(rig.active.logs),len(rig.deploy.logs))
            again=rig.call(deploy)
            assert not again['success'],detail
            rig.arm.step()
            time.sleep(0.025)
            assert mark==(len(rig.active.logs),len(rig.deploy.logs)),detail
            detail['no_commands_after_fault']=True
            if mode=='stop_link_failure':
                assert not evidence['stop_receipts'][0]['reply_valid'],detail
        base.save_rig(name,rig,detail)
    finally:
        rig.close()


def cases(base):
    result=[]
    def add(name,**kwargs):
        result.append((name,lambda n=name,k=kwargs:exercise(base,n,**k)))
    add('servo_under_gain')
    add('servo_over_gain',gain=1.15)
    add('servo_bias_deploy',gain=1.0,bias=0.004)
    add('servo_bias_retract',gain=1.0,bias=0.004,deploy=False)
    add('servo_larger_gain_error',gain=0.75)
    add('servo_gain_beyond_limit',gain=0.7,expected=False,expected_code='VISUAL_SERVO_OFFSET_LIMIT')
    add('servo_motor_lag',motor_mode='lag_follow')
    for seed in (7,19,41):
        add('servo_noise_seed_'+str(seed),noise=0.0004,seed=seed,repeated=3)
    add('servo_outliers',noise=0.0002,mode='outliers')
    add('servo_camera_30hz',period=1/30)
    add('servo_jammed',mode='jammed',expected=False,expected_code='VISUAL_SERVO_NO_PROGRESS')
    add('servo_saturated',mode='saturated',expected=False,expected_code='VISUAL_SERVO_NO_PROGRESS')
    add('servo_wrong_jacobian',changes={'jacobian_column':(-1.0,0.0,0.0)},expected=False,expected_code='VISUAL_SERVO_NO_PROGRESS')
    add('servo_travel_limit',changes={'max_length_m':0.053},expected=False,expected_code='VISUAL_SERVO_TRAVEL_LIMIT')
    add('servo_offset_limit',changes={'max_offset_m':0.003},expected=False,expected_code='VISUAL_SERVO_OFFSET_LIMIT')
    add('servo_total_limit',changes={'max_total_correction_m':0.003},expected=False,expected_code='VISUAL_SERVO_TOTAL_TRAVEL_LIMIT')
    add('servo_iteration_limit',changes={'max_adjustments':2},expected=False,expected_code='VISUAL_SERVO_ITERATION_LIMIT')
    add('servo_time_limit',changes={'max_duration_s':0.04},expected=False,expected_code='VISUAL_SERVO_TIMEOUT')
    add('servo_off_axis',mode='off_axis',expected=False,expected_code='VISUAL_SERVO_UNCONTROLLABLE_TRANSLATION')
    add('servo_tracking_lost',mode='lost_correction',expected=False,expected_code='CAMERA_TRACKING_LOST')
    add('servo_stale',mode='stale_correction',expected=False,expected_code='CAMERA_FEEDBACK_STALE')
    add('servo_motor_stall',mode='motor_stall',expected=False,expected_code='MOTION_PROGRESS_STALLED')
    add('servo_stop_link_failure',mode='stop_link_failure',expected=False,expected_code='CAMERA_TRACKING_LOST')
    add('servo_false_encoder',motor_mode='coherent_false',expected=False,expected_code='VISUAL_SERVO_NO_PROGRESS')
    add('servo_operation_timeout',arm_timeout_s=0.12,expected=False,expected_code='VISUAL_SERVO_OPERATION_TIMEOUT')
    add('servo_initial_outside_limits',changes={'min_length_m':0.001},expected=False,
        expected_code='VISUAL_SERVO_INITIAL_POSITION_OUT_OF_LIMIT')
    return result
