"""Independent synthetic camera: reads only simulated physical state, never q reports.

The 1:1 translation used here is a test fixture, not arm calibration. Base and
pan/arc mechanisms are fixed. No image processing, SLAM or visual servo runs.
"""
from dataclasses import asdict, replace
import math
import time
from atarm_controller.camera_feedback import CameraPose,CameraGoal,CameraGateConfig


class PhysicalCamera:
    def __init__(self, physical_position):
        self.physical_position=physical_position
        self.mode='normal'
        self.last=None
        self.counter=0
        self.logs=[]
        self.period_s=0.01

    def __call__(self):
        now=time.monotonic()
        if self.mode=='exception':
            raise RuntimeError('Injected unavailable pose producer')
        if self.mode=='missing':
            return None
        if self.mode=='freeze_goal' and self.last and self.last.position_m[0]>=0.049:
            return self.last
        if self.last and now-self.last.capture_s<self.period_s and self.mode not in ('stale','future'):
            return self.last
        self.counter+=1
        x=self.physical_position()
        raw_x=x
        y=0.0
        quat=(0.0,0.0,0.0,1.0)
        if self.mode=='static': x=0.0
        if self.mode=='partial': x*=0.5
        if self.mode=='jump' and x>0.001: x=0.5
        if self.mode=='off_axis': y=0.01
        if self.mode=='wrong_orientation' and x>=0.049:
            quat=(0.0,0.0,math.sin(0.2/2),math.cos(0.2/2))
        if self.mode=='jitter': x+=0.0002 if self.counter%2 else -0.0002
        stamp=now-1.0 if self.mode=='stale' else now+1.0 if self.mode=='future' else now
        epoch='reset-map' if self.mode=='epoch' else 'sim-run-1'
        parent='other_world' if self.mode=='frame' else 'sim_base_fixed'
        tracking=self.mode!='lost' and not (self.mode=='lost_goal' and x>=0.049)
        sample=CameraPose(self.counter,stamp,parent,'sim_tip_camera',epoch,tracking,(x,y,0.0),quat)
        if self.mode=='replay_goal' and self.last and x>=0.049:
            sample=replace(sample,frame_id=self.last.frame_id-1)
        if self.mode=='old_capture_goal' and self.last and x>=0.049:
            sample=replace(sample,capture_s=self.last.capture_s)
        if self.mode=='nan': sample=replace(sample,position_m=(float('nan'),0.0,0.0))
        if self.mode=='bad_quaternion': sample=replace(sample,quaternion_xyzw=(0.0,0.0,0.0,0.0))
        self.last=sample
        self.logs.append({'physical_x_m':raw_x,'mode':self.mode,'pose':asdict(sample)})
        return sample


def goals():
    return {False:CameraGoal('sim_base_fixed','sim_tip_camera','sim-run-1',(0.0,0.0,0.0)),
            True:CameraGoal('sim_base_fixed','sim_tip_camera','sim-run-1',(0.05,0.0,0.0))}


def exercise(base,name,mode='normal',motor='normal',deploy=True,expected=True):
    rig=base.Rig()
    try:
        camera=PhysicalCamera(lambda:rig.physical_mm()/1000.0)
        config=CameraGateConfig(max_age_s=0.04,position_tolerance_m=0.001,
            orientation_tolerance_rad=0.05,settle_s=0.015,progress_min_m=0.0005,
            progress_timeout_s=0.09,max_speed_m_s=4.0,step_margin_m=0.003)
        options={'camera_required':True,'camera_feedback':camera,'camera_goals':goals(),'camera_config':config}
        if mode=='no_provider': options['camera_feedback']=None
        if mode=='no_goal': options['camera_goals']={}
        if mode=='slow': camera.period_s=1.0/30.0
        rig.construct(control_options=options,deploy_timeout_s=0.3)
        rig.home()
        if not deploy:
            assert rig.call(True)['success']
        camera.mode=mode
        rig.deploy.mode=motor
        before=rig.physical_mm()
        mark=len(rig.deploy.logs)
        started=time.monotonic()
        response=rig.call(deploy)
        detail={'response':response,'state':rig.arm.deploy_state,'motor_mode':motor,'camera_mode':mode,
            'before_mm':before,'after_mm':rig.physical_mm(),'elapsed_s':time.monotonic()-started,
            'motor_feedback':rig.arm.motors[3].get_current_status(),
            'camera_evidence':getattr(rig.arm,'camera_status',lambda:{'integration_missing':True})(),
            'camera_samples':camera.logs,'post_call_tx_count':len(rig.deploy.logs)-mark}
        base.save_rig(name,rig,detail)
        assert response['success']==expected,detail
        if expected:
            assert rig.arm.deploy_state==('DEPLOYED' if deploy else 'RETRACTED'),detail
            assert detail['camera_evidence']['ready'],detail
            assert abs(rig.physical_mm()-(50.0 if deploy else 0.0))<0.05,detail
            assert detail['camera_evidence']['stable_frames']>=2,detail
        else:
            assert rig.arm.deploy_state=='FAULT',detail
            assert 'CAMERA_' in response['message'],detail
            expected_fault={
                'static':'CAMERA_PROGRESS_STALLED','partial':'CAMERA_PROGRESS_STALLED',
                'off_axis':'CAMERA_PROGRESS_STALLED','jump':'CAMERA_POSE_DISCONTINUITY',
                'wrong_orientation':'CAMERA_TARGET_NOT_CONFIRMED','freeze_goal':'CAMERA_FEEDBACK_STALE',
                'lost_goal':'CAMERA_TRACKING_LOST','replay_goal':'CAMERA_FRAME_REPLAY_OR_MUTATION',
                'old_capture_goal':'CAMERA_CAPTURE_TIME_NOT_ADVANCING','missing':'CAMERA_FEEDBACK_MISSING',
                'stale':'CAMERA_FEEDBACK_STALE','future':'CAMERA_TIMESTAMP_IN_FUTURE',
                'epoch':'CAMERA_REFERENCE_MISMATCH','frame':'CAMERA_REFERENCE_MISMATCH',
                'lost':'CAMERA_TRACKING_LOST','nan':'CAMERA_POSE_INVALID',
                'bad_quaternion':'CAMERA_POSE_INVALID','exception':'CAMERA_PROVIDER_ERROR',
                'no_provider':'CAMERA_FEEDBACK_MISSING','no_goal':'CAMERA_GOAL_INVALID'}
            if mode in expected_fault:
                assert expected_fault[mode] in response['message'],detail
            if mode in ('missing','stale','future','epoch','frame','lost','nan','bad_quaternion','exception','no_provider','no_goal'):
                assert detail['post_call_tx_count']==0,detail
            mark=len(rig.deploy.logs)
            again=rig.call(deploy)
            assert not again['success'] and len(rig.deploy.logs)==mark,detail
        if motor=='coherent_false':
            assert abs(rig.physical_mm()-before)<0.05,detail
            assert detail['motor_feedback']['feedback_valid'],detail
            assert detail['motor_feedback']['motion_guard']['fault'] is None,detail
            assert rig.arm.motion_progress['seen'],detail
            assert detail['camera_evidence']['fault']=='CAMERA_PROGRESS_STALLED',detail
            assert abs(detail['motor_feedback']['actuate_len']-(0.05 if deploy else 0.0))<0.001,detail
    finally:
        rig.close()


def repeated(base):
    rig=base.Rig()
    try:
        camera=PhysicalCamera(lambda:rig.physical_mm()/1000.0)
        rig.construct(control_options={'camera_required':True,'camera_feedback':camera,'camera_goals':goals(),
            'camera_config':CameraGateConfig(max_age_s=0.05,settle_s=0.015,max_speed_m_s=4.0,step_margin_m=0.003)})
        rig.home()
        for endpoint in (False,True,True,False,False):
            assert rig.call(endpoint)['success']
        base.save_rig('camera_repeated_endpoints',rig,{'state':rig.arm.deploy_state,
            'camera_evidence':rig.arm.camera_status(),'camera_samples':camera.logs})
    finally:
        rig.close()


def cases(base):
    result=[]
    for deploy in (True,False):
        suffix='deploy' if deploy else 'retract'
        result.append(('camera_normal_'+suffix,lambda d=deploy,n='camera_normal_'+suffix:exercise(base,n,deploy=d)))
        result.append(('camera_coherent_false_'+suffix,lambda d=deploy,n='camera_coherent_false_'+suffix:
            exercise(base,n,motor='coherent_false',deploy=d,expected=False)))
    for mode in ('static','partial','off_axis','wrong_orientation','freeze_goal','lost_goal','replay_goal',
                 'old_capture_goal','missing','stale','future','epoch','frame','lost','nan',
                 'bad_quaternion','exception','no_provider','no_goal','jitter','slow','jump'):
        name='camera_'+mode
        result.append((name,lambda m=mode,n=name:exercise(base,n,mode=m,expected=m in ('jitter','slow'))))
    result.append(('camera_repeated_endpoints',lambda:repeated(base)))
    return result
