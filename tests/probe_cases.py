"""Pre-retry small-motion probes; synthetic camera truth is never in control."""
from dataclasses import replace,asdict
from copy import deepcopy
import importlib,math,time
from recovery_cases import setup,counts,external_home
from bend_cases import B,H
from atarm_controller.camera_recovery import RecoveryConfig


def config(**changes):
    from atarm_controller.recovery_probe import ProbeConfig,ProbeWaypoint
    # Explicit calibration of the synthetic plant; not read from the live camera.
    xy=tuple(tuple(sum(B[k][i]*H[k][j] for k in range(3))/1.5 for j in range(3)) for i in range(2))
    offsets=[(0.0,0.0,0.0,0.002)]
    offsets+=[tuple(B[k][0]*0.0015 for k in range(3))+(0.002,),offsets[0]]
    offsets+=[tuple(B[k][1]*0.0015 for k in range(3))+(0.002,),offsets[0],(0.0,)*4]
    labels=('EXTEND_SMALL','BEND_U','RETURN_U','BEND_V','RETURN_V','RETURN_HOME')
    points=tuple(ProbeWaypoint(label,q,tuple(sum(xy[i][j]*q[j] for j in range(3)) for i in range(2))+(q[3],))
        for label,q in zip(labels,offsets))
    values=dict(waypoints=points,max_excursion_m=(0.0015,0.0015,0.0015,0.002),
        max_total_joint_travel_m=0.02,max_step_m=0.0002,speed_m_s=0.05,period_s=0.002,
        settle_s=0.025,timeout_s=1.5,stage_timeout_s=0.3,no_progress_s=0.08,
        motor_tolerance_m=0.00008,camera_tolerance_m=0.00025,camera_noise_bound_m=0.00004,
        min_camera_motion_m=0.0003,max_pose_age_s=0.07)
    values.update(changes);return ProbeConfig(**values)


class ProbeCamera:
    def __init__(self,raw,rig,kind):
        self.raw=raw;self.rig=rig;self.kind=kind;self.counts={};self.last_id=None;self.triggered=False
        self.freeze=None;self.injected_s=None
    def __call__(self):
        pose=self.raw();p=getattr(self.rig.arm,'motion_probe',None)
        if p is None or not p.active:
            if p and p.state=='SUCCEEDED' and self.kind=='post_loss': return replace(pose,tracking_ok=False)
            return pose
        phase=p.phase
        if pose.frame_id!=self.last_id:
            self.counts[phase]=self.counts.get(phase,0)+1;self.last_id=pose.frame_id
        target='RETURN_HOME' if self.kind=='return_jam' else 'BEND_U'
        if phase==target and self.counts.get(phase,0)>=2:
            if not self.triggered: self.injected_s=time.monotonic()
            self.triggered=True
            if self.kind=='lost': pose=replace(pose,tracking_ok=False)
            elif self.kind=='stale': pose=replace(pose,capture_s=pose.capture_s-1)
            elif self.kind=='map': pose=replace(pose,map_epoch='probe-map-reset')
            elif self.kind=='replay': pose=replace(pose,frame_id=1)
            elif self.kind=='frozen':
                if self.freeze is None: self.freeze=pose
                pose=self.freeze
            elif self.kind=='cancel': self.rig.arm.cancel_camera_recovery()
            elif self.kind=='return_jam': self.rig.deploy.mode='stall'
            elif self.kind=='bad_crc': self.rig.active.motor_modes[0]='bad_crc'
            elif self.kind=='timeout': self.rig.active.motor_modes[0]='timeout'
            elif self.kind=='busy' and self.counts[phase]==2:
                arm=self.rig.arm;mark=counts(self.rig)
                assert not arm.run_camera_cycle(self.raw.plan)['success']
                assert not arm.recover_camera_cycle(mechanically_confirmed=True,fault_cleared=True)['success']
                assert not arm.set_deploy(False)['success']
                assert not arm.move_bend_camera(self.raw.plan.bend_goals[0])['success']
                assert not arm.step();assert mark==counts(self.rig)
        if self.kind=='reverse':
            pose=replace(pose,position_m=tuple(-v for v in pose.position_m))
        if self.kind=='camera_no_motion': pose=replace(pose,position_m=(0.0,0.0,0.0))
        return pose


def exercise(base,name,kind='healthy',changes=None):
    lib=importlib.import_module('atarm_controller.arm_controller_lib')
    options={}
    if hasattr(lib,'RecoveryMotionProbe'):
        options={'probe_config':config(**(changes or {})),
            'recovery_config':RecoveryConfig(timeout_s=2.0,period_s=0.002,settle_s=0.025,max_pose_age_s=0.07)}
        if kind=='recovery_timeout': options['recovery_config']=replace(options['recovery_config'],timeout_s=0.055)
        if kind=='bad_model':
            c=options['probe_config']
            options['probe_config']=replace(c,waypoints=tuple(replace(p,camera_offset_m=tuple(2*v for v in p.camera_offset_m)) for p in c.waypoints))
    rig,raw,plan,ignored=setup(base,extra_options=options)
    try:
        raw.fault_phase=None;raw.fault=None;raw.last=None
        camera=ProbeCamera(raw,rig,kind);rig.arm.camera_feedback=camera
        rig.arm._fault('SIMULATED_FAULT_REQUIRING_RECOVERY')
        prior=rig.arm.cycle_status();mark=counts(rig)
        assert not rig.arm.run_camera_cycle(plan)['success'];assert mark==counts(rig)
        if kind in ('deploy_stall','deploy_false','fix_after_failure'): rig.deploy.mode='coherent_false' if kind=='deploy_false' else 'stall'
        if kind.startswith('bend_stall_') or kind.startswith('bend_false_'):
            index=int(kind.rsplit('_',1)[-1]);rig.active.motor_modes[(1,0,2)[index]]='stall' if kind.startswith('bend_stall') else 'coherent_false'
        if kind=='noise': raw.noise=0.00002
        if kind=='node':
            import types,json
            node=object.__new__(importlib.import_module('atarm_controller.arm_controller_node').UnitreeTensegrityControllerNode)
            node.arm=rig.arm;response=types.SimpleNamespace(success=False,message='')
            node._recover_service(None,response);result=rig.arm.recovery_status()['result']
            assert response.success==result['success']
            published={}
            for key in ('actuation_lengths_curr_pub','actuation_lengths_trgt_pub','motor_angles_curr_pub','controller_state_pub'):
                setattr(node,key,types.SimpleNamespace(publish=lambda msg,k=key:published.update({k:msg.data})))
            node._publish_status_callback()
            assert json.loads(published['controller_state_pub'])['recovery_probe']['state']=='SUCCEEDED'
        else: result=rig.arm.recover_camera_cycle(mechanically_confirmed=True,fault_cleared=True)
        status=rig.arm.recovery_status();probe=rig.arm.probe_status() if hasattr(rig.arm,'probe_status') else {'enabled':False}
        detail={'kind':kind,'recovery_result':result,'recovery':status,'probe':probe,'prior_cycle':prior,
            'root_state':rig.arm.deploy_state,'physical_q':raw.physical_q(),'physical_deploy_mm':rig.physical_mm(),
            'true_p':raw.truth(),'injected_s':camera.injected_s}
        base.save_rig(name,rig,detail)
        success=kind in ('healthy','noise','busy','node')
        assert result['success']==success,(kind,result)
        if success:
            assert result['ready_for_retry'] and not result['complete']
            assert probe['state']=='SUCCEEDED' and len(probe['stages'])==6
            assert all(s['verified'] for s in probe['stages'])
            assert all(abs(v)<0.00002 for v in raw.physical_q()) and abs(rig.physical_mm())<0.02
            assert rig.arm.bend_status()['model']['accepted']==0
            retry=rig.arm.run_camera_cycle(plan);detail['retry']=retry;detail['retry_cycle']=rig.arm.cycle_status()
            assert retry['success'] and retry['complete'],retry
        else:
            assert rig.arm.deploy_state=='FAULT' and not result['ready_for_retry'] and not result['complete']
            assert rig.arm.cycle_status()==prior
            assert probe['state'] in ('FAULT','REQUIRED') and status['stop_requested'] and len(status['stop_receipts'])==4
            if kind.startswith('bend_stall') or kind in ('deploy_stall','return_jam','fix_after_failure'):
                reasons=('PROBE_MOTOR_NOT_TRACKING','PROBE_MOTOR_FEEDBACK_INVALID')
            elif kind.startswith('bend_false') or kind in ('deploy_false','camera_no_motion','reverse','bad_model'):
                reasons=('PROBE_CAMERA_NO_RESPONSE','PROBE_CAMERA_DIRECTION_MISMATCH','PROBE_CAMERA_RESPONSE_MISMATCH')
            else:
                reasons=({'lost':('CAMERA_TRACKING_LOST',),'post_loss':('CAMERA_TRACKING_LOST',),
                    'stale':('CAMERA_FEEDBACK_STALE',),'map':('CAMERA_REFERENCE_MISMATCH',),
                    'replay':('PROBE_CAMERA_SOURCE_NOT_ADVANCED',),
                    'frozen':('CAMERA_FEEDBACK_STALE',),'cancel':('RECOVERY_CANCELLED',),
                    'probe_timeout':('PROBE_TIME_LIMIT',),'stage_timeout':('PROBE_STAGE_TIME_LIMIT',),
                    'recovery_timeout':('RECOVERY_TIME_LIMIT',),
                    'bad_crc':('BAD_CRC',),'timeout':('TIMEOUT',)})[kind]
            assert any(reason in result['message'] for reason in reasons),(kind,result)
            mark2=counts(rig);assert not rig.arm.run_camera_cycle(plan)['success'];rig.arm.step();assert counts(rig)==mark2
            detail['retry_blocked_without_commands']=True
            if kind=='fix_after_failure':
                rig.deploy.mode='normal';external_home(rig,raw)
                second=rig.arm.recover_camera_cycle(mechanically_confirmed=True,fault_cleared=True)
                detail['second_recovery']=rig.arm.recovery_status();detail['second_probe']=rig.arm.probe_status()
                assert second['success'] and not second['complete'],second
                retry=rig.arm.run_camera_cycle(plan);detail['retry']=retry
                assert retry['success'] and retry['complete'],retry
        base.save_rig(name,rig,detail)
    finally: rig.close()


def cases(base):
    result=[]
    kinds=['healthy','noise','busy','deploy_stall','deploy_false','camera_no_motion','reverse',
        'lost','stale','map','replay','frozen','cancel','return_jam','bad_crc','timeout','post_loss','fix_after_failure']
    kinds += ['bend_'+mode+'_'+str(i) for mode in ('stall','false') for i in range(3)]
    for kind in kinds: result.append(('probe_'+kind,lambda k=kind:exercise(base,'probe_'+k,kind=k)))
    for kind,changes in [('probe_timeout',{'timeout_s':0.06}),('stage_timeout',{'stage_timeout_s':0.03}),
        ('recovery_timeout',{}),('node',{}),('bad_model',{})]:
        result.append(('probe_'+kind,lambda k=kind,c=changes:exercise(base,'probe_'+k,kind=k,changes=c)))
    return result
