"""Three physical simulated rotors, coupled cable plant and independent camera."""
from dataclasses import asdict,replace
import math,random,struct,time,types
from atarm_controller.camera_feedback import CameraPose,CameraGoal,CameraGateConfig
from atarm_controller.online_jacobian import JacobianUpdateConfig
from atarm_controller.coupled_bend import BendServoConfig,CoupledJacobian,mv,inverse_action,length,sub,singulars

B=((1.0,0.0),(-0.5,math.sqrt(3)/2),(-0.5,-math.sqrt(3)/2))
H=((0.65,0.25,-0.08),(0.15,0.95,0.18),(-0.08,0.22,0.72))

def bend_config(noise=0.0001,adaptive=True,**changes):
    values=dict(cable_mix=B,initial_jacobian=((1.15,0.0),(0.0,1.15),(0.0,0.0)),
        min_lengths_m=(-0.04,)*3,max_lengths_m=(0.04,)*3,
        online_update=JacobianUpdateConfig(position_noise_bound_m=max(noise,0.0001),min_norm=0.15,max_norm=3.0,
            min_actuation_m=0.001,learning_rate=0.5) if adaptive else None,
        camera_noise_bound_m=max(noise,0.0001),
        speed_m_s=0.6,period_s=0.002,settle_s=0.006,extra_settle_s=0.004,
        timeout_s=2.0,motor_timeout_s=0.12,max_step_m=0.0025,max_steps=24,no_progress_steps=4)
    values.update(changes);return BendServoConfig(**values)


class BendCamera:
    def __init__(self,rig,plant='coupled',noise=0.0,seed=1):
        self.rig=rig;self.plant=plant;self.noise=noise;self.random=random.Random(seed)
        self.last=None;self.counter=0;self.logs=[];self.mode='normal';self.period=0.01
        self.motor_fault=None
        self.ownership_checked=False
    def physical_q(self):
        return tuple(sign*self.rig.active.position[mid]*6.2832/16384/9.1*0.026
            for mid,sign in zip((1,0,2),(1,1,-1)))
    def truth(self):
        q=self.physical_q();u=inverse_action(B,q);xy=inverse_action(B,mv(H,q))
        if self.plant=='nonlinear':
            xy=(xy[0]+5*u[0]*u[1],xy[1]+3*(u[0]*u[0]-u[1]*u[1]))
        if self.plant=='changing':
            excess=max(0,u[0]-0.005);xy=(xy[0]-0.3*excess,xy[1]+0.25*excess)
        if self.plant=='jammed': xy=(0.0,0.0)
        if self.plant=='singular': xy=(xy[0],0.0)
        return (xy[0],xy[1],self.rig.physical_mm()/1000)
    def __call__(self):
        now=time.monotonic()
        if self.last and now-self.last.capture_s<self.period: return self.last
        self.counter+=1;truth=self.truth()
        p=tuple(v+self.random.uniform(-self.noise,self.noise) for v in truth)
        sample=CameraPose(self.counter,now,'sim_base_fixed','sim_tip_camera','bend-map-1',True,p)
        bending=hasattr(self.rig.arm,'_bend_motion_active') and self.rig.arm._bend_motion_active
        if bending and self.mode=='ownership' and not self.ownership_checked:
            arm=self.rig.arm;mark=(len(self.rig.active.logs),len(self.rig.deploy.logs))
            assert not arm.set_deploy(False)['success']
            assert not arm.confirm_retracted_reference(mechanically_confirmed=True)['success']
            assert not arm.move_bend_camera(camera_goals()[True])['success']
            assert not arm.update_bend(0.1,0.2)
            assert not arm.step()
            try: arm.apply_active_micro_adjust([0.001]*3)
            except Exception as exc: assert 'BEND_DIRECT_ADJUST_DISABLED' in str(exc)
            else: raise AssertionError('Raw micro-adjust must not bypass closed loop')
            assert mark==(len(self.rig.active.logs),len(self.rig.deploy.logs))
            self.ownership_checked=True
        if bending and self.mode=='lost': sample=replace(sample,tracking_ok=False)
        if bending and self.mode=='lost_late' and max(abs(v) for v in self.physical_q())>0.005:
            sample=replace(sample,tracking_ok=False)
        if bending and self.mode=='stale': sample=replace(sample,capture_s=now-1.0)
        if bending and self.mode=='epoch': sample=replace(sample,map_epoch='reset-map')
        if bending and self.mode=='replay' and self.last: sample=replace(sample,frame_id=self.last.frame_id)
        if bending and self.mode=='deploy_drift':
            self.rig.deploy.motor_modes[2]='external_drift'
            self.rig.deploy.forced_position_offset_raw=int(0.0004*9.1/0.026*16384/6.2832)
        if bending and self.motor_fault and max(abs(v) for v in self.physical_q())>0.0005:
            self.rig.active.motor_modes[0]=self.motor_fault
        self.last=sample;self.logs.append({'physical_q':self.physical_q(),'true_p':truth,'pose':asdict(sample)})
        return sample


def camera_goals():
    return {d:CameraGoal('sim_base_fixed','sim_tip_camera','bend-map-1',(0.0,0.0,0.05 if d else 0.0)) for d in (False,True)}


def exercise(base,name,plant='coupled',noise=0.0,seed=1,adaptive=True,expected=True,
             changes=None,mode='normal',motor=None,repeat=0,waypoints=False,use_node=False,expected_reason=None):
    rig=base.Rig()
    try:
        camera=BendCamera(rig,plant,noise,seed)
        cfg=bend_config(max(noise*math.sqrt(3),0.0001),adaptive,**(changes or {}))
        cc=CameraGateConfig(max_age_s=0.07,position_tolerance_m=0.001,settle_s=0.025,
            progress_timeout_s=0.7,progress_min_m=0.0003,max_speed_m_s=4.0,step_margin_m=0.004)
        options={'camera_required':True,'camera_feedback':camera,'camera_goals':camera_goals(),'camera_config':cc}
        lib=__import__('atarm_controller.arm_controller_lib',fromlist=['*'])
        if hasattr(lib,'CoupledBendServo'): options['bend_servo_config']=cfg
        rig.construct(control_options=options,deploy_timeout_s=0.4);rig.home()
        assert rig.call(True)['success']
        before_deploy=rig.physical_mm();goal=CameraGoal('sim_base_fixed','sim_tip_camera','bend-map-1',(0.012,-0.008,0.05))
        if mode=='off_plane': goal=replace(goal,position_m=(0.012,-0.008,0.06))
        camera.mode=mode
        camera.motor_fault=motor
        began=time.monotonic()
        if hasattr(rig.arm,'move_bend_camera'):
            if use_node:
                module=__import__('atarm_controller.arm_controller_node',fromlist=['*'])
                node=object.__new__(module.UnitreeTensegrityControllerNode)
                node.arm=rig.arm;node.bend_goal_template=goal
                node.get_logger=lambda:types.SimpleNamespace(info=lambda s:None,warn=lambda s:None)
                node._bend_camera_position_callback(types.SimpleNamespace(data=list(goal.position_m)))
                response=rig.arm.bend_status()['result']
                published={}
                for attr in ('actuation_lengths_curr_pub','motor_angles_curr_pub','actuation_lengths_trgt_pub','controller_state_pub'):
                    setattr(node,attr,types.SimpleNamespace(publish=lambda msg,a=attr:published.__setitem__(a,msg.data)))
                node._publish_status_callback()
                import json
                assert json.loads(published['controller_state_pub'])['bend_visual']['enabled']
            else: response=rig.arm.move_bend_camera(goal)
        else:
            # Pre-integration contrast: nominal cable arrival through the existing step() path.
            rig.arm.final_target_actuation_lengths[:3]=mv(B,goal.position_m[:2])
            arrived=False
            for _ in range(100):
                arrived=rig.arm.step()
                if arrived: break
                time.sleep(0.002)
            response={'success':bool(arrived),'message':'Legacy nominal cable completion'}
        elapsed=time.monotonic()-began
        status=getattr(rig.arm,'bend_status',lambda:{'enabled':False})()
        detail={'response':response,'elapsed_s':elapsed,'plant':plant,'noise_per_axis_m':noise,'seed':seed,
            'state':rig.arm.deploy_state,'goal':asdict(goal),'true_p':camera.truth(),
            'true_error_m':length(sub(camera.truth(),goal.position_m)),
            'physical_q':camera.physical_q(),'deploy_before_mm':before_deploy,'deploy_after_mm':rig.physical_mm(),
            'bend':status,'camera_samples':camera.logs,'motor_status':[m.get_current_status() for m in rig.arm.motors]}
        base.save_rig(name,rig,detail)
        assert response['success']==expected,detail
        if expected:
            assert detail['true_error_m']<=0.001+0.00001,detail
            assert status['enabled'] and status['state']=='SUCCEEDED',detail
            assert abs(rig.physical_mm()-before_deploy)<0.05,detail
            assert all(abs(v)>0.00005 for v in camera.physical_q()),detail
            assert all(max(abs(v) for v in step['delta_q'])<=cfg.max_step_m+1e-12 for step in status['history']),detail
            assert all(abs(sum(step['delta_q']))<1e-10 for step in status['history']),detail
            if adaptive:
                assert status['model']['accepted']>0,detail
            positions=[camera.physical_q()]
            repeat_steps=[]
            for _ in range(repeat):
                assert rig.arm.move_bend_camera(goal)['success']
                positions.append(camera.physical_q());repeat_steps.append(rig.arm.bend_status()['steps'])
            detail['repeat_peak_to_peak_m']=max(max(q[i] for q in positions)-min(q[i] for q in positions) for i in range(3))
            detail['repeat_steps']=repeat_steps
            assert detail['repeat_peak_to_peak_m']<=0.001,detail
            if waypoints:
                replies=[]
                for p in ((-0.004,0.006,0.05),(0.006,0.009,0.05),(0.012,-0.008,0.05)):
                    reply=rig.arm.move_bend_camera(replace(goal,position_m=p));replies.append(reply)
                    assert reply['success'],(detail,replies)
                detail['waypoints']=replies;detail['waypoint_model']=rig.arm.bend_status()['model']
                assert detail['waypoint_model']['excitation_rank']==2
            if not noise:
                mark=(len(rig.active.logs),len(rig.deploy.logs))
                assert not rig.arm.update_bend(0.1,0.2)
                assert mark==(len(rig.active.logs),len(rig.deploy.logs))
            if mode=='ownership': assert camera.ownership_checked
        else:
            assert expected_reason is not None,'Failure acceptance requires a specific expected reason'
            codes=(expected_reason,) if isinstance(expected_reason,str) else expected_reason
            assert any(code in response['message'] for code in codes),detail
            assert status['state']=='FAULT' and rig.arm.deploy_state=='FAULT',detail
            assert elapsed<cfg.timeout_s+0.6,detail
            assert status['stop_requested'] and len(status['stop_receipts'])==4,detail
            mark=(len(rig.active.logs),len(rig.deploy.logs))
            assert not rig.arm.move_bend_camera(goal)['success'];rig.arm.step()
            assert mark==(len(rig.active.logs),len(rig.deploy.logs)),detail
            detail['no_commands_after_fault']=True
            if motor not in ('bad_crc','timeout'):
                assert all(m.cmd.kp==m.cmd.kd==m.cmd.dq==m.cmd.tau==0 for m in rig.arm.motors),detail
            for receipt in status['stop_receipts']:
                if receipt['reply_valid']:
                    board=rig.deploy if receipt['actuator_index']==3 else rig.active
                    wire=next(row for row in reversed(board.logs) if row['motor_id']==receipt['motor_id'])
                    raw=bytes.fromhex(wire['request_hex'])
                    assert all(struct.unpack_from('<h',raw,off)[0]==0 for off in (12,14,20,22)),detail
        base.save_rig(name,rig,detail)
        return detail
    finally: rig.close()


def paired(base,name,plant):
    fixed=exercise(base,name+'_fixed',plant=plant,adaptive=False)
    online=exercise(base,name+'_online',plant=plant)
    base.DETAILS[name]={'fixed_steps':fixed['bend']['steps'],'online_steps':online['bend']['steps'],
        'fixed_error_m':fixed['true_error_m'],'online_error_m':online['true_error_m']}


def config_guard(base,name,kind):
    changes={'cable_mix':((1,0),(2,0),(-3,0))} if kind=='mix' else {'initial_jacobian':((1,1),(0,0),(0,0))}
    try: bend_config(**changes)
    except ValueError: base.DETAILS[name]={'rejected_before_device_access':True}
    else: raise AssertionError('Singular configuration must be rejected')


def cases(base):
    result=[]
    def add(name,**kwargs): result.append((name,lambda n=name,k=kwargs:exercise(base,n,**k)))
    add('bend_coupled',repeat=2)
    add('bend_nonlinear',plant='nonlinear')
    add('bend_changing',plant='changing')
    add('bend_waypoints',waypoints=True)
    add('bend_node_callback',use_node=True)
    add('bend_command_ownership',mode='ownership')
    for seed in (7,19,41): add('bend_noise_'+str(seed),noise=0.00025,seed=seed,repeat=3)
    for plant in ('jammed','singular'):
        add('bend_'+plant,plant=plant,expected=False,expected_reason=('BEND_CAMERA_NO_PROGRESS','BEND_TRAVEL_LIMIT'))
    for mode in ('lost','lost_late','stale','epoch','replay','deploy_drift','off_plane'):
        reason={'lost':'CAMERA_TRACKING_LOST','lost_late':'CAMERA_TRACKING_LOST','stale':'CAMERA_FEEDBACK_STALE',
            'epoch':'CAMERA_REFERENCE_MISMATCH','replay':'CAMERA_FRAME_REPLAY_OR_MUTATION',
            'deploy_drift':'BEND_DEPLOY_ACTUATOR_MOVED','off_plane':'BEND_UNCONTROLLABLE_TRANSLATION'}[mode]
        add('bend_'+mode,mode=mode,expected=False,expected_reason=reason)
    for fault in ('stall','coherent_false','bad_crc','timeout'):
        reason={'stall':('BEND_COMMON_MODE_CHANGED','BEND_MOTOR_PROGRESS_STALLED'),
            'coherent_false':('BEND_TRAVEL_LIMIT','BEND_CAMERA_NO_PROGRESS','BEND_ITERATION_LIMIT'),
            'bad_crc':'BAD_CRC','timeout':'TIMEOUT'}[fault]
        add('bend_one_motor_'+fault,motor=fault,expected=False,expected_reason=reason)
    add('bend_travel_limit',changes={'max_lengths_m':(0.003,0.04,0.04)},expected=False,expected_reason='BEND_TRAVEL_LIMIT')
    add('bend_offset_limit',changes={'max_offset_m':0.003},expected=False,expected_reason='BEND_OFFSET_LIMIT')
    add('bend_total_limit',changes={'max_total_travel_m':0.004},expected=False,expected_reason='BEND_TOTAL_TRAVEL_LIMIT')
    add('bend_iteration_limit',changes={'max_steps':1},expected=False,expected_reason='BEND_ITERATION_LIMIT')
    add('bend_time_limit',changes={'timeout_s':0.04},expected=False,expected_reason='BEND_OPERATION_TIMEOUT')
    for plant in ('coupled','changing'):
        name='bend_paired_'+plant;result.append((name,lambda n=name,p=plant:paired(base,n,p)))
    for kind in ('mix','model'):
        name='bend_bad_'+kind;result.append((name,lambda n=name,k=kind:config_guard(base,n,k)))
    return result
