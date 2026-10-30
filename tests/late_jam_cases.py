"""Position-triggered late jams in the fake boards, after a successful probe."""
from copy import deepcopy
import math,struct,time
from recovery_cases import setup,external_home,counts
from probe_cases import config as probe_config
from atarm_controller.camera_recovery import RecoveryConfig
from atarm_controller.coupled_bend import inverse_action,mv
from bend_cases import B,H

RAW_TO_METRES=6.2832/16384/9.1*0.026


class PositionJam:
    def __init__(self,rig,phase,fraction,false_feedback=False,single=None):
        self.rig=rig;self.phase=phase;self.fraction=fraction
        self.false_feedback=false_feedback;self.single=single;self.event=None
    def xy(self,positions):
        q=tuple(sign*positions[mid]*RAW_TO_METRES for mid,sign in zip((1,0,2),(1,1,-1)))
        return inverse_action(B,mv(H,q))
    def before_command(self,board,motor_id,target):
        arm=self.rig.arm
        if self.event or not arm.cycle.active or arm.cycle.phase!=self.phase: return
        if self.phase=='DEPLOY':
            if board is not self.rig.deploy or motor_id!=2: return
            trigger=self.fraction*0.05
            old=-board.position[2]*RAW_TO_METRES;new=-target*RAW_TO_METRES
            if new<trigger: return
            board.position[2]=round(-trigger/RAW_TO_METRES)
            board.mode='coherent_false' if self.false_feedback else 'stall'
            actual=-board.position[2]*RAW_TO_METRES
            physical={'deployment_m':actual,'bend_q':tuple(0.0 for _ in range(3))}
        else:
            if board is not self.rig.active: return
            if self.single is not None and motor_id!=(1,0,2)[self.single]: return
            trigger=self.fraction*0.01
            old_xy=self.xy(board.position);proposed=dict(board.position);proposed[motor_id]=target
            new_xy=self.xy(proposed)
            project=lambda p:0.8*p[0]-0.6*p[1]
            old=project(old_xy);new=project(new_xy)
            if new<trigger: return
            alpha=0.0 if old>=trigger else (trigger-old)/max(new-old,1e-12)
            current=board.position[motor_id]
            board.position[motor_id]=round(current+max(0.0,min(1.0,alpha))*(target-current))
            affected=(motor_id,) if self.single is not None else (1,0,2)
            for mid in affected: board.motor_modes[mid]='coherent_false' if self.false_feedback else 'stall'
            actual=project(self.xy(board.position))
            physical={'deployment_m':self.rig.physical_mm()/1000,
                'bend_q':tuple(sign*board.position[mid]*RAW_TO_METRES for mid,sign in zip((1,0,2),(1,1,-1)))}
        self.event={'injected_s':time.monotonic(),'phase':self.phase,'requested_fraction':self.fraction,
            'trigger_progress_m':trigger,'actual_progress_m':actual,'false_feedback':self.false_feedback,
            'single_bend_index':self.single,'physical':physical,
            'wire_counts_before_trigger_response':counts(self.rig)}


def verify_stop(rig,status,injected_s):
    receipts=status['stop_receipts'];assert len(receipts)==4
    assert sorted(s['actuator_index'] for s in receipts)==[0,1,2,3]
    times=[]
    for receipt in receipts:
        assert receipt['reply_valid'],receipt
        index=receipt['actuator_index'];board=rig.deploy if index==3 else rig.active
        tx=[r for r in board.adapter.trace if (r.get('tx_ns') or 0)>0]
        wires=board.logs[-len(tx):]
        records=[(r,w) for r,w in zip(tx,wires) if r['frame_id']==receipt['transport_frame_id']]
        assert len(records)==1
        r,w=records[0];raw=bytes.fromhex(w['request_hex'])
        assert struct.unpack_from('<hh',raw,20)==(0,0)
        assert struct.unpack_from('<hh',raw,12)==(0,0)
        assert r['tx_ns']/1e9>=injected_s
        times.append((r['tx_ns']/1e9,r['rx_ns']/1e9))
    last_stop=max(t[0] for t in times)
    # Once all four cancellation requests are issued, no position loop resumes.
    for board in (rig.active,rig.deploy):
        tx=[r for r in board.adapter.trace if (r.get('tx_ns') or 0)>0]
        for r,w in zip(tx,board.logs[-len(tx):]):
            if r['tx_ns']/1e9>last_stop:
                assert struct.unpack_from('<hh',bytes.fromhex(w['request_hex']),20)==(0,0)
    return {'first_zero_effort_request_ms':(min(t[0] for t in times)-injected_s)*1000,
        'last_zero_effort_reply_ms':(max(t[1] for t in times)-injected_s)*1000}


def exercise(base,name,phase='DEPLOY',fraction=0.5,false_feedback=False,single=None,noise=0.0,repair=False,healthy=False):
    options={'probe_config':probe_config(),
        'recovery_config':RecoveryConfig(timeout_s=2.0,period_s=0.002,settle_s=0.025,max_pose_age_s=0.07)}
    rig,camera,plan,ignored=setup(base,extra_options=options)
    try:
        camera.fault_phase=None;camera.fault=None;camera.last=None;camera.noise=noise
        rig.arm._fault('SIMULATED_EXTERNAL_FAULT_BEFORE_RECOVERY')
        recovered=rig.arm.recover_camera_cycle(mechanically_confirmed=True,fault_cleared=True)
        probe=rig.arm.probe_status();recovery=rig.arm.recovery_status()
        assert recovered['success'] and not recovered['complete'],recovered
        assert probe['state']=='SUCCEEDED' and all(s['verified'] for s in probe['stages'])
        assert recovery['history'][-1]['proof']['frame_id']>probe['stages'][-1]['frame_id']
        injector=PositionJam(rig,phase,fraction,false_feedback,single)
        if not healthy: rig.active.late_jam=rig.deploy.late_jam=injector
        result=rig.arm.run_camera_cycle(plan);returned=time.monotonic();cycle=rig.arm.cycle_status()
        truth=camera.truth()
        goal=(rig.arm.camera_goals[False] if healthy else plan.neutral_goal if phase=='DEPLOY' else plan.bend_goals[0])
        error=math.sqrt(sum((a-b)**2 for a,b in zip(truth,goal.position_m)))
        detail={'recovery_result':recovered,'recovery_before_cycle':recovery,'passed_probe':probe,
            'result':result,'cycle':cycle,'root_state':rig.arm.deploy_state,'injection':injector.event,
            'true_camera_p':truth,'true_error_m':error,'physical_q':camera.physical_q(),
            'physical_deploy_mm':rig.physical_mm(),'noise_per_axis_m':noise}
        base.save_rig(name,rig,detail)
        if healthy:
            assert result['success'] and result['complete'] and cycle['state']=='SUCCEEDED'
            assert math.sqrt(sum(v*v for v in truth))<=0.001
        else:
            event=injector.event;assert event is not None,'Required physical threshold was never reached'
            assert event['actual_progress_m']>=event['trigger_progress_m']-0.000002
            assert event['injected_s']>recovery['history'][-1]['ended_s']
            assert not result['success'] and not result['complete'] and cycle['state']=='FAULT'
            assert rig.arm.deploy_state=='FAULT' and cycle['stop_requested']
            if phase=='DEPLOY':
                assert any(code in result['message'] for code in ('MOTION_PROGRESS_STALLED','UNCOMMANDED_DISPLACEMENT',
                    'VISUAL_SERVO_NO_PROGRESS','VISUAL_SERVO_OPERATION_TIMEOUT','CAMERA_PROGRESS_STALLED'))
            else:
                assert any(code in result['message'] for code in ('BEND_MOTOR_PROGRESS_STALLED','BEND_COMMON_MODE_CHANGED',
                    'BEND_CAMERA_NO_PROGRESS','BEND_OPERATION_TIMEOUT','UNCOMMANDED_DISPLACEMENT','CAMERA_PROGRESS_STALLED'))
            assert error>0.0015
            phases=[s['phase'] for s in cycle['stages']]
            assert phases[-1]==phase and not cycle['stages'][-1]['verified']
            assert 'RETRACT' not in phases and 'VERIFY_END' not in phases
            if phase=='DEPLOY': assert 'BEND_1' not in phases
            detail['stop_timing']=verify_stop(rig,cycle,event['injected_s'])
            detail['fault_to_return_ms']=(returned-event['injected_s'])*1000
            assert detail['fault_to_return_ms']<1200,detail['fault_to_return_ms']
            mark=counts(rig)
            assert not rig.arm.run_camera_cycle(plan)['success'];assert not rig.arm.set_deploy(False)['success']
            rig.arm.step();assert mark==counts(rig)
            assert rig.arm.cycle_status()==cycle
            detail['subsequent_requests_blocked_without_commands']=True
            if repair:
                rig.active.late_jam=rig.deploy.late_jam=None
                rig.active.motor_modes.clear();rig.deploy.mode='normal';external_home(rig,camera)
                again=rig.arm.recover_camera_cycle(mechanically_confirmed=True,fault_cleared=True)
                assert again['success'] and not again['complete'],again
                retried=rig.arm.run_camera_cycle(plan)
                assert retried['success'] and retried['complete'],retried
                detail['after_repair']={'recovery':again,'probe':rig.arm.probe_status(),'retry':retried,'cycle':rig.arm.cycle_status()}
        base.save_rig(name,rig,detail)
    finally: rig.close()


def cases(base):
    result=[]
    def add(name,**kwargs): result.append((name,lambda n=name,k=kwargs:exercise(base,n,**k)))
    for phase,label in (('DEPLOY','deploy'),('BEND_1','bend')):
        for fraction in (0.25,0.5,0.75):
            for fake in (False,True):
                add('late_'+label+'_'+str(int(fraction*100))+('_false' if fake else '_actual'),
                    phase=phase,fraction=fraction,false_feedback=fake)
    for i in range(3): add('late_single_bend_'+str(i),phase='BEND_1',single=i)
    for phase,label in (('DEPLOY','deploy'),('BEND_1','bend')):
        add('late_'+label+'_noise',phase=phase,false_feedback=True,noise=0.00002)
        add('late_'+label+'_repair',phase=phase,repair=True)
    add('late_healthy_control',healthy=True)
    return result
