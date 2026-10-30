"""Real rclpy/DDS, independent pose publisher process and native PTY motor boards."""
import argparse,faulthandler,hashlib,importlib.util,json,math,multiprocessing as mp,os,platform,queue,struct,subprocess,sys,threading,time,traceback
from pathlib import Path
PROJECT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(PROJECT),str(PROJECT/'tests')]
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from test_simulated_serial import Rig
from bend_cases import B,H
from atarm_controller.coupled_bend import inverse_action,mv
from atarm_controller.ros_control_config import load_control_config
from atarm_controller.ros_pose_bridge import RosPoseBridge
RAW=6.2832/16384/9.1*.026

def publisher_process(samples,mode,ready,stop):
    rclpy.init();node=Node('independent_simulated_pose_source')
    pp=node.create_publisher(PoseStamped,'atarm/camera/pose',4)
    sp=node.create_publisher(String,'atarm/camera/source_status',4)
    last=None;count=0
    def tick():
        nonlocal last,count
        sample=None
        try:
            while True:sample=samples.get_nowait()
        except queue.Empty:pass
        if sample is None:return
        code=mode.value
        if code==1 and last is not None:sample=last
        else:last=sample
        stamp,p,frame_id=sample
        pose=PoseStamped();pose.header.stamp.sec=stamp//1000000000;pose.header.stamp.nanosec=stamp%1000000000
        pose.header.frame_id='wrong_frame' if code==4 else 'sim_base_fixed'
        pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=p
        pose.pose.orientation.w=1.
        meta=String();meta.data=json.dumps({'stamp_ns':stamp,'frame_id':frame_id,'tracking_ok':code!=2,
            'map_epoch':'reset-map' if code==3 else 'bend-map-1','metric':True,'child_frame':'sim_tip_camera'})
        pp.publish(pose);sp.publish(meta);count+=1
        if count>10:ready.set()
    node.create_timer(.01,tick)
    try:
        while rclpy.ok() and not stop.is_set():rclpy.spin_once(node,timeout_sec=.05)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

def wait(condition,timeout=8.,label='condition'):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        value=condition()
        if value:return value
        time.sleep(.002)
    raise AssertionError('Timeout waiting for '+label)

def matched_stops(receipts,calls):
    assert len(receipts)==4
    selected=[]
    for receipt in receipts:
        side='deploy' if receipt['actuator_index']==3 else 'active'
        records=[x for x in calls if x['side']==side and x['frame_id']==receipt['transport_frame_id']]
        assert len(records)==1 and receipt['reply_valid']
        record=records[0];assert record['kp']==record['kd']==record['tau']==record['dq']==0
        selected.append(record)
    return selected

def run_case(name,out,legacy=False):
    faulthandler.dump_traceback_later(12,repeat=True)
    ctx=mp.get_context('spawn');samples=ctx.Queue(8);mode=ctx.Value('i',0);ready=ctx.Event();source_stop=ctx.Event()
    rig=Rig();world_stop=threading.Event();calls=[];world=[];statuses=[];executor=None;thread=None;node=None;client=None
    source=ctx.Process(target=publisher_process,args=(samples,mode,ready,source_stop));source.start()
    for board in (rig.active,rig.deploy):
        exchange=board.adapter.exchange
        def wrapped(cmd,fn=exchange,side='active' if board is rig.active else 'deploy'):
            e={'side':side,'id':cmd.id,'kp':cmd.kp,'kd':cmd.kd,'tau':cmd.tau,'dq':cmd.dq,
                'start_ns':time.monotonic_ns(),'thread':threading.current_thread().name}
            result=fn(cmd);e.update({'end_ns':time.monotonic_ns(),'frame_id':result.transport_frame_id,'valid':result.valid});calls.append(e);return result
        board.adapter.exchange=wrapped
    def physical():
        q=tuple(sign*rig.active.position[mid]*RAW for mid,sign in zip((1,0,2),(1,1,-1)))
        xy=inverse_action(B,mv(H,q));return (xy[0],xy[1],rig.physical_mm()/1000)
    def produce():
        index=0
        while not world_stop.wait(.01):
            index+=1;stamp=time.time_ns();p=physical();world.append({'s':time.monotonic(),'stamp_ns':stamp,'position_m':p,'frame_id':index})
            try:samples.put_nowait((stamp,p,index))
            except queue.Full:pass
    world_thread=threading.Thread(target=produce,daemon=True);world_thread.start()
    init_args=['--ros-args','-p','deploy_motor.direction:=-1','-p','deploy_motor_torque_gain_scale:=1.0',
        '-p','port_active:=sim://active','-p','port_deploy:=sim://deploy','-p','step_rate_hz:=100.0','-p','publish_rate_hz:=40.0',
        '-p','deploy_speed_m_s:=0.03','-p','deploy_timeout_s:=5.0','-p','deploy_settle_s:=0.05',
        '-p','feedback_max_age_s:=0.3','-p','feedback_max_speed_m_s:=4.0','-p','feedback_step_margin_m:=0.004',
        '-p','progress_timeout_s:=0.3','-p','control_config_path:='+str(PROJECT/'config/ros_simulation_only.json')]
    rclpy.init(args=init_args)
    try:
        if legacy:
            spec=importlib.util.spec_from_file_location('atarm_controller.original_node',PROJECT/'tests/original_node_snapshot.py')
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            options,reference=load_control_config(PROJECT/'config/ros_simulation_only.json')
            bridge=RosPoseBridge(**reference,max_age_s=.15)
            node=module.UnitreeTensegrityControllerNode(serial_factory=rig.factory,camera_feedback=bridge,**options)
            node.pose_bridge=bridge;node.step_timer.cancel();node.publish_timer.cancel()
            def receive_pose(m):
                try:bridge.pose(m,node.get_clock().now().nanoseconds)
                except ValueError:pass
            def receive_meta(m):
                try:bridge.status(m,node.get_clock().now().nanoseconds)
                except ValueError:pass
            node.create_subscription(PoseStamped,'atarm/camera/pose',receive_pose,4)
            node.create_subscription(String,'atarm/camera/source_status',receive_meta,4)
        else:
            from atarm_controller.arm_controller_node import UnitreeTensegrityControllerNode
            node=UnitreeTensegrityControllerNode(serial_factory=rig.factory)
        rig.arm=node.arm
        client=Node('integration_test_client')
        client.create_subscription(String,'atarm/arm_status/controller_state',lambda m:statuses.append({'s':time.monotonic(),'value':json.loads(m.data)}),40)
        executor=SingleThreadedExecutor();executor.add_node(node);executor.add_node(client)
        errors=[]
        def spin():
            try:executor.spin()
            except Exception:errors.append(traceback.format_exc())
        thread=threading.Thread(target=spin,name='single-ROS-executor',daemon=True);thread.start()
        assert ready.wait(5.);wait(lambda:node.pose_bridge.diagnostics()['accepted']>=5,label='real DDS pose receipt')
        def service(service_name):
            cl=client.create_client(Trigger,service_name);assert cl.wait_for_service(timeout_sec=2.)
            sent=time.monotonic();future=cl.call_async(Trigger.Request());wait(future.done,timeout=8.,label=service_name)
            reply=future.result();client.destroy_client(cl)
            try:body=json.loads(reply.message)
            except Exception:body={'success':reply.success,'message':reply.message}
            return {'sent_s':sent,'received_s':time.monotonic(),'success':reply.success,'body':body}
        def latest_job():return statuses[-1]['value'].get('ros_job',{}) if statuses else {}
        def finish(job_id,timeout=10.):
            return wait(lambda:latest_job() if latest_job().get('job_id')==job_id and latest_job().get('state') in ('SUCCEEDED','FAILED','CANCELLED','WORKER_FAILED') else None,timeout,label='job completion')
        if legacy:
            recovered=node.arm.recover_camera_cycle(mechanically_confirmed=True,fault_cleared=True);assert recovered['success'],recovered
            # The unmodified service blocks its real SingleThreadedExecutor.
            cl=client.create_client(Trigger,'run_camera_cycle');assert cl.wait_for_service(timeout_sec=2.)
            began=time.monotonic();future=cl.call_async(Trigger.Request())
            wait(lambda:node.arm.cycle.phase=='DEPLOY',2.,'legacy deploy phase')
            count=node.pose_bridge.accepted;time.sleep(.03)
            cancel=service('cancel_camera_cycle');wait(future.done)
            result={'legacy':True,'run_reply_success':future.result().success,'cancel':cancel,
                'accepted_during_block':node.pose_bridge.accepted-count,'cycle':node.arm.cycle_status(),
                'source_frames':len([x for x in world if x['s']>=began]),'callback_history':node.pose_bridge.history}
            assert not future.result().success and not cancel['success'],result
            assert cancel['received_s']-cancel['sent_s']>.08,result
        else:
            recovery=service('recover_after_external_home_confirmation');assert recovery['success'],recovery
            recovered=finish(recovery['body']['job_id']);assert recovered['state']=='SUCCEEDED',recovered
            start=service('run_camera_cycle');assert start['success'] and start['body']['complete'] is False
            job_id=start['body']['job_id'];cancel=None;duplicate=None;injected=None
            if name!='complete':
                target='BEND_1' if name=='cancel_bend' else 'RETRACT' if name=='cancel_retract' else 'DEPLOY'
                def at_phase():
                    if not statuses:return False
                    s=statuses[-1]['value'];return s.get('camera_cycle',{}).get('phase')==target and s.get('ros_job',{}).get('state')=='RUNNING'
                wait(at_phase,timeout=10.,label=target)
                if target=='DEPLOY':wait(lambda:physical()[2]>.006,2.,'visible deploy motion')
                if name.startswith('cancel_'):
                    duplicate=service('run_camera_cycle');assert not duplicate['success']
                    cancel=service('cancel_camera_cycle');assert cancel['success'],cancel
                else:
                    injected=time.monotonic();mode.value={'freeze':1,'tracking_lost':2,'map_reset':3,'wrong_frame':4}[name]
            job=finish(job_id,timeout=15.)
            hist=node.pose_bridge.history
            moving=[x for x in hist if start['sent_s']<=x['received_s']<=job['ended_s']]
            gaps=[b['received_s']-a['received_s'] for a,b in zip(moving,moving[1:])]
            result={'legacy':False,'start':start,'recovery':recovery,'recovered':recovered,'job':job,'cancel':cancel,'duplicate':duplicate,
                'injected_s':injected,'pose_frames_during_motion':len(moving),'maximum_receive_gap_s':max(gaps,default=0),
                'callback_history':list(hist),'world':list(world),'bridge':node.pose_bridge.diagnostics(),
                'last_status':statuses[-1] if statuses else None,'physical_end_m':physical()}
            if name=='complete':
                assert job['state']=='SUCCEEDED' and job['complete'] and len(moving)>60,result
                assert max(gaps)<.12 and math.sqrt(sum(v*v for v in physical()))<.001
            elif cancel:
                assert job['state']=='CANCELLED' and not job['complete'],result
                zero=matched_stops(job['stop_receipts'],calls)
                assert {('deploy',2),('active',1),('active',0),('active',2)}<={(x['side'],x['id']) for x in zero}
                result['cancel_to_first_zero_ms']=(min(x['start_ns'] for x in zero)/1e9-cancel['sent_s'])*1000
                result['cancel_to_all_zero_reply_ms']=(max(x['end_ns'] for x in zero)/1e9-cancel['sent_s'])*1000
                assert result['cancel_to_first_zero_ms']<100 and result['cancel_to_all_zero_reply_ms']<200,result
                assert len(job['stop_receipts'])==4 and all(x['reply_valid'] for x in job['stop_receipts'])
                mark=len([x for x in calls if x['kp']]);restart=service('run_camera_cycle');retry=finish(restart['body']['job_id'])
                result['retry_after_cancel']=retry;assert not retry['success'] and mark==len([x for x in calls if x['kp']])
                assert len(moving)>5 and max(gaps)<.12
            else:
                assert job['state']=='FAILED' and not job['complete'],result
                result['fault_stop_receipts']=list(node.arm.cycle.stop_receipts)
                zero=matched_stops(result['fault_stop_receipts'],calls)
                result['fault_to_zero_ms']=(min(x['start_ns'] for x in zero)/1e9-injected)*1000
                assert result['fault_to_zero_ms']<400,result
            writers={x['thread'] for x in calls if x['start_ns']/1e9>=start['sent_s']}
            assert writers=={'atarm-single-motor-owner'},writers
        assert not errors,errors
        result['calls']=list(calls);result['active_wire']=list(rig.active.logs);result['deploy_wire']=list(rig.deploy.logs)
        result['active_receive']=list(rig.active.adapter.trace);result['deploy_receive']=list(rig.deploy.adapter.trace)
        result['runtime']={'rclpy_file':rclpy.__file__,'ros_distro':os.environ.get('ROS_DISTRO'),'executor':'SingleThreadedExecutor',
            'pose_publisher_pid':source.pid,'controller_pid':os.getpid(),'platform':platform.platform(),'physical_devices':False}
        (out/'case.json').write_text(json.dumps(result,indent=2)+'\n')
        return result
    except Exception:
        if node and hasattr(node,'publish_timer'):node.publish_timer.cancel()
        partial={'calls':list(calls),'statuses':list(statuses)[-12:],'world':list(world),'pose_source_exitcode':source.exitcode}
        if node:
            partial['bridge']=node.pose_bridge.diagnostics();partial['callback_history']=node.pose_bridge.history
            if hasattr(node,'motion_worker'):partial['worker']=node.motion_worker.snapshot()
        (out/'partial.json').write_text(json.dumps(partial,indent=2)+'\n')
        raise
    finally:
        faulthandler.cancel_dump_traceback_later()
        world_stop.set();world_thread.join(2.)
        if executor:executor.shutdown(timeout_sec=2.)
        if thread:thread.join(3.)
        if node:node.destroy_node()
        if client:client.destroy_node()
        rclpy.shutdown()
        source_stop.set();source.join(3.)
        if source.is_alive():source.terminate();source.join(3.)
        samples.cancel_join_thread();samples.close()
        rig.active.close();rig.deploy.close()

def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--case');p.add_argument('--legacy',action='store_true');args=p.parse_args()
    args.out.mkdir(parents=True,exist_ok=False)
    if args.case:
        try:
            d=run_case(args.case,args.out,args.legacy)
            print(json.dumps({'case':args.case,'passed':True,'cancel_to_first_zero_ms':d.get('cancel_to_first_zero_ms'),
                'cancel_to_all_zero_reply_ms':d.get('cancel_to_all_zero_reply_ms'),'pose_frames':d.get('pose_frames_during_motion')}));return 0
        except Exception:
            (args.out/'error.txt').write_text(traceback.format_exc());raise
    cases=['complete','cancel_deploy','cancel_bend','cancel_retract','freeze','tracking_lost','map_reset','wrong_frame']
    rows=[]
    for name in cases:
        cmd=[sys.executable,'-B',str(Path(__file__)),'--out',str(args.out/name),'--case',name]
        r=subprocess.run(cmd,capture_output=True,text=True,timeout=45)
        (args.out/(name+'.log')).write_text(r.stdout+r.stderr)
        rows.append({'case':name,'passed':r.returncode==0,'output_tail':(r.stdout+r.stderr)[-1600:]})
    report={'cases':rows,'passed':sum(x['passed'] for x in rows),'count':len(rows),
        'source_sha256':{str(p.relative_to(PROJECT)):hashlib.sha256(p.read_bytes()).hexdigest()
            for folder in ('atarm_controller','tests','config') for p in (PROJECT/folder).glob('*') if p.is_file()}}
    (args.out/'results.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report['cases'],indent=2))
    return 0 if report['passed']==report['count'] else 1
if __name__=='__main__':raise SystemExit(main())
