"""DDS external vision + native A1 protocol + two independent PTY boards.

The controller runs in its own process, with no simulated plant/truth imports.
Wire emulators and camera truth remain exclusively in test processes.
"""
import argparse,json,math,multiprocessing as mp,os,signal,subprocess,sys,time,traceback
from pathlib import Path
PROJECT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(PROJECT),str(PROJECT/'tests')]
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from pty_cartesian_world import WireBoard,true_tcp,pose_process,crc


def spin(node,condition,seconds=10.):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.01)
        value=condition()
        if value:return value
    raise AssertionError('Timed out waiting for DDS condition')


def case(name,out):
    namespace='/fd_'+name;ctx=mp.get_context('spawn');q=ctx.Array('d',[0.,0.,0.,.04]);mode=ctx.Value('i',0);stop=ctx.Event()
    active=WireBoard('active',q);deploy=WireBoard('deploy',q);source=None;server=None;node=None;log=None
    states=[];acks=[];result={'case':name,'passed':False};shutdown_sent=False
    try:
        if name=='reference_mismatch':active.raw[1]=5000
        source=ctx.Process(target=pose_process,args=(q,mode,stop,namespace));source.start()
        config=json.loads((PROJECT/'config/offline_fd_validation.json').read_text())
        if name=='coordinated':config['controller']['strategy']='coordinated'
        (out/'session_config.json').write_text(json.dumps(config,indent=2))
        log=(out/'node.log').open('w')
        command=[sys.executable,'-B','-m','atarm_controller.cartesian_fd_node','--offline-pty',
            '--active-fd',str(active.slave),'--deploy-fd',str(deploy.slave),'--session-config',str(out/'session_config.json'),
            '--audit',str(out/'node_audit.json'),'--ros-args','-r','__ns:='+namespace]
        server=subprocess.Popen(command,pass_fds=(active.slave,deploy.slave),stdout=log,stderr=subprocess.STDOUT)
        rclpy.init();node=Node('fd_test_client',namespace=namespace)
        node.create_subscription(String,'atarm/cartesian/state',lambda m:states.append(json.loads(m.data)),100)
        node.create_subscription(String,'atarm/cartesian/ack',lambda m:acks.append(json.loads(m.data)),30)
        target=node.create_publisher(String,'atarm/cartesian/target',10)
        confirm=node.create_client(Trigger,'atarm/cartesian/confirm_reference');cancel=node.create_client(Trigger,'atarm/cartesian/cancel')
        spin(node,lambda:states and states[-1].get('observation') and states[-1]['vision']['accepted']>=8)
        def service(client):
            assert client.wait_for_service(timeout_sec=2.)
            began=time.monotonic_ns();future=client.call_async(Trigger.Request());spin(node,future.done)
            reply=future.result();return {'success':reply.success,'body':json.loads(reply.message),'sent_ns':began,'received_ns':time.monotonic_ns()}
        def send(goal):
            m=String();m.data=json.dumps(goal);target.publish(m)
            return spin(node,lambda:next((a for a in reversed(acks) if a['request_id']==goal['request_id']),None))
        goal={'request_id':'main','position_m':[.3124208,.03748,.1949592],
            'parent_frame':'control_base','child_frame':'tcp','map_epoch':'validation_map_0'}
        result['premature']=send(dict(goal,request_id='before_confirm'))
        assert result['premature']['reason']=='REFERENCE_NOT_CONFIRMED'
        assert all(x['kp_raw']==x['kd_raw']==0 for x in active.logs+deploy.logs)
        result['confirm']=service(confirm);assert result['confirm']['success']
        if name=='reference_mismatch':
            spin(node,lambda:states[-1]['job'].get('reason')=='ACTUATOR_MECHANICAL_REFERENCE_MISMATCH')
            assert not states[-1]['ready'] and all(x['kp_raw']==0 for x in active.logs+deploy.logs)
            result['final']=states[-1];result['passed']=True
        else:
            spin(node,lambda:states[-1]['ready'])
            result['accepted']=send(goal);assert result['accepted']['accepted'] and not result['accepted']['complete']
            spin(node,lambda:states[-1]['job']['state']=='RUNNING' or math.dist(list(q),[0.,0.,0.,.04])>.0001)
            if name=='complete':
                result['busy']=send(dict(goal,request_id='overlap'));assert result['busy']['reason']=='BUSY'
                # Duplicate ID is checked before BUSY.
                count=len(acks);target.publish(String(data=json.dumps(goal)));spin(node,lambda:len(acks)>count)
                result['duplicate']=acks[-1];assert result['duplicate']['reason']=='DUPLICATE_ID'
            if name=='cancel':
                result['cancel']=service(cancel);assert result['cancel']['success']
            elif name=='shutdown':
                server.send_signal(signal.SIGINT);shutdown_sent=True
            elif name in ('tracking_lost','frozen_pose','map_reset','wrong_frame','delayed_pose','nonmetric'):
                mode.value={'frozen_pose':1,'tracking_lost':2,'map_reset':3,'wrong_frame':4,'delayed_pose':5,'nonmetric':6}[name]
            elif name in ('bad_crc','wrong_id','timeout','motor_error','stall','false_encoder'):
                active.mode=name;deploy.mode=name
            if name!='shutdown':
                final=spin(node,lambda:states[-1] if states[-1]['job'].get('request_id')=='main' and
                    states[-1]['job']['state'] in ('SUCCEEDED','FAULT','CANCELLED') else None,seconds=25.)
                result['final']=final;job=final['job']
                expected='SUCCEEDED' if name in ('complete','coordinated','hold_fault') else 'CANCELLED' if name=='cancel' else 'FAULT'
                assert job['state']==expected,job
                assert job['complete'] is (expected=='SUCCEEDED')
                if expected=='SUCCEEDED':
                    result['true_error_m']=math.dist(true_tcp(list(q)),goal['position_m'])
                    assert result['true_error_m']<=.0005,result
                    if name=='complete':
                        result['invalid']=send(dict(goal,request_id='nan',position_m=[math.nan,0,0]));assert not result['invalid']['accepted']
                        result['wrong_map']=send(dict(goal,request_id='wrong_map',map_epoch='old'));assert result['wrong_map']['reason']=='REFERENCE_MISMATCH'
                    if name=='hold_fault':
                        mode.value=2;spin(node,lambda:states[-1]['fault']);assert states[-1]['job']['state']=='FAULT';result['hold_fault']=states[-1]
                else:
                    assert len(job['stop_receipts'])==4
                    if name in ('cancel','tracking_lost','frozen_pose','map_reset','wrong_frame','delayed_pose','nonmetric','stall','false_encoder'):
                        assert all(r['reply_valid'] for r in job['stop_receipts']),job
                    else:assert not all(r['reply_valid'] for r in job['stop_receipts']),job
                    result['retry']=send(dict(goal,request_id='retry'));assert result['retry']['reason']=='EXPLICIT_RESTART_REQUIRED'
                assert len([s for s in states if s['job']['state'] in ('RUNNING','ACCEPTED')])>=1
            result['passed']=True
    except Exception:
        result['error']=traceback.format_exc()
    finally:
        if server:
            if server.poll() is None and not shutdown_sent:server.send_signal(signal.SIGINT)
            try:server.wait(timeout=8.)
            except subprocess.TimeoutExpired:server.kill();server.wait();result['passed']=False;result['shutdown_timeout']=True
            result['server_returncode']=server.returncode
            if server.returncode!=0:result['passed']=False
        if log:log.close()
        if node:node.destroy_node();rclpy.shutdown()
        stop.set()
        if source:
            source.join(4.)
            if source.is_alive():source.terminate();source.join();result['passed']=False
            result['source_returncode']=source.exitcode;result['source_pid']=source.pid
            if source.exitcode!=0:result['passed']=False
        result['physical_end_q']=list(q);result['controller_pid']=server.pid if server else None
        result['client_pid']=os.getpid();result['states_count']=len(states)
        audit_path=out/'node_audit.json'
        if audit_path.exists():
            audit=json.loads(audit_path.read_text());result['io_threads']=audit['io_threads']
            if audit['worker_alive'] or not audit['closed'] or not audit['stopped'] or audit['io_threads']!=['cartesian-fd-owner']:
                result['passed']=False;result['audit_failed']=True
            result['native_transactions']=len(audit['trace'])
            # Confirm stop receipts identify actual stop requests, not earlier
            # passive polls that also happen to contain zero effort.
            for completed in audit['jobs']:
                for receipt in completed['job'].get('stop_receipts',[]):
                    match=[t for t in audit['trace'] if t['channel']==receipt['actuator_index'] and t['frame_id']==receipt.get('transport_frame_id') and t['stop']]
                    if receipt['reply_valid'] and not match:result['passed']=False;result['missing_stop_transaction']=receipt
            if name=='cancel':
                stop_times=[r['tx_ns'] for r in result['final']['job']['stop_receipts'] if r.get('reply_valid')]
                result['cancel_to_first_stop_ms']=(min(stop_times)-result['cancel']['sent_ns'])/1e6
        else:result['passed']=False;result['missing_audit']=True
        for b in (active,deploy):
            (out/(b.bus+'_wire.json')).write_text(json.dumps(b.logs,indent=2));b.close()
        (out/'dds_states.json').write_text(json.dumps(states,indent=2));(out/'dds_acks.json').write_text(json.dumps(acks,indent=2))
        (out/'case.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--case',default='all');a=p.parse_args()
    a.out.mkdir(parents=True,exist_ok=False)
    cases=['complete','coordinated','cancel','tracking_lost','frozen_pose','map_reset','wrong_frame','delayed_pose','nonmetric',
           'bad_crc','wrong_id','timeout','motor_error','stall','false_encoder','reference_mismatch','hold_fault','shutdown'] if a.case=='all' else [a.case]
    rows=[]
    for name in cases:
        folder=a.out/name;folder.mkdir();r=case(name,folder)
        rows.append({k:r.get(k) for k in ('case','passed','true_error_m','cancel_to_first_stop_ms','native_transactions','states_count')})
        print(json.dumps(rows[-1]),flush=True)
    (a.out/'results.json').write_text(json.dumps({'passed':all(r['passed'] for r in rows),'cases':rows},indent=2)+'\n')
    return 0 if all(r['passed'] for r in rows) else 1


if __name__=='__main__':raise SystemExit(main())
