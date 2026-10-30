"""Isolate the original callback's blocking with a controlled 350-ms operation."""
import argparse,hashlib,importlib.util,json,sys,threading,time,types
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args();a.out.mkdir()
spec=importlib.util.spec_from_file_location('atarm_controller.original_node',ROOT/'tests/original_node_snapshot.py')
original=importlib.util.module_from_spec(spec);spec.loader.exec_module(original)
started=threading.Event();timestamps={};received=[];published=[];stop=threading.Event()
class ControlledArm:
    def run_camera_cycle(self,plan):
        timestamps['operation_start_s']=time.monotonic();started.set();time.sleep(.35)
        timestamps['operation_end_s']=time.monotonic();return {'success':False,'message':'Controlled blocking operation ended'}
    def cancel_camera_cycle(self):
        timestamps['cancel_callback_s']=time.monotonic();return {'success':True,'message':'Cancel callback finally executed'}
rclpy.init();controller=Node('original_callback_reproduction');client=Node('original_callback_client')
controller.arm=ControlledArm();controller.cycle_plan=None
controller.create_service(Trigger,'red/run',types.MethodType(original.UnitreeTensegrityControllerNode._cycle_service,controller))
controller.create_service(Trigger,'red/cancel',types.MethodType(original.UnitreeTensegrityControllerNode._cancel_cycle_service,controller))
controller.create_subscription(PoseStamped,'red/pose',lambda m:received.append(time.monotonic()),4)
pub=client.create_publisher(PoseStamped,'red/pose',4)
executor=SingleThreadedExecutor();executor.add_node(controller);executor.add_node(client)
thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
def produce():
    while not stop.wait(.01):
        m=PoseStamped();m.header.stamp=client.get_clock().now().to_msg();pub.publish(m);published.append(time.monotonic())
producer=threading.Thread(target=produce,daemon=True);producer.start()
run=client.create_client(Trigger,'red/run');cancel=client.create_client(Trigger,'red/cancel')
assert run.wait_for_service(timeout_sec=2.) and cancel.wait_for_service(timeout_sec=2.)
time.sleep(.3);f=run.call_async(Trigger.Request());assert started.wait(2.);time.sleep(.03)
timestamps['cancel_request_s']=time.monotonic();cf=cancel.call_async(Trigger.Request())
deadline=time.monotonic()+3.
while not cf.done() and time.monotonic()<deadline:time.sleep(.002)
assert cf.done() and f.done()
delay=(timestamps['cancel_callback_s']-timestamps['cancel_request_s'])*1000
inside=lambda s:timestamps['operation_start_s']<s<timestamps['operation_end_s']
result={'scope':'real ROS original callbacks, controlled blocking operation; no motor-stop latency claim',
    'original_node_sha256':hashlib.sha256((ROOT/'tests/original_node_snapshot.py').read_bytes()).hexdigest(),
    'cancel_callback_delay_ms':delay,'pose_published_during_block':sum(map(inside,published)),
    'pose_received_during_block':sum(map(inside,received)),'timestamps':timestamps,'rclpy_file':rclpy.__file__}
assert delay>250 and result['pose_published_during_block']>20 and result['pose_received_during_block']==0,result
(a.out/'results.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
stop.set();producer.join();executor.shutdown(timeout_sec=2.);thread.join(2.)
controller.destroy_node();client.destroy_node();rclpy.shutdown()
