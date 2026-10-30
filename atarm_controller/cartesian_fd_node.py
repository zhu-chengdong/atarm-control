"""External-vision Cartesian ROS node with explicit calibrated FD injection.

CLI accepts caller-created PTY descriptors for offline validation only. No
serial path discovery, hardware opening, automatic homing or autodeploy.
"""
import argparse
import json
import os
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from .external_vision import ExternalVision
from .actuator_io import PtyFdFactory
from .cartesian_fd_runtime import FdJobRunner,load_session


class FdCartesianNode(Node):
    def __init__(self,session_path,transport_factory,**kwargs):
        super().__init__('cartesian_fd_node',**kwargs)
        d,cfg,channels=load_session(session_path)
        self.vision=ExternalVision(d['vision'],cfg.max_pose_age_s)
        self.runner=FdJobRunner(cfg,channels,transport_factory,self.vision,speed_m_s=d['speed_m_s'],calibration_id=d['calibration_id'])
        self.pp=self.create_subscription(PoseStamped,'atarm/camera/pose',self.pose,8)
        self.sp=self.create_subscription(String,'atarm/camera/source_status',self.source_status,8)
        self.ack=self.create_publisher(String,'atarm/cartesian/ack',10)
        self.state=self.create_publisher(String,'atarm/cartesian/state',10)
        self.target=self.create_subscription(String,'atarm/cartesian/target',self.goal,10)
        self.cs=self.create_service(Trigger,'atarm/cartesian/cancel',self.cancel)
        self.rs=self.create_service(Trigger,'atarm/cartesian/confirm_reference',self.confirm)
        self.sequence=0;self.timer=self.create_timer(.05,self.publish_state)

    def pose(self,message):
        try:self.vision.pose(message,self.get_clock().now().nanoseconds)
        except ValueError:pass  # bridge latches the exact cause for the owner

    def source_status(self,message):
        try:self.vision.status(message,self.get_clock().now().nanoseconds)
        except ValueError:pass

    @staticmethod
    def string(value):
        m=String();m.data=json.dumps(value,allow_nan=False);return m

    def goal(self,message):self.ack.publish(self.string(self.runner.submit_json(message.data)))

    def cancel(self,request,response):
        d=self.runner.cancel();response.success=d['accepted'];response.message=json.dumps(d);return response

    def confirm(self,request,response):
        d=self.runner.confirm_reference();response.success=d['accepted'];response.message=json.dumps(d);return response

    def publish_state(self):
        self.sequence+=1;d=self.runner.snapshot();d.update(state_sequence=self.sequence,published_ros_stamp_ns=self.get_clock().now().nanoseconds)
        self.state.publish(self.string(d))

    def destroy_node(self):
        try:self.timer.cancel();self.runner.close()
        finally:super().destroy_node()


def main(args=None):
    p=argparse.ArgumentParser()
    p.add_argument('--offline-pty',action='store_true');p.add_argument('--active-fd',type=int,required=True)
    p.add_argument('--deploy-fd',type=int,required=True);p.add_argument('--session-config',type=Path,required=True)
    p.add_argument('--audit',type=Path,required=True)
    a,ros_args=p.parse_known_args(args)
    d,_,_=load_session(a.session_config)
    if not a.offline_pty or d.get('calibration_kind')!='synthetic-pty-validation':
        p.error('This CLI is offline PTY validation only; physical commissioning requires explicit FD provisioning and calibrated configuration.')
    factory=PtyFdFactory({'active':a.active_fd,'deploy':a.deploy_fd})
    rclpy.init(args=ros_args);node=None;executor=SingleThreadedExecutor()
    try:
        node=FdCartesianNode(a.session_config,factory);executor.add_node(node);executor.spin()
    except KeyboardInterrupt:pass
    finally:
        executor.shutdown()
        if node:
            node.destroy_node();audit=dict(node.runner.audit,worker_alive=node.runner.thread.is_alive(),pid=os.getpid(),
                rclpy_file=rclpy.__file__,physical_devices_opened=False)
            a.audit.write_text(json.dumps(audit,indent=2,allow_nan=False)+'\n',encoding='utf-8')
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
