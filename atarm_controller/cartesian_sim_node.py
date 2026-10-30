"""ROS2 front end for synthetic Cartesian control, with no physical adapter."""
import json
import os
from pathlib import Path
import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger
from .cartesian_jobs import CartesianJobRunner


class CartesianSimNode(Node):
    def __init__(self, *, runner_factory=CartesianJobRunner, **kwargs):
        super().__init__('cartesian_sim_node', **kwargs)
        self.declare_parameter('strategy', 'alternating')
        self.declare_parameter('simulation_speed', 4.)
        self.declare_parameter('test_fault', '')
        self.runner = runner_factory(strategy=self.get_parameter('strategy').value,
            simulation_speed=self.get_parameter('simulation_speed').value,
            fault=self.get_parameter('test_fault').value)
        qos = QoSProfile(depth=10)
        self.ack_pub = self.create_publisher(String, 'atarm/cartesian_sim/ack', qos)
        self.state_pub = self.create_publisher(String, 'atarm/cartesian_sim/state', qos)
        self.goal_sub = self.create_subscription(String, 'atarm/cartesian_sim/target', self.goal_callback, qos)
        self.cancel_srv = self.create_service(Trigger, 'atarm/cartesian_sim/cancel', self.cancel_callback)
        self.reset_srv = self.create_service(Trigger, 'atarm/cartesian_sim/reset', self.reset_callback)
        self.state_sequence = 0
        self.timer = self.create_timer(.05, self.publish_state)
        self.get_logger().info('SIMULATION ONLY: no native SDK, serial adapter, or physical device is loaded.')

    @staticmethod
    def message(data):
        msg = String(); msg.data = json.dumps(data, allow_nan=False)
        return msg

    def goal_callback(self, msg):
        self.ack_pub.publish(self.message(self.runner.submit_json(msg.data)))

    def cancel_callback(self, request, response):
        data = self.runner.cancel(); response.success = data['accepted']
        response.message = json.dumps(data, allow_nan=False)
        return response

    def reset_callback(self, request, response):
        data = self.runner.reset(); response.success = data['accepted']
        response.message = json.dumps(data, allow_nan=False)
        return response

    def publish_state(self):
        data = self.runner.snapshot(); self.state_sequence += 1
        data.update(state_sequence=self.state_sequence, published_ros_stamp_ns=self.get_clock().now().nanoseconds)
        self.state_pub.publish(self.message(data))

    def destroy_node(self):
        try:
            self.timer.cancel(); self.runner.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args); node = None; executor = SingleThreadedExecutor()
    try:
        node = CartesianSimNode(); executor.add_node(node); executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
            if os.environ.get('ATARM_SIM_AUDIT_PATH'):
                Path(os.environ['ATARM_SIM_AUDIT_PATH']).write_text(json.dumps({
                    'pid': os.getpid(), 'worker_alive': node.runner.thread.is_alive(),
                    'simulation_stopped': node.runner.plant.stopped, 'io_threads': sorted(node.runner.plant.io_threads),
                    'last_state': node.runner.snapshot(),
                    'hardware_modules_loaded': [x for x in sys.modules if x.startswith(('atarm_controller.serial_adapter', 'atarm_controller.arm_controller_lib', 'unitree_actuator_sdk'))]
                }, allow_nan=False, indent=2), encoding='utf-8')
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
