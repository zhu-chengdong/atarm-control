#!/usr/bin/env python3
"""
ROS2 node for the Unitree A1 based tensegrity arm controller.

Subscriptions:
  deploy/set                  std_msgs/Bool              deploy or retract the arm
  bend/set_theta_phi_rad      std_msgs/Float64MultiArray [theta_rad, phi_rad]
  bend/set_theta_phi_deg      std_msgs/Float64MultiArray [theta_deg, phi_deg]
  atarm/micro_adjust_m        std_msgs/Float64MultiArray [micro_adjust_m]

Publications:
  motor_status/actuation_lengths_m  std_msgs/Float64MultiArray
  motor_status/actuation_angles_rad std_msgs/Float64MultiArray
  target/actuation_lengths_m        std_msgs/Float64MultiArray

Services:
  deploy                         std_srvs/Trigger
  retract                        std_srvs/Trigger
"""

import math
import json
from dataclasses import replace

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger

from .arm_controller_lib import TensegrityArm, ControlError

class UnitreeTensegrityControllerNode(Node):
    def __init__(self, serial_factory=None, *, camera_feedback=None, camera_goals=None, camera_config=None, visual_servo_config=None, bend_servo_config=None, bend_goal_template=None, bend_camera_config=None, cycle_config=None, cycle_plan=None,recovery_config=None,probe_config=None,window_progress_config=None,pause_budget_config=None):
        super().__init__('arm_controller_node')
        self.bend_goal_template=bend_goal_template
        self.cycle_plan=cycle_plan

        self.declare_parameter('port_active',          '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT6CAQYY-if00-port0')
        self.declare_parameter('port_deploy',          '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT5N1XWM-if00-port0')
        self.declare_parameter('is_dry_run',           False)
        self.declare_parameter('step_rate_hz',         50.0)
        self.declare_parameter('publish_rate_hz',      20.0)
        self.declare_parameter('auto_deploy_on_start', False)
        self.declare_parameter('deploy_delta_length',  0.05)
        self.declare_parameter('active_motor_0.id',         1)
        self.declare_parameter('active_motor_1.id',         0)
        self.declare_parameter('active_motor_2.id',         2)
        self.declare_parameter('active_motor_0.direction',  1)
        self.declare_parameter('active_motor_1.direction',  1)
        self.declare_parameter('active_motor_2.direction', -1)
        self.declare_parameter('deploy_motor.id',           2)
        self.declare_parameter('deploy_motor.direction',   0)
        self.declare_parameter('micro_adjust_max_step_m',  0.01)
        self.declare_parameter('deploy_motor_torque_gain_scale', 0.0)
        self.declare_parameter('deploy_timeout_s', 10.0)
        self.declare_parameter('deploy_speed_m_s', 0.01)
        self.declare_parameter('deploy_settle_s', 0.1)
        self.declare_parameter('feedback_max_age_s', 0.1)
        self.declare_parameter('feedback_max_speed_m_s', 1.0)
        self.declare_parameter('feedback_step_margin_m', 0.0005)
        self.declare_parameter('uncommanded_tolerance_m', 0.0015)
        self.declare_parameter('progress_min_m', 0.0005)
        self.declare_parameter('progress_timeout_s', 1.0)

        self.port_active          =          self.get_parameter('port_active'         ).get_parameter_value().string_value
        self.port_deploy          =          self.get_parameter('port_deploy'         ).get_parameter_value().string_value
        self.is_dry_run           =          self.get_parameter('is_dry_run'          ).get_parameter_value().bool_value
        self.step_rate_hz         = max(1.0, self.get_parameter('step_rate_hz'        ).get_parameter_value().double_value)
        self.publish_rate_hz      = max(1.0, self.get_parameter('publish_rate_hz'     ).get_parameter_value().double_value)
        self.auto_deploy_on_start =          self.get_parameter('auto_deploy_on_start').get_parameter_value().bool_value
        self.deploy_delta_length  =          self.get_parameter('deploy_delta_length' ).get_parameter_value().double_value
        self.micro_adjust_max_step_m =       self.get_parameter('micro_adjust_max_step_m').get_parameter_value().double_value
        self.deploy_motor_torque_gain_scale = self.get_parameter('deploy_motor_torque_gain_scale').get_parameter_value().double_value

        active_motor_ids = [
            self.get_parameter('active_motor_0.id').get_parameter_value().integer_value,
            self.get_parameter('active_motor_1.id').get_parameter_value().integer_value,
            self.get_parameter('active_motor_2.id').get_parameter_value().integer_value,
        ]
        active_motor_directions = [
            self.get_parameter('active_motor_0.direction').get_parameter_value().integer_value,
            self.get_parameter('active_motor_1.direction').get_parameter_value().integer_value,
            self.get_parameter('active_motor_2.direction').get_parameter_value().integer_value,
        ]
        deploy_motor_id        = self.get_parameter('deploy_motor.id'       ).get_parameter_value().integer_value
        deploy_motor_direction = self.get_parameter('deploy_motor.direction').get_parameter_value().integer_value

        self.arm = TensegrityArm(
            serial_port_active     =self.port_active,
            serial_port_deploy     =self.port_deploy,
            dry_run                =self.is_dry_run,
            is_debug               =False,
            active_motor_ids       =active_motor_ids,
            active_motor_directions=active_motor_directions,
            deploy_motor_id        =deploy_motor_id,
            deploy_motor_direction =deploy_motor_direction,
            deploy_delta_length    =self.deploy_delta_length,
            deploy_motor_torque_gain_scale=self.deploy_motor_torque_gain_scale,
            deploy_period_s=1.0 / self.step_rate_hz,
            deploy_timeout_s=self.get_parameter('deploy_timeout_s').value,
            deploy_speed_m_s=self.get_parameter('deploy_speed_m_s').value,
            deploy_settle_s=self.get_parameter('deploy_settle_s').value,
            feedback_max_age_s=self.get_parameter('feedback_max_age_s').value,
            serial_factory=serial_factory,
            camera_feedback=camera_feedback,
            camera_goals=camera_goals,
            camera_config=camera_config,
            visual_servo_config=visual_servo_config,
            bend_servo_config=bend_servo_config,
            bend_camera_config=bend_camera_config,
            cycle_config=cycle_config,
            recovery_config=recovery_config,
            probe_config=probe_config,
            window_progress_config=window_progress_config,
            pause_budget_config=pause_budget_config,
            feedback_max_speed_m_s=self.get_parameter('feedback_max_speed_m_s').value,
            feedback_step_margin_m=self.get_parameter('feedback_step_margin_m').value,
            uncommanded_tolerance_m=self.get_parameter('uncommanded_tolerance_m').value,
            progress_min_m=self.get_parameter('progress_min_m').value,
            progress_timeout_s=self.get_parameter('progress_timeout_s').value,
        )
        if not self.arm.initialize():
            self.arm.close()
            raise ControlError('Initial feedback validation failed.')

        qos = QoSProfile(depth=10)
        self.create_subscription(Bool,              'deploy/set',             self._deploy_callback,   qos)
        self.create_subscription(Float64MultiArray, 'bend/set_theta_phi_rad', self._bend_rad_callback, qos)
        self.create_subscription(Float64MultiArray, 'bend/set_theta_phi_deg', self._bend_deg_callback, qos)
        self.create_subscription(Float64MultiArray, 'bend/set_camera_position_m', self._bend_camera_position_callback, qos)
        self.create_subscription(Float64MultiArray, 'atarm/micro_adjust_m', self._micro_adjust_callback, qos)

        self.create_service(Trigger, 'deploy',    self._deploy_service)
        self.create_service(Trigger, 'retract',   self._retract_service)
        self.create_service(Trigger,'run_camera_cycle',self._cycle_service)
        self.create_service(Trigger,'cancel_camera_cycle',self._cancel_cycle_service)
        # Calling this asserts that external physical home and fault checks
        # have been performed. It requests verification, never automatic homing.
        self.create_service(Trigger,'recover_after_external_home_confirmation',self._recover_service)
        self.create_service(Trigger,'cancel_camera_recovery',self._cancel_recovery_service)
        # Calling this service is an explicit operator assertion that the
        # mechanism is physically at its retracted reference, not auto-homing.
        self.create_service(Trigger, 'confirm_retracted_reference', self._confirm_reference_service)

        self.actuation_lengths_curr_pub = self.create_publisher(Float64MultiArray, 'atarm/arm_status/actuation_lengths_m_current', qos)
        self.actuation_lengths_trgt_pub = self.create_publisher(Float64MultiArray, 'atarm/arm_status/actuation_lengths_m_target',  qos)
        self.motor_angles_curr_pub      = self.create_publisher(Float64MultiArray, 'atarm/arm_status/motor_angles_rad_current',    qos)
        self.controller_state_pub = self.create_publisher(String, 'atarm/arm_status/controller_state', qos)

        self.step_timer    = self.create_timer(1.0 / self.step_rate_hz,    self._step_timer_callback)
        self.publish_timer = self.create_timer(1.0 / self.publish_rate_hz, self._publish_status_callback)

        if self.auto_deploy_on_start:
            self.arm.set_deploy(True)

        self.get_logger().info(f'Unitree tensegrity controller initialised. dry_run={self.is_dry_run}, '
                               f'active_port={self.port_active if not self.is_dry_run else "N/A"}, '
                               f'deploy_port={self.port_deploy if not self.is_dry_run else "N/A"}.')

    def _micro_adjust_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) not in (1, 3, 4):
            self.get_logger().warn('micro_adjust expects a 1-, 3-, or 4-element Float64MultiArray, in metres.')
            return

        deltas = [float(v) for v in msg.data]
        capped = [max(-self.micro_adjust_max_step_m, min(self.micro_adjust_max_step_m, value)) for value in deltas]

        try:
            if len(capped) == 1:
                self.arm.apply_deploy_micro_adjust(capped[0])
                self.get_logger().info(f'Micro-adjust deploy motor by {capped[0]:.6f} m.')
            elif len(capped) == 3:
                self.arm.apply_active_micro_adjust(capped)
                self.get_logger().info(f'Micro-adjust active motors by {capped} m.')
            else:
                self.arm.apply_micro_adjust(capped)
                self.get_logger().info(f'Micro-adjust all motors by {capped} m.')
        except (ValueError, ControlError) as exc:
            self.get_logger().warn(f'Invalid micro-adjust command: {exc}')

    def _deploy_callback(self, msg: Bool) -> None:
        self._set_deploy(bool(msg.data))

    def _bend_rad_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 2:
            self.get_logger().warn('bend/set_theta_phi_rad expects [theta_rad, phi_rad].')
            return
        self._set_bend(float(msg.data[0]), float(msg.data[1]))

    def _bend_camera_position_callback(self,msg:Float64MultiArray) -> None:
        if self.bend_goal_template is None or len(msg.data)!=3 or not all(math.isfinite(v) for v in msg.data):
            self.get_logger().warn('Camera bend target requires an explicit goal frame/template and three finite metres.')
            return
        result=self.arm.move_bend_camera(replace(self.bend_goal_template,position_m=tuple(float(v) for v in msg.data)))
        self.get_logger().info(result['message'])

    def _bend_deg_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 2:
            self.get_logger().warn('bend/set_theta_phi_deg expects [theta_deg, phi_deg].')
            return
        self._set_bend(math.radians(float(msg.data[0])), math.radians(float(msg.data[1])))

    def _deploy_service(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        result = self._set_deploy(True)
        response.success = result['success']
        response.message = result['message']
        return response

    def _retract_service(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        result = self._set_deploy(False)
        response.success = result['success']
        response.message = result['message']
        return response

    def _cycle_service(self,_request,response):
        result=self.arm.run_camera_cycle(self.cycle_plan)
        response.success=result['success'];response.message=result['message']
        return response

    def _cancel_cycle_service(self,_request,response):
        result=self.arm.cancel_camera_cycle()
        response.success=result['success'];response.message=result['message']
        return response

    def _confirm_reference_service(self, _request, response):
        result = self.arm.confirm_retracted_reference(mechanically_confirmed=True)
        response.success = result['success']
        response.message = result['message']
        return response

    def _recover_service(self,_request,response):
        result=self.arm.recover_camera_cycle(mechanically_confirmed=True,fault_cleared=True)
        response.success=result['success'];response.message=result['message']
        return response

    def _cancel_recovery_service(self,_request,response):
        result=self.arm.cancel_camera_recovery()
        response.success=result['success'];response.message=result['message']
        return response

    def _set_deploy(self, deploy: bool):
        self.get_logger().info('Deploying arm.' if deploy else 'Retracting arm.')
        return self.arm.set_deploy(deploy)

    def _set_bend(self, theta: float, phi: float) -> None:
        if not self.arm.deployed:
            self.get_logger().warn('Ignoring bend command because arm is not deployed.')
            return
        if not self.arm.update_bend(theta, phi):
            self.get_logger().warn('Bend angle target rejected; camera-controlled mode requires a camera position target.')
            return
        self.get_logger().info(f'Bend target set: theta={theta:.6f} rad, phi={phi:.6f} rad')

    def _step_timer_callback(self) -> None:
        self.arm.step()

    def _publish_status_callback(self) -> None:
        feedbacks = self.arm.get_feedbacks()

        lengths_msg      = Float64MultiArray()
        lengths_msg.data = [float(fb['actuate_len']) if fb['feedback_valid'] else float('nan') for fb in feedbacks]
        self.actuation_lengths_curr_pub.publish(lengths_msg)

        angles_msg      = Float64MultiArray()
        angles_msg.data = [float(fb['actuate_ang']) if fb['feedback_valid'] else float('nan') for fb in feedbacks]
        self.motor_angles_curr_pub.publish(angles_msg)

        targets_msg      = Float64MultiArray()
        targets_msg.data = [float(value) for value in self.arm.final_target_actuation_lengths]
        self.actuation_lengths_trgt_pub.publish(targets_msg)
        state_msg = String()
        state_msg.data = json.dumps({'state': self.arm.deploy_state, 'deployed': self.arm.deployed,
            'motion': self.arm.last_motion_result,
            'feedback_valid': [fb['feedback_valid'] for fb in feedbacks],
            'feedback_age_s': [fb['feedback_age_s'] for fb in feedbacks],
            'transport_frame_id': [fb['transport_frame_id'] for fb in feedbacks],
            'rx_monotonic_ns': [fb['rx_monotonic_ns'] for fb in feedbacks],
            'freshness_basis': [fb['freshness_basis'] for fb in feedbacks],
            'device_sequence': [None] * 4, 'device_sample_time': [None] * 4,
            'motion_checks':[fb['motion_guard'] for fb in feedbacks],
            'motion_progress':self.arm.motion_progress,
            'camera_feedback':self.arm.camera_status(),
            'visual_servo':self.arm.visual_servo_status(),
            'bend_visual':self.arm.bend_status(),
            'camera_cycle':self.arm.cycle_status(),'camera_recovery':self.arm.recovery_status(),
            'recovery_probe':self.arm.probe_status()}, allow_nan=False)
        self.controller_state_pub.publish(state_msg)

    def destroy_node(self) -> None:
        try:
            # Shutdown does not perform an unobserved or unbounded movement.
            self.step_timer.cancel()
            self.publish_timer.cancel()
            self.arm.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = UnitreeTensegrityControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().fatal(f'Failed to run node: {exc}')
        else:
            print(f'Failed to run node: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# ================================================================
# 主程序 - 演示
# ================================================================

if __name__ == "__main__":
    main()
