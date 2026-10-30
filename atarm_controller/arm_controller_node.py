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
from geometry_msgs.msg import PoseStamped
from .ros_pose_bridge import RosPoseBridge
from .ros_motion_worker import RosMotionWorker
from .ros_control_config import load_control_config

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger

from .arm_controller_lib import TensegrityArm, ControlError

class UnitreeTensegrityControllerNode(Node):
    def __init__(self, serial_factory=None, *, camera_feedback=None, camera_goals=None, camera_config=None, visual_servo_config=None, bend_servo_config=None, bend_goal_template=None, bend_camera_config=None, cycle_config=None, cycle_plan=None,recovery_config=None,probe_config=None,window_progress_config=None,pause_budget_config=None, **node_options):
        super().__init__('arm_controller_node', **node_options)
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

        self.declare_parameter('control_config_path','')
        self.declare_parameter('camera_pose_topic','atarm/camera/pose')
        self.declare_parameter('camera_source_status_topic','atarm/camera/source_status')
        config_path=self.get_parameter('control_config_path').value
        options={}
        if config_path:
            options,reference=load_control_config(config_path)
            camera_goals=options['camera_goals'];camera_config=options['camera_config']
            bend_camera_config=options['bend_camera_config'];visual_servo_config=options['visual_servo_config']
            bend_servo_config=options['bend_servo_config'];cycle_config=options['cycle_config']
            recovery_config=options['recovery_config'];probe_config=options.get('probe_config')
            window_progress_config=options['window_progress_config'];pause_budget_config=options['pause_budget_config']
            self.cycle_plan=options['cycle_plan'];self.bend_goal_template=options['bend_goal_template']
        elif camera_goals:
            g=camera_goals[False];reference={k:getattr(g,k) for k in ('parent_frame','child_frame','map_epoch')}
        else:
            raise ControlError('Explicit control_config_path or fully configured closed-loop goals required.')
        self.pose_bridge=RosPoseBridge(**reference,max_age_s=(camera_config.max_age_s if camera_config else .1))
        if camera_feedback is None:camera_feedback=self.pose_bridge

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

        self.motion_worker=RosMotionWorker(self.arm,period_s=1.0/self.step_rate_hz)

        qos = QoSProfile(depth=10)
        self.create_subscription(PoseStamped,self.get_parameter('camera_pose_topic').value,self._pose_callback,QoSProfile(depth=4))
        self.create_subscription(String,self.get_parameter('camera_source_status_topic').value,self._source_status_callback,QoSProfile(depth=4))
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

        self.publish_timer = self.create_timer(1.0 / self.publish_rate_hz, self._publish_status_callback)

        if self.auto_deploy_on_start:
            self._set_deploy(True)

        self.get_logger().info(f'Unitree tensegrity controller initialised. dry_run={self.is_dry_run}, '
                               f'active_port={self.port_active if not self.is_dry_run else "N/A"}, '
                               f'deploy_port={self.port_deploy if not self.is_dry_run else "N/A"}.')

    def _pose_callback(self,msg):
        try:self.pose_bridge.pose(msg,self.get_clock().now().nanoseconds)
        except Exception as exc:
            self.pose_bridge.invalidate(str(exc));self.get_logger().error(str(exc))

    def _source_status_callback(self,msg):
        try:self.pose_bridge.status(msg,self.get_clock().now().nanoseconds)
        except Exception as exc:
            self.pose_bridge.invalidate(str(exc));self.get_logger().error(str(exc))

    def _response(self,response,result):
        # success acknowledges acceptance only. Final result is on controller_state.
        response.success=result.get('success') is True
        response.message=json.dumps(result,allow_nan=False)
        return response

    def _submit(self,name,call):return self.motion_worker.submit(name,call)

    def _micro_adjust_callback(self,msg):
        if len(msg.data) not in (1,3,4) or not all(math.isfinite(v) for v in msg.data):return
        values=tuple(max(-self.micro_adjust_max_step_m,min(self.micro_adjust_max_step_m,float(x))) for x in msg.data)
        def call():
            fn=self.arm.apply_deploy_micro_adjust if len(values)==1 else self.arm.apply_active_micro_adjust if len(values)==3 else self.arm.apply_micro_adjust
            result=fn(values[0] if len(values)==1 else values)
            return result if isinstance(result,dict) else {'success':result is not False,'complete':False,'message':'Adjustment command applied' if result is not False else 'Adjustment rejected'}
        self._submit('micro_adjust',call)

    def _deploy_callback(self,msg):self._set_deploy(bool(msg.data))
    def _bend_rad_callback(self,msg):
        if len(msg.data)>=2 and all(math.isfinite(v) for v in msg.data[:2]):self._set_bend(float(msg.data[0]),float(msg.data[1]))
    def _bend_deg_callback(self,msg):
        if len(msg.data)>=2 and all(math.isfinite(v) for v in msg.data[:2]):self._set_bend(math.radians(msg.data[0]),math.radians(msg.data[1]))
    def _bend_camera_position_callback(self,msg):
        if self.bend_goal_template is None or len(msg.data)!=3 or not all(math.isfinite(v) for v in msg.data):return
        goal=replace(self.bend_goal_template,position_m=tuple(float(v) for v in msg.data))
        self._submit('bend_camera',lambda:self.arm.move_bend_camera(goal))
    def _deploy_service(self,request,response):return self._response(response,self._set_deploy(True))
    def _retract_service(self,request,response):return self._response(response,self._set_deploy(False))
    def _cycle_service(self,request,response):return self._response(response,self._submit('camera_cycle',lambda:self.arm.run_camera_cycle(self.cycle_plan)))
    def _cancel_cycle_service(self,request,response):return self._response(response,self.motion_worker.cancel())
    def _confirm_reference_service(self,request,response):
        return self._response(response,self._submit('confirm_reference',lambda:self.arm.confirm_retracted_reference(mechanically_confirmed=True)))
    def _recover_service(self,request,response):
        return self._response(response,self._submit('recover',lambda:self.arm.recover_camera_cycle(mechanically_confirmed=True,fault_cleared=True)))
    def _cancel_recovery_service(self,request,response):return self._response(response,self.motion_worker.cancel())
    def _set_deploy(self,deploy):return self._submit('deploy' if deploy else 'retract',lambda:self.arm.set_deploy(deploy))
    def _set_bend(self,theta,phi):
        def call():
            if not self.arm.deployed:return {'success':False,'complete':False,'message':'Arm is not deployed'}
            result=self.arm.update_bend(theta,phi)
            return {'success':bool(result),'complete':False,'message':'Bend target accepted' if result else 'Camera position target required'}
        return self._submit('bend_angles',call)

    def _publish_status_callback(self):
        status=self.motion_worker.snapshot()
        status['ros_pose_source']=self.pose_bridge.diagnostics()
        feedback=status.get('feedbacks',[])
        if feedback:
            for publisher,key in ((self.actuation_lengths_curr_pub,'actuate_len'),(self.motor_angles_curr_pub,'actuate_ang')):
                msg=Float64MultiArray();msg.data=[float(v[key]) if v.get(key) is not None else float('nan') for v in feedback];publisher.publish(msg)
            msg=Float64MultiArray();msg.data=[float(v) for v in status.get('targets',[])];self.actuation_lengths_trgt_pub.publish(msg)
        msg=String();msg.data=json.dumps(status,allow_nan=False);self.controller_state_pub.publish(msg)

    def destroy_node(self):
        try:
            if hasattr(self,'publish_timer'):self.publish_timer.cancel()
            if hasattr(self,'motion_worker'):self.motion_worker.close()
            elif hasattr(self,'arm'):self.arm.close()
        finally:super().destroy_node()


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
