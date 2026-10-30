#!/usr/bin/env python3
"""
ROS2 node for two DYNAMIXEL XC330-T288-T servos in Extended Position Control Mode for the end effector.

- Servo ID 1: pan_motion
- Servo ID 2: arc_motion

Topics:
  /pan_motion/set_angle_deg   std_msgs/Float64   target output-shaft angle in degrees
  /arc_motion/set_angle_deg   std_msgs/Float64   target output-shaft angle in degrees
  /pan_motion/present_angle_deg std_msgs/Float64 current output-shaft angle in degrees
  /arc_motion/present_angle_deg std_msgs/Float64 current output-shaft angle in degrees

Services:
  /pan_motion/set_zero        std_srvs/Trigger   set current position as output angle 0 deg
  /arc_motion/set_zero        std_srvs/Trigger   set current position as output angle 0 deg
#   /set_all_zero               std_srvs/Trigger   set current position as zero for both servos
"""

import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

import rclpy
from rclpy.node import Node, Publisher
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64
from std_srvs.srv import Trigger


# DYNAMIXEL X-series / XC330-T288-T control table addresses
ADDR_OPERATING_MODE       = 11
ADDR_HOMING_OFFSET        = 20
ADDR_TORQUE_ENABLE        = 64
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY     = 112
ADDR_GOAL_POSITION        = 116
ADDR_PRESENT_POSITION     = 132

PROTOCOL_VERSION = 2.0
BAUDRATE_DEFAULT = 57600

TORQUE_DISABLE = 0
TORQUE_ENABLE  = 1
EXTENDED_POSITION_CONTROL = 4

TICKS_PER_REV  = 4096.0
UINT32_MASK    = 0xFFFFFFFF
INT32_SIGN_BIT = 0x80000000
HOMING_OFFSET_MIN = -1044479
HOMING_OFFSET_MAX = 1044479
PORT_BUSY_CLEAR_WAIT_S = 0.05
PORT_BUSY_CLEAR_INTERVAL_S = 0.005


@dataclass
class ServoConfig:
    dxl_id:              int
    name:                str
    gear_ratio:          float
    homing_offset_ticks: int = 0
    invert_di:           bool = False
    min_output_deg:      Optional[float] = None
    max_output_deg:      Optional[float] = None


class EndEffectorControllerNode(Node):
    def __init__(self):
        super().__init__('end_effector_controller_node')

        self.declare_parameter('device_name', '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBEQOED-if00-port0')
        self.declare_parameter('baudrate',    BAUDRATE_DEFAULT)
        self.declare_parameter('is_dry_run',  False)
        self.declare_parameter('publish_rate_hz', 20.0)

        self.declare_parameter('pan_motion.gear_ratio', 154.0/19.0)
        self.declare_parameter('arc_motion.gear_ratio', 100.0/6.0*2.0)  # 100:6 gear, module 2.0 spur gear, so it is 100:6*2.0 ratio
        self.declare_parameter('pan_motion.invert_direction', False)
        self.declare_parameter('arc_motion.invert_direction', True)
        self.declare_parameter('arc_motion.min_output_deg', 0.0)
        self.declare_parameter('arc_motion.max_output_deg', 45.0)

        self.device_name                  = self.get_parameter('device_name').get_parameter_value().string_value
        self.baudrate                     = self.get_parameter('baudrate'   ).get_parameter_value().integer_value
        self.is_dry_run                   = self.get_parameter('is_dry_run' ).get_parameter_value().bool_value
        self.publish_rate_hz              = max(1.0, self.get_parameter('publish_rate_hz').get_parameter_value().double_value)

        self.servos: Dict[str, ServoConfig] = {
            'pan_motion': ServoConfig(
                dxl_id    =1,
                name      ='pan_motion',
                gear_ratio=self.get_parameter('pan_motion.gear_ratio'      ).get_parameter_value().double_value,
                invert_di =self.get_parameter('pan_motion.invert_direction').get_parameter_value().bool_value,
            ),
            'arc_motion': ServoConfig(
                dxl_id        =2,
                name          ='arc_motion',
                gear_ratio    =self.get_parameter('arc_motion.gear_ratio'      ).get_parameter_value().double_value,
                invert_di     =self.get_parameter('arc_motion.invert_direction').get_parameter_value().bool_value,
                min_output_deg=self.get_parameter('arc_motion.min_output_deg'  ).get_parameter_value().double_value,
                max_output_deg=self.get_parameter('arc_motion.max_output_deg'  ).get_parameter_value().double_value,
            ),
        }

        self.port_handler: Optional[PortHandler]     = None
        self.packet_handler: Optional[PacketHandler] = None # pyright: ignore[reportGeneralTypeIssues]
        self._is_shutting_down = False
        self._sim_present_ticks:  Dict[str, int] = {name: 0 for name in self.servos}
        self._sim_homing_offsets: Dict[str, int] = {name: 0 for name in self.servos}

        if not self.is_dry_run:
            self._init_port()
            for servo in self.servos.values():
                self._setup_servo(servo)
                self._load_servo_homing_offset(servo)
        else:
            self.get_logger().warn('Dry-run mode enabled: no serial communication will be performed.')

        qos = QoSProfile(depth=10)

        self.angle_publishers: Dict[str, Publisher] = {}
        for name, servo in self.servos.items():
            self.create_subscription(Float64, f'{name}/set_angle_deg',   lambda msg, servo_name=name: self._set_angle_callback(msg, servo_name), qos)
            self.create_subscription(Float64, f'{name}/force_angle_deg', lambda msg, servo_name=name: self._force_angle_callback(msg, servo_name), qos)
            self.create_service(     Trigger, f'{name}/set_zero',        lambda request, response, servo_name=name: self._set_zero_service(request, response, servo_name))
            self.angle_publishers[name] = self.create_publisher(Float64, f'{name}/present_angle_deg', qos)

        self.present_angle_timer = self.create_timer(1.0 / self.publish_rate_hz, self._publish_present_angles)

        self.get_logger().info('End effector controller initialised.')
        for servo in self.servos.values():
            self.get_logger().info(f'{servo.name}: ID={servo.dxl_id}, gear_ratio={servo.gear_ratio}, invert_direction={servo.invert_di}, homing_offset_ticks={servo.homing_offset_ticks}')

    # --------------------------- initialisation ---------------------------

    def _init_port(self) -> None:
        """连接舵机"""
        self.port_handler   = PortHandler(self.device_name)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            raise RuntimeError(f'Failed to open port: {self.device_name}')
        if not self.port_handler.setBaudRate(self.baudrate):
            raise RuntimeError(f'Failed to set baudrate: {self.baudrate}')

        self.get_logger().info(f'Opened {self.device_name} at {self.baudrate} baud.')

    def _setup_servo(self, servo: ServoConfig) -> None:
        """设置为扩展位置控制模式"""
        # Operating Mode is in EEPROM area, so torque must be disabled before changing it.
        self._write1(servo.dxl_id, ADDR_TORQUE_ENABLE,  TORQUE_DISABLE,            'torque disable')
        self._write1(servo.dxl_id, ADDR_OPERATING_MODE, EXTENDED_POSITION_CONTROL, 'extended position mode')
        self._write1(servo.dxl_id, ADDR_TORQUE_ENABLE,  TORQUE_ENABLE,             'torque enable')

    # --------------------------- ROS callbacks ----------------------------

    def _set_angle_callback(self, msg: Float64, servo_name: str) -> None:
        if self._is_shutting_down:
            return

        servo                = self.servos[servo_name]
        requested_output_deg = float(msg.data)
        target_output_deg    = self._clamp_output_deg(servo, requested_output_deg)

        if target_output_deg != requested_output_deg:
            self.get_logger().warn(f'{servo.name}: requested target {requested_output_deg:.6f} deg clamped to {target_output_deg:.6f} deg')

        self._force_angle_callback(Float64(data=target_output_deg), servo_name)

        # if self._is_shutting_down:
        #     return

        # servo                = self.servos[servo_name]
        # requested_output_deg = float(msg.data)
        # target_output_deg    = self._clamp_output_deg(servo, requested_output_deg)
        # target_ticks         = self.output_deg_to_ticks(servo, target_output_deg)

        # if target_output_deg != requested_output_deg:
        #     self.get_logger().warn(f'{servo.name}: requested target {requested_output_deg:.6f} deg clamped to {target_output_deg:.6f} deg')

        # if self.is_dry_run:
        #     self._sim_present_ticks[servo_name] = target_ticks
        #     self.get_logger().info(f'[dry-run] {servo.name}: output target={target_output_deg:.6f} deg, goal_position={target_ticks} ticks')
        #     return

        # self._write4(servo.dxl_id, ADDR_GOAL_POSITION, self._int32_to_uint32(target_ticks), 'goal position')
        # self.get_logger().info(f'{servo.name}: output target={target_output_deg:.6f} deg, motor target={target_output_deg * servo.gear_ratio:.6f} deg, goal_position={target_ticks} ticks')

    def _force_angle_callback(self, msg: Float64, servo_name: str) -> None:
        if self._is_shutting_down:
            return

        servo                = self.servos[servo_name]
        requested_output_deg = float(msg.data)
        target_ticks         = self.output_deg_to_ticks(servo, requested_output_deg)

        if self.is_dry_run:
            self._sim_present_ticks[servo_name] = target_ticks
            self.get_logger().info(f'[dry-run] {servo.name}: output target={requested_output_deg:.6f} deg, goal_position={target_ticks} ticks')
            return

        self._write4(servo.dxl_id, ADDR_GOAL_POSITION, self._int32_to_uint32(target_ticks), 'goal position')
        self.get_logger().info(f'{servo.name}: output target={requested_output_deg:.6f} deg, motor target={requested_output_deg * servo.gear_ratio:.6f} deg, goal_position={target_ticks} ticks')

    def _set_zero_service(self, _request: Trigger.Request, response: Trigger.Response, servo_name: str) -> Trigger.Response:
        if self._is_shutting_down:
            response.success = False
            response.message = 'Controller is shutting down.'
            return response

        servo = self.servos[servo_name]
        ok    = self._set_hardware_zero(servo)
        response.success = ok
        response.message = f'{servo.name} homing offset set to {servo.homing_offset_ticks} ticks.' if ok else f'Failed to write {servo.name} homing offset.'
        return response

    def _publish_present_angles(self) -> None:
        if self._is_shutting_down:
            return

        for name, servo in self.servos.items():
            if self.is_dry_run:
                present_ticks = self._sim_present_ticks[name]
            else:
                present_ticks, ok = self._read_present_position(servo.dxl_id)
                if not ok or present_ticks is None:
                    continue

            msg = Float64()
            msg.data = self.ticks_to_output_deg(servo, present_ticks)
            self.angle_publishers[name].publish(msg)

    # --------------------------- zero and conversion ----------------------

    def _set_hardware_zero(self, servo: ServoConfig) -> bool:
        if self.is_dry_run:
            present_ticks = self._sim_present_ticks[servo.name]
            new_offset    = self._sim_homing_offsets[servo.name] - present_ticks
            if not self._homing_offset_in_range(new_offset):
                return False
            self._sim_homing_offsets[servo.name] = new_offset
            servo.homing_offset_ticks            = new_offset
            self._sim_present_ticks[servo.name]  = 0
            return True

        present_ticks, ok = self._read_present_position(servo.dxl_id)
        if not ok or present_ticks is None:
            return False
        homing_offset, ok = self._read_homing_offset(servo.dxl_id)
        if not ok or homing_offset is None:
            return False

        new_offset = homing_offset - present_ticks
        if not self._homing_offset_in_range(new_offset):
            self.get_logger().error(f'{servo.name}: calculated homing offset {new_offset} is outside DYNAMIXEL range [{HOMING_OFFSET_MIN}, {HOMING_OFFSET_MAX}]')
            return False

        if not self._write1(servo.dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, 'torque disable for homing offset'):
            return False
        ok        = self._write4(servo.dxl_id, ADDR_HOMING_OFFSET, self._int32_to_uint32(new_offset), 'homing offset')
        torque_ok = self._write1(servo.dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE,                     'torque enable after homing offset')
        if not ok or not torque_ok:
            return False

        servo.homing_offset_ticks = new_offset
        self.get_logger().info(f'{servo.name}: hardware zero stored, present_position={present_ticks} ticks, homing_offset={new_offset} ticks')
        return True

    def _load_servo_homing_offset(self, servo: ServoConfig) -> bool:
        homing_offset, ok = self._read_homing_offset(servo.dxl_id)
        if ok and homing_offset is not None:
            servo.homing_offset_ticks = homing_offset
        return ok

    def output_deg_to_ticks(self, servo: ServoConfig, output_deg: float) -> int:
        direction = -1.0 if servo.invert_di else 1.0
        motor_deg = direction * output_deg * servo.gear_ratio
        delta_ticks = int(round(motor_deg / 360.0 * TICKS_PER_REV))
        return delta_ticks

    def ticks_to_output_deg(self, servo: ServoConfig, ticks: int) -> float:
        direction = -1.0 if servo.invert_di else 1.0
        motor_deg = int(ticks) / TICKS_PER_REV * 360.0
        return direction * motor_deg / servo.gear_ratio

    @staticmethod
    def _clamp_output_deg(servo: ServoConfig, output_deg: float) -> float:
        if servo.min_output_deg is not None:
            output_deg = max(servo.min_output_deg, output_deg)
        if servo.max_output_deg is not None:
            output_deg = min(servo.max_output_deg, output_deg)
        return output_deg

    # --------------------------- low-level SDK wrappers -------------------

    def _write1(self, dxl_id: int, address: int, value: int, label: str) -> bool:
        assert self.port_handler is not None and self.packet_handler is not None
        self._ensure_port_available()
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, address, int(value))
        ok = self._check_result(dxl_id, dxl_comm_result, dxl_error, f'write {label}')
        return ok

    def _write4(self, dxl_id: int, address: int, value: int, label: str) -> bool:
        assert self.port_handler is not None and self.packet_handler is not None
        self._ensure_port_available()
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(self.port_handler, dxl_id, address, int(value) & UINT32_MASK)
        ok = self._check_result(dxl_id, dxl_comm_result, dxl_error, f'write {label}')
        return ok

    def _read_present_position(self, dxl_id: int) -> Tuple[Optional[int], bool]:
        assert self.port_handler is not None and self.packet_handler is not None
        self._ensure_port_available()
        raw, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, dxl_id, ADDR_PRESENT_POSITION)
        ok = self._check_result(dxl_id, dxl_comm_result, dxl_error, 'read present position', log_success=False)
        if not ok:
            return None, False
        return self._uint32_to_int32(raw), True

    def _read_homing_offset(self, dxl_id: int) -> Tuple[Optional[int], bool]:
        assert self.port_handler is not None and self.packet_handler is not None
        self._ensure_port_available()
        raw, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, dxl_id, ADDR_HOMING_OFFSET)
        ok = self._check_result(dxl_id, dxl_comm_result, dxl_error, 'read homing offset', log_success=False)
        if not ok:
            return None, False
        return self._uint32_to_int32(raw), True

    @staticmethod
    def _homing_offset_in_range(value: int) -> bool:
        return HOMING_OFFSET_MIN <= int(value) <= HOMING_OFFSET_MAX

    def _check_result(self, dxl_id: int, dxl_comm_result: int, dxl_error: int, action: str, log_success: bool = True) -> bool:
        assert self.packet_handler is not None
        if dxl_comm_result != COMM_SUCCESS:
            self._safe_log('error', f'ID {dxl_id}: failed to {action}: {self.packet_handler.getTxRxResult(dxl_comm_result)}')
            return False
        if dxl_error != 0:
            self._safe_log('error', f'ID {dxl_id}: failed to {action}: {self.packet_handler.getRxPacketError(dxl_error)}')
            return False
        if log_success:
            self._safe_log('info', f'ID {dxl_id}: succeeded to {action}')
        return True

    def _ensure_port_available(self) -> None:
        if self.port_handler is None or not self.port_handler.is_using:
            return

        deadline = time.monotonic() + PORT_BUSY_CLEAR_WAIT_S
        while self.port_handler.is_using and time.monotonic() < deadline:
            time.sleep(PORT_BUSY_CLEAR_INTERVAL_S)

        if self.port_handler is not None and self.port_handler.is_using:
            self.port_handler.is_using = False

    def _safe_log(self, level: str, message: str) -> None:
        if rclpy.ok():
            try:
                getattr(self.get_logger(), level)(message)
                return
            except Exception:
                pass
        print(f'[{level.upper()}] {message}')

    @staticmethod
    def _uint32_to_int32(value: int) -> int:
        value = int(value) & UINT32_MASK
        if value & INT32_SIGN_BIT:
            return value - (1 << 32)
        return value

    @staticmethod
    def _int32_to_uint32(value: int) -> int:
        return int(value) & UINT32_MASK

    # --------------------------- shutdown --------------------------------

    def destroy_node(self) -> None:
        self._is_shutting_down = True
        self.present_angle_timer.cancel()

        if not self.is_dry_run and self.port_handler is not None and self.packet_handler is not None:
            ''' 逐个关扭矩 确保安全关闭 每个舵机试5次 '''
            for servo in self.servos.values():
                for i in range(5):
                    self._ensure_port_available()
                    if self._write1(servo.dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, 'torque disable on shutdown'):
                        break
            self.port_handler.closePort()
            self._safe_log('info', 'Torque disabled and port closed.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = EndEffectorControllerNode()
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


if __name__ == '__main__':
    main()
