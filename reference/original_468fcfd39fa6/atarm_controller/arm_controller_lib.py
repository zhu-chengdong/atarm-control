#!/usr/bin/env python3
"""
Tensegrity Arm Controller - 基于宇树 A1 电机和 SDK 的张拉整体机械臂控制程序
"""

import time
import math
from typing import Optional
from unitree_actuator_sdk import SerialPort, MotorCmd, MotorData, MotorType, MotorMode, queryMotorMode, queryGearRatio


# ================================================================
# 电机封装
# ================================================================
class A1Motor:
    """封装单个 A1 电机的控制逻辑。"""
    def __init__(self, motor_id: int, serial_ptr: SerialPort, set_foc_on_init: bool, direction: int, *, is_debug: bool, is_deploy_motor: bool = False, torque_gain_scale: float = 1.0):
        self.motor_id         = motor_id
        self.serial           = serial_ptr
        self.is_debug         = is_debug
        self.is_deploy_motor  = is_deploy_motor

        # 电机正方向
        # +1: 正常方向 -1: bobbin 反装，控制方向取反
        if direction not in (1, -1):
            raise ValueError("direction must be either +1 or -1")
        self.direction = direction

        # 电机命令结构体 以及其中的常量设置
        self.cmd            = MotorCmd()
        self.cmd.motorType  = MotorType.A1
        self.cmd.mode       = queryMotorMode(MotorType.A1, MotorMode.FOC)
        self.cmd.id         = motor_id
        # 电机反馈结构体
        self.data           = MotorData()
        self.data.motorType = MotorType.A1

        # 电机参数
        self.gear_ratio     = queryGearRatio(MotorType.A1)
        self.bobbin_radius  = 26 / 1000.0
        self.max_torque_nm  = 10.0
        if self.is_deploy_motor:
            self.max_torque_nm *= max(1.0, torque_gain_scale)
        print(f"[Motor {self.motor_id}] Initialized with gear ratio: {self.gear_ratio}, bobbin radius: {self.bobbin_radius:.4f} m, max torque: {self.max_torque_nm:.2f} Nm")

        # 刚度参数 (输出侧 -> 转子侧)
        self.output_kp = 15.0 # 26 TODO: find better kp and kd for the arm application
        self.output_kd = 2.5 # 0.6
        if self.is_deploy_motor:
            self.output_kp *= max(1.0, torque_gain_scale)
            self.output_kd *= max(1.0, torque_gain_scale)
        self.a1_special_kp_div = 26.07
        self.a1_special_kd_mul = 100.0
        self.rotor_kp = (self.output_kp / (self.gear_ratio ** 2)) / self.a1_special_kp_div
        self.rotor_kd = (self.output_kd / (self.gear_ratio ** 2)) * self.a1_special_kd_mul

        # 用于记录电机当前状态的变量
        self.__current_shaft_ang_rad   = 0.0
        self.__current_shaft_vel_rad_s = 0.0
        self.__current_tau             = 0.0
        self.__current_temp            = 0
        self.__current_merror          = 0
        self.__startup_shaft_ang_rad   = 0.0
        
        if set_foc_on_init:
            self.__set_foc_mode()
            self.__startup_shaft_ang_rad = self.__current_shaft_ang_rad
            print(f"[Motor {self.motor_id}] Initialized in FOC mode. Startup shaft angle: {self.__startup_shaft_ang_rad:.4f} rad") if self.is_debug else None
        else:
            print(f"[Motor {self.motor_id}] Initialized without setting FOC mode. Current status not initialised.") if self.is_debug else None

    def __set_foc_mode(self):
        """将电机切换到 FOC 闭环伺服模式 (初始化用)。"""
        print(f"[Motor {self.motor_id}] Setting FOC mode...") if self.is_debug else None
        ''' 先以 0 刚度初始化 '''
        self.cmd.q   = 0.0 # rotor 角度
        self.cmd.dq  = 0.0 # rotor 速度
        self.cmd.kp  = 0.0
        self.cmd.kd  = 0.0
        self.cmd.tau = 0.0
        while not self.serial.sendRecv(self.cmd, self.data):
            print(f"[Motor {self.motor_id}] Failed to send/receive FOC mode command, retrying...") if self.is_debug else None
        ''' 每次发送命令后, 读取反馈更新到存储的当前状态 '''
        self.__post_command_update()
        print(f"[Motor {self.motor_id}] FOC mode set successfully.") if self.is_debug else None

    def _send_command_with_retry(self, max_retries: int = 10, retry_delay_s: float = 0.001) -> bool:
        """发送目标命令并在超时时重试，避免通讯失败被静默忽略。"""
        for attempt in range(max_retries):
            if self.serial.sendRecv(self.cmd, self.data):
                self.__post_command_update()
                return True
            if self.is_debug:
                print(f"[Motor {self.motor_id}] Target command failed (attempt {attempt + 1}/{max_retries}), retrying...")
            time.sleep(retry_delay_s)
        if self.is_debug:
            print(f"[Motor {self.motor_id}] Target command failed after {max_retries} attempts; keeping last valid motor state.")
        return False

    def set_target_angle(self, angle_rad: float, vel_rad_s: float = 0.0, tau: float = 0.0, *, retry_on_timeout: bool = False):
        """
        设置目标角度 (弧度)。

        angle_rad: 对外定义的输出侧作动角度。
                   正值表示 tension / 收紧。
        vel_rad_s: 对外定义的输出侧作动角速度。
        tau:       输出侧前馈力矩 Nm。

        注意：
        angle_rad 是相对于启动位置的作动角，而不是电机绝对 shaft angle。
        direction 用于修正 bobbin 安装方向。
        """
        # 对外正方向 -> 实际电机 shaft 正方向
        physical_angle_rad = self.direction * angle_rad
        physical_vel_rad_s = self.direction * vel_rad_s
        physical_tau       = self.direction * tau
        physical_tau       = max(-self.max_torque_nm, min(self.max_torque_nm, physical_tau))

        ''' A1 SDK 的接口是转子侧的角度和速度, 需要进行换算 '''
        # 转子侧换算, 注意加上启动时的初始角度偏移 (startup_shaft_ang_rad), 以保证在开机位姿基础上的相对控制
        cmd_q  = (physical_angle_rad + self.__startup_shaft_ang_rad) * self.gear_ratio
        cmd_dq =  physical_vel_rad_s                                 * self.gear_ratio

        self.cmd.kp   = self.rotor_kp
        self.cmd.kd   = self.rotor_kd
        self.cmd.q    = cmd_q
        self.cmd.dq   = cmd_dq
        self.cmd.tau  = physical_tau

        if retry_on_timeout:
            self._send_command_with_retry(max_retries=10, retry_delay_s=0.001)
        else:
            self.serial.sendRecv(self.cmd, self.data)
            self.__post_command_update()

    def set_target_actuation_length(self, length_m: float, vel_m_s: float = 0.0, tau: float = 0.0, *, retry_on_timeout: bool = False):
        """
        Set target length (m) for the cable, which is the delta L actually.
        A + value corresponds to tension. A - value corresponds to loosen.
        """
        angle_rad = length_m / self.bobbin_radius
        vel_rad_s = vel_m_s  / self.bobbin_radius
        self.set_target_angle(angle_rad, vel_rad_s, tau, retry_on_timeout=retry_on_timeout)

    def __post_command_update(self):
        """每次发送命令后, 读取反馈更新到存储的当前状态 """
        self.__current_shaft_ang_rad   = self.data.q  / self.gear_ratio
        self.__current_shaft_vel_rad_s = self.data.dq / self.gear_ratio
        self.__current_tau             = self.data.tau
        self.__current_temp            = self.data.temp
        self.__current_merror          = self.data.merror

    def get_current_status(self) -> dict:
        """读取电机反馈。"""

        # 原始 shaft 角度变化
        physical_actuation_ang_rad = self.__current_shaft_ang_rad - self.__startup_shaft_ang_rad

        # 转换成对外定义的作动方向
        # 这样 actuate_len > 0 永远表示 tension / 收紧
        actuation_ang_rad = self.direction * physical_actuation_ang_rad
        actuation_len_m   = actuation_ang_rad * self.bobbin_radius

        physical_actuation_vel_rad_s = self.__current_shaft_vel_rad_s
        actuation_vel_rad_s          = self.direction * physical_actuation_vel_rad_s
        actuation_vel_m_s            = actuation_vel_rad_s * self.bobbin_radius

        return {
            'shaft_ang':   self.__current_shaft_ang_rad,
            'shaft_vel':   self.__current_shaft_vel_rad_s,

            'actuate_ang':      actuation_ang_rad,
            'actuate_vel':      actuation_vel_rad_s,
            'actuate_len':      actuation_len_m,
            'actuate_len_vel':  actuation_vel_m_s,

            'tau':         self.__current_tau,
            'temp':        self.__current_temp,
            'merror':      self.__current_merror,
            'id':          self.motor_id,
            'direction':   self.direction
        }


# ================================================================
# 虚拟串口 (用于 dry-run)
# ================================================================

class DummySerialPort:
    """虚拟串口，供 dry-run 测试使用。"""
    def __init__(self, port: str, *, is_debug: bool):
        self.port     = port
        self.is_debug = is_debug

        self.opened = False
        print(f"[DummySerial] Opening serial port {port}") if self.is_debug else None
        self.opened = True

    def sendRecv(self, cmd: MotorCmd, data: MotorData) -> bool:
        data.q      = cmd.q  # 模拟电机能跟上目标位置
        data.dq     = cmd.dq
        data.tau    = cmd.tau
        data.temp   = 40  # 固定温度
        data.merror = 0   # 无错误
        return True

    def close(self):
        if self.opened:
            print("[DummySerial] Closing serial port") if self.is_debug else None
            self.opened = False


# ================================================================
# 张拉整体机械臂控制器
# ================================================================

class TensegrityArmBase:
    """
    张拉整体机械臂控制器。

    机械结构:
      3 根弦控制弯折 (pitch/yaw), 1 根弦控制伸缩 (deploy/retract)
    """

    def __init__(self, serial_port_active: str, serial_port_deploy: str, dry_run: bool = False, *, is_debug: bool,
        active_motor_ids:        Optional[list] =None,
        active_motor_directions: Optional[list] =None,
        deploy_motor_id:         Optional[int]  =None,
        deploy_motor_direction:  Optional[int]  =None,
        deploy_delta_length: float = 0.05,
        deploy_motor_torque_gain_scale: float = 3.0,
    ):
        self.serial_instance_active = None
        self.serial_instance_deploy = None

        self.dry_run = dry_run
        self.is_debug = is_debug
        self.deploy_motor_torque_gain_scale = max(1.0, float(deploy_motor_torque_gain_scale))

        active_motor_ids        = active_motor_ids        or [1, 0, 2]
        active_motor_directions = active_motor_directions or [1, 1, -1]
        deploy_motor_id         = deploy_motor_id         or 2
        deploy_motor_direction  = deploy_motor_direction  or -1

        if not self.dry_run:
            try:
                self.serial_instance_active = SerialPort(serial_port_active)
            except Exception as e:
                print(f"[Arm] Error occurred while initialising active serial ports, error message: \"{e}\".")
                raise
            try:
                self.serial_instance_deploy = SerialPort(serial_port_deploy)
            except Exception as e:
                print(f"[Arm] Error occurred while initialising deploy serial ports, error message: \"{e}\".")
                raise
        else:
            self.serial_instance_active = DummySerialPort(serial_port_active, is_debug=self.is_debug)
            self.serial_instance_deploy = DummySerialPort(serial_port_deploy, is_debug=self.is_debug)

        motor_0 = A1Motor(active_motor_ids[0], self.serial_instance_active, set_foc_on_init=True, direction=active_motor_directions[0], is_debug=self.is_debug)
        motor_1 = A1Motor(active_motor_ids[1], self.serial_instance_active, set_foc_on_init=True, direction=active_motor_directions[1], is_debug=self.is_debug)
        motor_2 = A1Motor(active_motor_ids[2], self.serial_instance_active, set_foc_on_init=True, direction=active_motor_directions[2], is_debug=self.is_debug)
        motor_3 = A1Motor(deploy_motor_id,     self.serial_instance_deploy, set_foc_on_init=True, direction=deploy_motor_direction,     is_debug=self.is_debug, is_deploy_motor=True, torque_gain_scale=self.deploy_motor_torque_gain_scale)
        self.motors = [motor_0, motor_1, motor_2, motor_3]

        self.final_target_actuation_lengths = [0.0] * 4
        self.last_commanded_lengths         = [0.0] * 4
        self.deployed                       = False
        self.deploy_delta_length            = deploy_delta_length

    def _apply_single_motor_delta(self, index: int, delta_m: float) -> float:
        """对单个执行器施加一个小的长度增量。"""
        if index < 0 or index >= len(self.motors):
            raise ValueError(f"Motor index out of range: {index}")

        current = self.motors[index].get_current_status()['actuate_len']
        target = current + float(delta_m)
        self.final_target_actuation_lengths[index] = target
        self.last_commanded_lengths[index] = target
        self.motors[index].set_target_actuation_length(target, retry_on_timeout=True)
        return target

    def apply_active_micro_adjust(self, delta_lengths_m: list[float]) -> list[float]:
        """对 3 个主动弦施加微调增量，单位为米。"""
        if len(delta_lengths_m) != 3:
            raise ValueError(f"Expected 3 deltas for active motors, received {len(delta_lengths_m)}")
        adjusted = []
        for idx, delta in enumerate(delta_lengths_m):
            adjusted.append(self._apply_single_motor_delta(idx, float(delta)))
        return adjusted

    def apply_deploy_micro_adjust(self, delta_length_m: float) -> float:
        """对 deploy 弦施加微调增量，单位为米。"""
        current = self.motors[3].get_current_status()['actuate_len']
        target = current + float(delta_length_m)
        self.final_target_actuation_lengths[3] = target
        self.last_commanded_lengths[3] = target
        self.motors[3].set_target_actuation_length(target, retry_on_timeout=True)
        self.deployed = self.deployed or (target > 0.0)
        return target

    def apply_micro_adjust(self, delta_lengths_m: list[float]) -> list[float]:
        """接受 3 个或 4 个执行器增量，分别微调 active/deploy 弦。"""
        if len(delta_lengths_m) == 3:
            return self.apply_active_micro_adjust(delta_lengths_m)
        if len(delta_lengths_m) == 4:
            adjusted = []
            for idx, delta in enumerate(delta_lengths_m):
                adjusted.append(self._apply_single_motor_delta(idx, float(delta)))
            return adjusted
        raise ValueError(f"Expected 3 or 4 deltas, received {len(delta_lengths_m)}")

    def initialize(self) -> bool:
        """初始化: 切换所有电机到 FOC 模式。"""
        for motor in self.motors:
            pass
        time.sleep(0.5)
        return True

    def set_deploy(self, deploy: bool):
        """设置展开/收起状态。"""
        if self.deployed == deploy:
            print(f"[Arm] Already {'deployed' if deploy else 'retracted'}. No action taken.") if self.is_debug else None
            return

        ''' 设置伸缩弦的目标长度 '''
        self.final_target_actuation_lengths[3] = self.deploy_delta_length if deploy else 0.0

        ''' 以当前长度为起点, 平滑地过渡到目标长度 '''
        temp_l = self.motors[3].get_current_status()['actuate_len']
        for _ in range(100000):
            temp_l += 0.001 * math.tanh(10.0 * (self.final_target_actuation_lengths[3] - temp_l))
            self.motors[3].set_target_actuation_length(temp_l)
            if abs(temp_l - self.final_target_actuation_lengths[3]) < 0.001:
                break
            print(f"[Arm] Deploying... final target: {self.final_target_actuation_lengths[3]:.3f} m, current step: {temp_l:.3f} m") if self.is_debug else None
        self.motors[3].set_target_actuation_length(self.final_target_actuation_lengths[3])  # 确保最终达到目标长度
        self.deployed = deploy
        self.last_commanded_lengths[3] = self.final_target_actuation_lengths[3]
        status = "Deployed" if deploy else "Retracted"
        print(f"[Arm] Status: {status}") if self.is_debug else None

    def close(self):
        """关闭所有电机和串口。"""
        for serial in (self.serial_instance_active, self.serial_instance_deploy):
            if hasattr(serial, 'close') and serial is not None:
                try:
                    serial.close()
                except Exception:
                    pass


class TensegrityArm(TensegrityArmBase):
    """张拉整体机械臂控制器 - 具体实现类 (可扩展更多功能)"""
    def __init__(self, serial_port_active: str, serial_port_deploy: str, dry_run: bool = False, *, is_debug: bool,
        active_motor_ids:        Optional[list[int]] = None,
        active_motor_directions: Optional[list[int]] = None,
        deploy_motor_id:         Optional[int]       = None,
        deploy_motor_direction:  Optional[int]       = None,
        deploy_delta_length: float = 0.05,
        deploy_motor_torque_gain_scale: float = 3.0,
    ):
        super().__init__(
            serial_port_active     =serial_port_active,
            serial_port_deploy     =serial_port_deploy,
            dry_run                =dry_run,
            is_debug               =is_debug,
            active_motor_ids       =active_motor_ids,
            active_motor_directions=active_motor_directions,
            deploy_motor_id        =deploy_motor_id,
            deploy_motor_direction =deploy_motor_direction,
            deploy_delta_length    =deploy_delta_length,
            deploy_motor_torque_gain_scale=deploy_motor_torque_gain_scale,
        )
        self.kinematics = TensegrityArmKinematics(is_debug=is_debug)

    def update_bend(self, theta: float, phi: float):
        """设置弯折角度 (theta: pitch, phi: yaw)。"""
        if not self.deployed:
            print("[Arm] Please deploy the arm before setting bend angles.") if self.is_debug else None
            return False

        l1, l2, l3 = self.kinematics.calculate_actuation_lengths_for_bend(theta, phi)
        self.final_target_actuation_lengths = [l1, l2, l3, self.final_target_actuation_lengths[3]]  # 保持伸缩弦长度不变
        return True

    def step(self):
        """ 根据最终长度目标, 使用tanh函数平滑控制命令, 并打印状态 """
        if not self.deployed:
            print("[Arm] Please deploy the arm before stepping.") if self.is_debug else None
            return False
        
        ''' 根据目前长度与目标长度的误差, 使用tanh函数平滑控制命令 '''
        for i in range(3):
            error = self.final_target_actuation_lengths[i] - self.last_commanded_lengths[i]
            k = 50.0           # 增大k可以让控制更接近于阶跃函数
            max_step = 0.002   # 每周期最多改 2 mm
            control_signal = max_step * math.tanh(k * error)
            next_length = self.last_commanded_lengths[i] + control_signal
            self.motors[i].set_target_actuation_length(next_length)
            self.last_commanded_lengths[i] = next_length


        if self.is_debug:
            feedbacks = [motor.get_current_status() for motor in self.motors]
            if not hasattr(self, '_print_counter'):
                self._print_counter = 0
            self._print_counter += 1
            if self._print_counter % 1 == 0:
                pos_degs = [f'{fb["shaft_ang"]*180/math.pi:.2f}°' for fb in feedbacks]
                act_degs = [f'{fb["actuate_ang"]*180/math.pi:.2f}°' for fb in feedbacks]
                lengths  = [f'{fb["actuate_len"]*1000.0:.4f}' for fb in feedbacks]
                temps    = [str(fb["temp"]) for fb in feedbacks]
                print(
                    f"[Arm] "
                    # f"shaft angle: {pos_degs} | "
                    f"actuation angle: {act_degs} | "
                    f"delta L: {lengths} mm | "
                    # f"temperature: {temps}°C"
                )

        total_error = sum(abs(self.final_target_actuation_lengths[i] - self.last_commanded_lengths[i]) for i in range(3))
        if total_error < 0.003:
            print(f"[Arm] Reached target bend angles within tolerance. Total length error: {total_error:.4f} m") if self.is_debug else None
            for i in range(3):
                self.motors[i].set_target_actuation_length(self.final_target_actuation_lengths[i])  # 确保最终达到目标长度
                self.last_commanded_lengths[i] = self.final_target_actuation_lengths[i]
            return True

        return False

    def get_feedbacks(self):
        return [motor.get_current_status() for motor in self.motors]


# ================================================================
# 张拉整体机械臂运动学
# ================================================================

class TensegrityArmKinematics:

    def __init__(self, *, is_debug: bool) -> None:
        self.is_debug = is_debug
        
        self.stage_1_edge_length = 190.526/1000.0
        self.stage_2_edge_length = 190.526/1000.0
        self.stage_3_edge_length = 190.526/1000.0
        self.stage_4_edge_length = 190.526/1000.0

        self.stage_12_neutral_include_angle = 27.936 * math.pi / 180.0
        self.stage_23_neutral_include_angle = 27.936 * math.pi / 180.0
        self.stage_34_neutral_include_angle = 27.936 * math.pi / 180.0
        
        self.stage_1_saddle_length = 123.236/1000.0
        self.stage_2_saddle_length = 123.236/1000.0
        self.stage_3_saddle_length = 123.236/1000.0
        self.stage_4_saddle_length = 123.236/1000.0

        ''' 三根绳间隔 120°, 单位向量 '''
        self.phi_cable_1 = 0
        self.phi_cable_2 = 2 * math.pi / 3
        self.phi_cable_3 = 4 * math.pi / 3
        self.cable_1_unit_vector = [math.cos(self.phi_cable_1), math.sin(self.phi_cable_1)]
        self.cable_2_unit_vector = [math.cos(self.phi_cable_2), math.sin(self.phi_cable_2)]
        self.cable_3_unit_vector = [math.cos(self.phi_cable_3), math.sin(self.phi_cable_3)]

        self.max_bending_angle = 60 / 180.0 * math.pi  # 最大弯折角度 (rad)

        self.neutral_total_length = self.calculate_total_cable_length(0.0)
        
        if self.is_debug:
            print("-" * 60)
            print(f"[Kinematics] 初始化中性位弦长...")
            print(f"[Kinematics] 中性位总弦长: {self.neutral_total_length:.4f} m")
            print(f"[Kinematics] 运动学后端已就绪")
            print("-" * 60)

    def calculate_component_bending_angle(self, theta: float, phi: float, component_phi: float) -> float:
        component_theta = theta * math.cos(phi - component_phi)
        return component_theta

    def calculate_segment_length(self, edge_length: float, saddle_length: float, stage_theta: float, neutral_angle: float) -> float:
        L = math.sqrt(edge_length**2 + saddle_length**2 - 2 * edge_length * saddle_length * math.cos(neutral_angle - stage_theta))
        return L

    def calculate_total_cable_length(self, component_theta: float) -> float:
        stage_theta = component_theta / 3.0
        total_length = 0.0
        stage_edge_lengths = [self.stage_2_edge_length,
                              self.stage_3_edge_length,
                              self.stage_4_edge_length]
        stage_saddle_lengths = [self.stage_1_saddle_length,
                                self.stage_2_saddle_length,
                                self.stage_3_saddle_length]
        neutral_included_angles = [self.stage_12_neutral_include_angle,
                                   self.stage_23_neutral_include_angle,
                                   self.stage_34_neutral_include_angle]
        for edge_length, saddle_length, neutral_angle in zip(stage_edge_lengths, stage_saddle_lengths, neutral_included_angles):
            segment_length = self.calculate_segment_length(edge_length, saddle_length, stage_theta, neutral_angle)
            total_length += segment_length
        total_length = total_length + (155.564/1000.0)  # 加上第一段固定长度
        print(f"[Kinematics] 计算得到分弦弯折角度: {component_theta:.3f} rad, 对应分弦总弦长: {total_length:.4f} m") if self.is_debug else None
        return total_length

    def calculate_active_cable_lengths(self, theta: float, phi: float) -> list:
        """
        根据给定的弯折角度 (theta, phi)，计算 3 条弯折弦的目标长度。

        theta: 弯折角度 (pitch) - 范围 [0, π/6]
        phi:   旋转角度 (yaw)   - 范围 [0, 2π]
        """

        theta = max(0, min(theta, self.max_bending_angle))
        phi = phi % (2 * math.pi)
        if phi < 0:
            phi += 2 * math.pi
        print(f"[Kinematics] 限制得到弯折角度: theta={theta:.3f} rad, phi={phi:.3f} rad") if self.is_debug else None

        component_theta_1 = self.calculate_component_bending_angle(theta, phi, self.phi_cable_1)
        component_theta_2 = self.calculate_component_bending_angle(theta, phi, self.phi_cable_2)
        component_theta_3 = self.calculate_component_bending_angle(theta, phi, self.phi_cable_3)

        l1 = self.calculate_total_cable_length(component_theta_1)
        l2 = self.calculate_total_cable_length(component_theta_2)
        l3 = self.calculate_total_cable_length(component_theta_3)

        print(f"[Kinematics] 计算得到目标弦长: l1={l1:.4f} m, l2={l2:.4f} m, l3={l3:.4f} m") if self.is_debug else None

        return [l1, l2, l3]

    def calculate_actuation_lengths_for_bend(self, theta: float, phi: float) -> list:
        """计算相对于中性位的弦长变化量 (deltaL)。"""
        target_lengths = self.calculate_active_cable_lengths(theta, phi)
        delta_lengths = [target_lengths[i] - self.neutral_total_length for i in range(3)]
        print(f"[Kinematics] 计算得到弦长变化量: deltaL1={delta_lengths[0]:.4f} m, deltaL2={delta_lengths[1]:.4f} m, deltaL3={delta_lengths[2]:.4f} m") if self.is_debug else None
        return delta_lengths
