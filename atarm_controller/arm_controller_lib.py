#!/usr/bin/env python3
"""
Tensegrity Arm Controller - 基于宇树 A1 电机和 SDK 的张拉整体机械臂控制程序
"""
from __future__ import annotations

import time
import math
from dataclasses import asdict,replace
from typing import Optional
from .serial_adapter import (MotorCmd, MotorData, MotorType, MotorMode,
                             queryMotorMode, queryGearRatio, FeedbackSample)
from .motion_checks import MotionGuardConfig, MotionEvidenceGuard
from .camera_feedback import CameraCompletionGate
from .visual_servo import BoundedVisualServo
from .coupled_bend import CoupledBendServo
from .camera_cycle import CameraArmCycle
from .camera_recovery import CameraRecovery
from .recovery_probe import RecoveryMotionProbe
from .progress_window import GoalProgressWindow,WindowProgressConfig,PauseBudgetConfig


class ControlError(RuntimeError):
    """A command cannot be confirmed from valid motor feedback."""


# ================================================================
# 电机封装
# ================================================================
class A1Motor:
    """封装单个 A1 电机的控制逻辑。"""
    def __init__(self, motor_id: int, serial_ptr, set_foc_on_init: bool, direction: int, *, is_debug: bool, is_deploy_motor: bool = False, torque_gain_scale: float = 1.0, clock=None, feedback_max_age_s: float = 0.1, motion_guard_config=None):
        self.motor_id         = motor_id
        self.serial           = serial_ptr
        self.is_debug         = is_debug
        self.is_deploy_motor  = is_deploy_motor
        self._clock = clock or time.monotonic
        self.feedback_max_age_s = feedback_max_age_s
        self._feedback_valid = False
        self._transport_frame_id = None
        self._freshness_basis = None
        self._feedback_received_at = None
        self._feedback_received_ns = None
        self.feedback_error = 'No verified feedback received.'
        self.motion_guard=MotionEvidenceGuard(motion_guard_config or MotionGuardConfig())

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
        if not self._send_command_with_retry(max_retries=10):
            raise ControlError(f'Motor {self.motor_id}: {self.feedback_error}')
        print(f"[Motor {self.motor_id}] FOC mode set successfully.") if self.is_debug else None

    def _send_command_with_retry(self, max_retries: int = 10, retry_delay_s: float = 0.001) -> bool:
        """发送目标命令并在超时时重试，避免通讯失败被静默忽略。"""
        for attempt in range(max_retries):
            if self.refresh_feedback():
                return True
            if self.motion_guard.fault is not None:
                return False
            if self.is_debug:
                print(f"[Motor {self.motor_id}] Target command failed (attempt {attempt + 1}/{max_retries}), retrying...")
            time.sleep(retry_delay_s)
        if self.is_debug:
            print(f"[Motor {self.motor_id}] Target command failed after {max_retries} attempts; keeping last valid motor state.")
        return False

    def refresh_feedback(self, *, rebaseline=False) -> bool:
        """Consume one immutable validated receive event, never a cached object.

        transport_frame_id counts actual host receive events, not motor samples.
        Device sampling freshness is not proven by the A1 wire protocol.
        """
        self._feedback_valid = False
        if self.motion_guard.fault is not None and not rebaseline:
            self.data.correct=False
            self.feedback_error=self.motion_guard.fault['code']+': latched; explicit reference recovery required.'
            return False
        try:
            sample = self.serial.exchange(self.cmd)
            if not sample.valid:
                raise ControlError(f'Serial receive failed: {sample.status}; cached data is invalid.')
            allowed_basis = ('HOST_VALIDATED_SERIAL_RX',)
            if isinstance(self.serial, DummySerialPort):
                allowed_basis += ('SIMULATED_RX',)
            if sample.freshness_basis not in allowed_basis or not sample.crc_ok or sample.actual_rx_length != 78:
                raise ControlError('Receive evidence is incomplete or has an unsupported basis.')
            if sample.motor_id != self.motor_id or sample.mode != self.cmd.mode:
                raise ControlError('Receive motor ID or mode does not match the command.')
            frame_id = sample.transport_frame_id
            if isinstance(frame_id, bool) or not isinstance(frame_id, int) or frame_id <= 0:
                raise ControlError('Valid transport receive event ID is unavailable.')
            if self._transport_frame_id is not None and frame_id <= self._transport_frame_id:
                raise ControlError('Repeated or out-of-order receive event; old frame cache rejected.')
            if not (0 < sample.tx_ns <= sample.rx_ns):
                raise ControlError('Receive event timestamps are invalid.')
            age = self._clock() - sample.rx_ns / 1e9
            if not -1e-6 <= age <= self.feedback_max_age_s:
                raise ControlError('Receive event is expired or from an incompatible clock.')
            for field in ('q', 'dq', 'tau', 'temp', 'merror'):
                if not math.isfinite(float(getattr(sample, field))):
                    raise ControlError(f'Nonfinite motor feedback: {field}.')
            if sample.merror != 0:
                raise ControlError(f'Motor error code {sample.merror}.')
            # Absolute encoder-derived travel keeps guard history valid when a
            # relative actuation reference is explicitly recalibrated.
            measured=self.direction*sample.q/self.gear_ratio*self.bobbin_radius
            target=(self.direction*self.cmd.q/self.gear_ratio*self.bobbin_radius
                    if self.cmd.kp>0 else None)
            self.motion_guard.observe(measured,sample.rx_ns,target,rebaseline=rebaseline)
            for field in ('q', 'dq', 'tau', 'temp', 'merror', 'motor_id', 'mode'):
                setattr(self.data, field, getattr(sample, field))
            self.data.hex_len = 78
            self.data.correct = True
            self.__post_command_update()
            self._transport_frame_id = frame_id
            self._freshness_basis = sample.freshness_basis
            self._feedback_received_at = sample.rx_ns / 1e9
            self._feedback_received_ns = sample.rx_ns
            self._feedback_valid = True
            self.feedback_error = ''
            return True
        except Exception as exc:
            self.data.correct = False
            self.feedback_error = str(exc)
            return False

    def confirm_current_reference(self):
        """Called only after a separately confirmed mechanical home."""
        if not self.get_current_status()['feedback_valid']:
            raise ControlError('Cannot reference an invalid or expired measurement.')
        self.__startup_shaft_ang_rad = self.__current_shaft_ang_rad

    def read_passive_feedback(self, *, rebaseline=False) -> bool:
        """Reference acquisition must not reissue an earlier position target."""
        self.cmd.kp = self.cmd.kd = self.cmd.tau = self.cmd.dq = 0.0
        return self.refresh_feedback(rebaseline=rebaseline)

    def rebind_passive_transport(self,serial):
        """Explicit new receive generation, without FOC initialization/motion."""
        self.cmd.kp=self.cmd.kd=self.cmd.dq=self.cmd.tau=0.0
        self.serial=serial;self._transport_frame_id=None
        self._feedback_valid=False;self.data.correct=False

    def request_zero_effort_stop(self):
        """One bounded cancellation exchange, including when motion checks fault.

        Sends only kp=kd=dq=tau=0, never a position correction. A valid reply is
        evidence of communication, not proof of braking or physical safety.
        Trusted normal-feedback history is not replaced by this stop receipt.
        """
        self.cmd.kp=self.cmd.kd=self.cmd.dq=self.cmd.tau=0.0
        result={'motor_id':self.motor_id,'requested':'ZERO_EFFORT','reply_valid':False}
        try:
            sample=self.serial.exchange(self.cmd)
            result.update({'reply_valid':bool(sample.valid),'native_status':sample.status,
                'transport_frame_id':sample.transport_frame_id})
        except Exception as exc:
            result['error']=str(exc)
        return result

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
            return self._send_command_with_retry(max_retries=10, retry_delay_s=0.001)
        else:
            return self.refresh_feedback()

    def set_target_actuation_length(self, length_m: float, vel_m_s: float = 0.0, tau: float = 0.0, *, retry_on_timeout: bool = False):
        """
        Set target length (m) for the cable, which is the delta L actually.
        A + value corresponds to tension. A - value corresponds to loosen.
        """
        angle_rad = length_m / self.bobbin_radius
        vel_rad_s = vel_m_s  / self.bobbin_radius
        return self.set_target_angle(angle_rad, vel_rad_s, tau, retry_on_timeout=retry_on_timeout)

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

        age = None if self._feedback_received_at is None else self._clock() - self._feedback_received_at
        valid = self._feedback_valid and age is not None and 0.0 <= age <= self.feedback_max_age_s
        return {
            'feedback_valid': valid,
            'transport_frame_id': self._transport_frame_id,
            'freshness_basis': self._freshness_basis,
            'feedback_sequence': None,
            'rx_monotonic_ns': self._feedback_received_ns,
            'device_sample_time': None,
            'motion_guard': self.motion_guard.status(),
            'feedback_age_s': age,
            'feedback_error': self.feedback_error if self.feedback_error else ('' if valid else 'Feedback expired.'),
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
    def __init__(self, port: str, *, is_debug: bool, clock=None):
        self.port     = port
        self.is_debug = is_debug

        self.opened = False
        print(f"[DummySerial] Opening serial port {port}") if self.is_debug else None
        self.opened = True
        self._sequences = {}
        self._clock = clock or time.monotonic

    def exchange(self, cmd: MotorCmd):
        self._sequences[cmd.id] = self._sequences.get(cmd.id, 0) + 1
        now = int(self._clock() * 1e9)
        return FeedbackSample(True, 'SIMULATED', self._sequences[cmd.id], now, now,
            78, 0, cmd.id, cmd.mode, 40, 0, cmd.q, cmd.dq, cmd.tau,
            crc_ok=True, freshness_basis='SIMULATED_RX')

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
        deploy_motor_torque_gain_scale: Optional[float] = None,
        clock=None, sleeper=None,
        deploy_timeout_s: float = 10.0,
        deploy_period_s: float = 0.02,
        deploy_speed_m_s: float = 0.01,
        deploy_settle_s: float = 0.1,
        deploy_tolerance_m: float = 0.001,
        feedback_max_age_s: float = 0.1,
        serial_factory=None,
        feedback_max_speed_m_s: float = 1.0,
        feedback_step_margin_m: float = 0.0005,
        uncommanded_tolerance_m: float = 0.0015,
        progress_min_m: float = 0.0005,
        progress_timeout_s: float = 1.0,
        camera_required: bool = True,
        camera_feedback=None,
        camera_goals=None,
        camera_config=None,
        visual_servo_config=None,
        bend_servo_config=None,
        bend_camera_config=None,
        cycle_config=None,
        recovery_config=None,
        probe_config=None,
        window_progress_config=None,
        pause_budget_config=None,
    ):
        self.serial_instance_active = None
        self.serial_instance_deploy = None

        self.dry_run = dry_run
        self.is_debug = is_debug
        self._clock = clock or time.monotonic
        self._sleep = sleeper or time.sleep
        self.camera_required=bool(camera_required)
        self.camera_feedback=camera_feedback
        self.camera_goals=dict(camera_goals or {})
        self.camera_gate=CameraCompletionGate(camera_config)
        self._base_camera_config=self.camera_gate.config
        if pause_budget_config is not None and not isinstance(pause_budget_config,PauseBudgetConfig):
            raise ValueError('Explicit PauseBudgetConfig required')
        self.pause_budget=pause_budget_config
        self.pause_budget_status={'enabled':False}
        if self.pause_budget is not None:
            window_progress_config=self.pause_budget.window(window_progress_config)
        if visual_servo_config is not None and not self.camera_required:
            raise ValueError('Visual correction requires independent camera feedback')
        self.visual_servo=(None if visual_servo_config is None else
            BoundedVisualServo(visual_servo_config,self.camera_gate.config.position_tolerance_m))
        self.visual_endpoint_lengths={}
        self._visual_motion_active=False
        if bend_servo_config is not None and not self.camera_required:
            raise ValueError('Coupled bend control requires independent camera feedback')
        self.bend_controller=None if bend_servo_config is None else CoupledBendServo(bend_servo_config,bend_camera_config or camera_config,window_progress_config,pause_budget_config)
        if window_progress_config is not None and not self.camera_required:
            raise ValueError('Short progress windows require independent camera feedback')
        self.deploy_progress_window=None if window_progress_config is None else GoalProgressWindow(window_progress_config)
        self._bend_motion_active=False
        self.cycle=CameraArmCycle(cycle_config)
        self._cycle_active=False
        self.recovery=CameraRecovery(recovery_config)
        self._recovery_active=False;self._serial_generation=0
        self.motion_probe=None if probe_config is None else RecoveryMotionProbe(probe_config)
        if deploy_motor_direction not in (1, -1):
            raise ValueError('Set deploy_motor_direction explicitly to the verified installation sign, +1 or -1.')
        if deploy_motor_torque_gain_scale is None or not math.isfinite(float(deploy_motor_torque_gain_scale)) or deploy_motor_torque_gain_scale < 1.0:
            raise ValueError('Set deploy_motor_torque_gain_scale explicitly to a verified finite value >= 1.')
        self.deploy_motor_torque_gain_scale = float(deploy_motor_torque_gain_scale)
        settings = {'deploy_timeout_s': deploy_timeout_s, 'deploy_period_s': deploy_period_s,
                    'deploy_speed_m_s': deploy_speed_m_s, 'deploy_settle_s': deploy_settle_s,
                    'deploy_tolerance_m': deploy_tolerance_m, 'feedback_max_age_s': feedback_max_age_s,
                    'progress_min_m':progress_min_m,'progress_timeout_s':progress_timeout_s}
        for name, value in settings.items():
            if not math.isfinite(float(value)) or value <= 0:
                raise ValueError(f'{name} must be finite and positive.')
            setattr(self, name, float(value))
        motion_limits=MotionGuardConfig(feedback_max_speed_m_s,feedback_step_margin_m,uncommanded_tolerance_m)
        if not math.isfinite(float(deploy_delta_length)) or deploy_delta_length <= 0:
            raise ValueError('deploy_delta_length must be finite and positive.')

        active_motor_ids        = active_motor_ids        or [1, 0, 2]
        active_motor_directions = active_motor_directions or [1, 1, -1]
        deploy_motor_id         = 2 if deploy_motor_id is None else deploy_motor_id

        if not self.dry_run:
            if serial_factory is None:
                raise ControlError('Explicit validated FD transport factory required; automatic device opening is disabled.')
            try:
                self.serial_instance_active = serial_factory(serial_port_active)
            except Exception as e:
                print(f"[Arm] Error occurred while initialising active serial ports, error message: \"{e}\".")
                raise
            try:
                self.serial_instance_deploy = serial_factory(serial_port_deploy)
            except Exception as e:
                print(f"[Arm] Error occurred while initialising deploy serial ports, error message: \"{e}\".")
                self.close()
                raise
        else:
            self.serial_instance_active = DummySerialPort(serial_port_active, is_debug=self.is_debug, clock=self._clock)
            self.serial_instance_deploy = DummySerialPort(serial_port_deploy, is_debug=self.is_debug, clock=self._clock)

        try:
            common = {'set_foc_on_init': True, 'is_debug': self.is_debug, 'clock': self._clock,
                      'feedback_max_age_s': self.feedback_max_age_s,'motion_guard_config':motion_limits}
            self.motors = [A1Motor(active_motor_ids[i], self.serial_instance_active,
                                  direction=active_motor_directions[i], **common) for i in range(3)]
            self.motors.append(A1Motor(deploy_motor_id, self.serial_instance_deploy,
                direction=deploy_motor_direction, is_deploy_motor=True,
                torque_gain_scale=self.deploy_motor_torque_gain_scale, **common))
        except Exception:
            self.close()
            raise

        self.final_target_actuation_lengths = [0.0] * 4
        self.last_commanded_lengths         = [0.0] * 4
        self.deployed                       = None
        self.deploy_state                   = 'UNKNOWN'
        self._reference_known               = False
        self.last_motion_result = {'success': False, 'complete': False, 'state': 'UNKNOWN',
                                   'message': 'Mechanical reference is unknown after startup.'}
        self.deploy_delta_length            = deploy_delta_length
        self.motion_progress=None

    def _result(self, success: bool, message: str):
        self.last_motion_result = {'success': success, 'complete': success,
                                   'state': self.deploy_state, 'message': message}
        return dict(self.last_motion_result)

    def _fault(self, message: str):
        if self.deploy_progress_window is not None: self.deploy_progress_window.finish(False,message)
        if self.bend_controller is not None and self.bend_controller.progress_window is not None:
            self.bend_controller.progress_window.finish(False,message)
        self.deploy_state = 'FAULT'
        self.deployed = None
        self._reference_known=False
        if self.motion_probe is not None: self.motion_probe.invalidate(message)
        if self.camera_required: self.recovery.mark_fault(message)
        receipts=None
        if (self._visual_motion_active or self._bend_motion_active or (self._cycle_active and not self.cycle.stop_requested)
            or (self._recovery_active and not self.recovery.stop_requested)):
            receipts=[dict(self.motors[i].request_zero_effort_stop(),actuator_index=i) for i in (3,0,1,2)]
        if self._cycle_active and receipts is not None:
            self.cycle.stop_requested=True
            self.cycle.stop_receipts=receipts
        if self._recovery_active and receipts is not None:
            self.recovery.stop_requested=True;self.recovery.stop_receipts=receipts
            if self.motion_probe is not None: self.motion_probe.stop_receipts=receipts
        if self._visual_motion_active:
            self.visual_servo.active=False
            self.visual_servo.fault=self.visual_servo.fault or message
            self.visual_servo.stop_requested=True
            self.visual_servo.stop_receipts=receipts
            self._visual_motion_active=False
        if self._bend_motion_active:
            self.bend_controller.state='FAULT'
            self.bend_controller.stop_requested=True
            self.bend_controller.stop_receipts=receipts
            self._bend_motion_active=False
        return self._result(False, message)

    def bend_status(self):
        return {'enabled':False} if self.bend_controller is None else self.bend_controller.status()

    def cycle_status(self):
        return self.cycle.status()

    def run_camera_cycle(self,plan):
        return self.cycle.run(self,plan)

    def cancel_camera_cycle(self):
        return self.cycle.request_cancel()

    def recovery_status(self): return self.recovery.status()

    def probe_status(self): return {'enabled':False} if self.motion_probe is None else self.motion_probe.status()

    def recover_camera_cycle(self,**confirmation): return self.recovery.run(self,**confirmation)

    def cancel_camera_recovery(self): return self.recovery.cancel()

    def _cycle_checkpoint(self):
        self.cycle.checkpoint(self)

    def move_bend_camera(self,goal,*,_cycle_token=None):
        if self._recovery_active: return {'success':False,'complete':False,'message':'RECOVERY_OWNS_MOTION'}
        if self._cycle_active and _cycle_token is not self.cycle.owner:
            return {'success':False,'complete':False,'message':'CYCLE_OWNS_MOTION'}
        if self.bend_controller is None:
            return {'success':False,'complete':False,'message':'BEND_CAMERA_CONTROL_NOT_CONFIGURED'}
        return self.bend_controller.move(self,goal)

    def visual_servo_status(self):
        status={'enabled':False} if self.visual_servo is None else self.visual_servo.status()
        status['short_window_progress']={'enabled':False} if self.deploy_progress_window is None else self.deploy_progress_window.status()
        status['pause_budget']=dict(self.pause_budget_status)
        return status

    def camera_status(self):
        if not self.camera_required:
            return {'enabled':False,'ready':False,'completion_basis':'MOTOR_ONLY'}
        status=self.camera_gate.status()
        status['completion_basis']='MOTOR_AND_INDEPENDENT_CAMERA'
        return status

    def _read_camera(self, deploy=None):
        if not self.camera_required:
            return True
        if not callable(self.camera_feedback):
            self.camera_gate._fail('CAMERA_FEEDBACK_MISSING')
        try:
            sample=self.camera_feedback()  # Nonblocking latest snapshot with source capture time.
        except Exception as exc:
            self.camera_gate._fail('CAMERA_PROVIDER_ERROR: '+str(exc))
        now=self._clock()
        if deploy is not None:
            return self.camera_gate.begin(self.camera_goals.get(bool(deploy)),sample,now)
        return self.camera_gate.observe(sample,now)

    def _refresh_all(self) -> bool:
        results = [motor.refresh_feedback() for motor in self.motors]
        return all(results)

    def _feedback_failure_message(self):
        return '; '.join(f'Motor {m.motor_id}: {m.get_current_status()["feedback_error"]}'
                         for m in self.motors if not m.get_current_status()['feedback_valid'])

    def confirm_retracted_reference(self, *, mechanically_confirmed: bool = False):
        """Operator/calibration assertion of a physically retracted mechanism.

        This is not an automatic homing routine. A startup angle, encoder zero,
        or a saved Boolean is insufficient evidence for this assertion.
        """
        if self._recovery_active:
            return {'success':False,'complete':False,'message':'RECOVERY_OWNS_MOTION'}
        if self.camera_required and self.deploy_state=='FAULT':
            return {'success':False,'complete':False,'message':'RECOVERY_API_REQUIRED'}
        if self._cycle_active:
            return self._result(False,'CYCLE_OWNS_MOTION')
        if self._bend_motion_active:
            return self._result(False,'BEND_MOTION_BUSY')
        if not mechanically_confirmed:
            return self._result(False, 'Physical retracted reference must be independently confirmed.')
        passive_ok = [motor.read_passive_feedback(rebaseline=True) for motor in self.motors]
        if not all(passive_ok):
            return self._fault(self._feedback_failure_message())
        return self._commit_retracted_reference()

    def _commit_retracted_reference(self):
        """Internal commit after the caller's reference checks have all passed."""
        self.motors[3].confirm_current_reference()
        # Discard pre-recovery bend targets without inventing new bend zeros.
        for index in range(3):
            measured = self.motors[index].get_current_status()['actuate_len']
            self.final_target_actuation_lengths[index] = measured
            self.last_commanded_lengths[index] = measured
        self.final_target_actuation_lengths[3] = 0.0
        self.last_commanded_lengths[3] = 0.0
        self._reference_known = True
        self.deployed = False
        self.deploy_state = 'RETRACTED'
        self.motion_progress=None
        self.camera_gate.reset()
        self.visual_endpoint_lengths={}
        self._visual_motion_active=False
        if self.visual_servo is not None:
            self.visual_servo.reset()
        if self.bend_controller is not None:
            self.bend_controller.reset()
        self.cycle.reset()
        if self.deploy_progress_window is not None: self.deploy_progress_window.reset()
        return self._result(True, 'Retracted reference confirmed; this did not perform a homing motion.')

    def _apply_single_motor_delta(self, index: int, delta_m: float) -> float:
        """对单个执行器施加一个小的长度增量。"""
        if self.bend_controller is not None:
            raise ControlError('BEND_DIRECT_ADJUST_DISABLED: use the independent camera target interface')
        if index < 0 or index >= len(self.motors):
            raise ValueError(f"Motor index out of range: {index}")
        if not self._reference_known or self.deploy_state == 'FAULT':
            raise ControlError('Micro-adjust rejected: reference unknown or controller faulted.')
        if not math.isfinite(float(delta_m)):
            raise ValueError('Micro-adjust must be finite.')
        if not self._refresh_all():
            self._fault(self._feedback_failure_message())
            raise ControlError(self.last_motion_result['message'])

        current = self.motors[index].get_current_status()['actuate_len']
        target = current + float(delta_m)
        self.final_target_actuation_lengths[index] = target
        if not self.motors[index].set_target_actuation_length(target, retry_on_timeout=True):
            self._fault(self._feedback_failure_message())
            raise ControlError(self.last_motion_result['message'])
        self.last_commanded_lengths[index] = target
        if index == 3:
            self.deployed = None
            self.deploy_state = 'INTERMEDIATE'
            self._result(False, 'Deploy micro-adjust applied; endpoint is unconfirmed.')
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
        return self._apply_single_motor_delta(3, delta_length_m)

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
        return self._refresh_all()

    def set_deploy(self, deploy: bool, *, _cycle_token=None):
        if self._recovery_active: return {'success':False,'complete':False,'message':'RECOVERY_OWNS_MOTION'}
        """设置展开/收起状态。"""
        if self._cycle_active and _cycle_token is not self.cycle.owner:
            return self._result(False,'CYCLE_OWNS_MOTION')
        self._cycle_checkpoint()
        if self._bend_motion_active:
            return self._result(False,'BEND_MOTION_BUSY')
        if not self._reference_known:
            return self._result(False, 'Motion rejected: mechanical reference unknown after startup.')
        if self.deploy_state == 'FAULT':
            return self._result(False, 'Motion rejected: controller fault requires explicit recovery/reference confirmation.')
        if self.bend_controller is not None:
            self.bend_controller.reset()
        try:
            self._read_camera(deploy=deploy)
        except Exception as exc:
            return self._fault(str(exc))
        if not self._refresh_all():
            return self._fault(self._feedback_failure_message())
        nominal_target = self.deploy_delta_length if deploy else 0.0
        target = self.visual_endpoint_lengths.get(bool(deploy),nominal_target) if self.visual_servo is not None else nominal_target
        if self.visual_servo is not None:
            try:
                already_confirmed=(not self._cycle_active and bool(deploy) in self.visual_endpoint_lengths and
                    abs(self.motors[3].get_current_status()['actuate_len']-target)<=self.deploy_tolerance_m)
                self.visual_servo.begin(nominal_target,target,self._clock(),confirmed_hold=already_confirmed)
            except Exception as exc:
                return self._fault(str(exc))
            self._visual_motion_active=True
        self.final_target_actuation_lengths[3] = target
        command = self.motors[3].get_current_status()['actuate_len']
        if self.visual_servo is not None and not (
                self.visual_servo.config.min_length_m<=command<=self.visual_servo.config.max_length_m):
            return self._fault('VISUAL_SERVO_INITIAL_POSITION_OUT_OF_LIMIT')
        start_position=command
        distance=abs(target-start_position)
        final_camera_tolerance=self.camera_gate.config.position_tolerance_m
        final_motor_tolerance=self.deploy_tolerance_m
        handoff_column=None
        if self._cycle_active and deploy and self.bend_controller is not None:
            reserve=self.cycle.cfg.neutral_tolerance_m-self.bend_controller.cfg.camera_noise_bound_m
            final_camera_tolerance=min(final_camera_tolerance,reserve)
            # Preserve the next stage's complete 3D error budget. A transverse
            # residual cannot be corrected by the deployment actuator alone.
            handoff_column=self.visual_servo.config.jacobian_column
            scale=math.sqrt(sum(v*v for v in handoff_column))
            final_motor_tolerance=min(final_motor_tolerance,reserve/scale)
        direction=1.0 if target>=start_position else -1.0
        needs_progress=distance>self.deploy_tolerance_m
        required_progress=min(self.progress_min_m,max(0.0,distance-self.deploy_tolerance_m))
        self.deploy_state = 'DEPLOYING' if deploy else 'RETRACTING'
        self.deployed = None
        started = self._clock()
        progress_timeout=self.progress_timeout_s
        if self.deploy_progress_window is not None:
            policy=self.deploy_progress_window.cfg
            rate=policy.expected_deploy_rate_m_s*min(1.0,self.deploy_speed_m_s/policy.reference_deploy_speed_m_s)
            self.deploy_progress_window.begin(self.camera_goals[bool(deploy)],self.camera_gate.config.position_tolerance_m,started,rate)
            if self.pause_budget is not None:
                minimum_rate=self.deploy_progress_window.rate
                column=(self.visual_servo.config.jacobian_column if self.visual_servo is not None else None)
                if column is None: return self._fault('PAUSE_BUDGET_REQUIRES_CALIBRATED_DEPLOYMENT')
                camera_per_motor=math.sqrt(sum(v*v for v in column))
                progress_timeout=self.pause_budget.timeout(self.progress_timeout_s,self.progress_min_m,minimum_rate,camera_per_motor)
                camera_timeout=self.pause_budget.timeout(self._base_camera_config.progress_timeout_s,
                    self._base_camera_config.progress_min_m,minimum_rate)
                self.camera_gate.config=replace(self._base_camera_config,progress_timeout_s=camera_timeout)
                self.pause_budget_status={'enabled':True,**asdict(self.pause_budget),
                    'motor_timeout_s':progress_timeout,'camera_progress_timeout_s':camera_timeout,
                    'minimum_camera_rate_m_s':minimum_rate,'camera_per_motor':camera_per_motor,
                    'motor_progress_m':self.progress_min_m,'window_s':policy.window_s}
        previous = started - self.deploy_period_s
        settled_since = None
        best_progress=advance_mark=0.0
        last_progress_time=started
        self.motion_progress={'start_m':start_position,'target_m':target,'required':needs_progress,
                              'threshold_m':required_progress,'best_m':0.0,'seen':not needs_progress,
                              'final_camera_tolerance_m':final_camera_tolerance,'final_motor_tolerance_m':final_motor_tolerance}
        # Keep the existing synchronous service contract: success means completed.
        # Fixed-period sleeps remove the busy loop; every tick refreshes four motors.
        # Transport calls must themselves have bounded I/O timeouts.
        while True:
            self._cycle_checkpoint()
            now = self._clock()
            if self.visual_servo is not None:
                try:
                    self.visual_servo.check_deadline(now)
                except Exception as exc:
                    return self._fault(str(exc))
            if now - started >= self.deploy_timeout_s:
                return self._fault(('VISUAL_SERVO_OPERATION_TIMEOUT: ' if self.visual_servo is not None else
                    'CAMERA_TARGET_NOT_CONFIRMED: ' if self.camera_required else '')+
                    'Motion timed out before all required measured targets remained stable.')
            dt = min(max(now - previous, 0.0), 2.0 * self.deploy_period_s)
            previous = now
            limit = self.deploy_speed_m_s * dt
            command += max(-limit, min(limit, target - command))
            if not self.motors[3].set_target_actuation_length(command):
                return self._fault(self._feedback_failure_message())
            self.last_commanded_lengths[3] = command
            active_ok = [motor.refresh_feedback() for motor in self.motors[:3]]
            feedback = self.motors[3].get_current_status()
            if not all(active_ok) or not feedback['feedback_valid']:
                return self._fault(self._feedback_failure_message())
            try:
                camera_ready=self._read_camera()
                if self.deploy_progress_window is not None: self.deploy_progress_window.observe(self.camera_gate.last_sample)
                self._cycle_checkpoint()
            except Exception as exc:
                return self._fault(str(exc))
            net_progress=direction*(feedback['actuate_len']-start_position)
            best_progress=max(best_progress,net_progress)
            if best_progress-advance_mark>=self.progress_min_m:
                advance_mark=best_progress
                last_progress_time=now
            seen_progress=not needs_progress or best_progress>=required_progress-1e-9
            self.motion_progress.update({'best_m':best_progress,'seen':seen_progress,
                'seconds_without_progress':now-last_progress_time})
            at_target = (abs(command - target) <= 1e-9
                         and abs(feedback['actuate_len'] - target) <= final_motor_tolerance
                         and abs(feedback['actuate_len_vel']) <= 0.002 and seen_progress)
            if needs_progress and not at_target and now-last_progress_time>=progress_timeout:
                self.motion_progress['fault']='MOTION_PROGRESS_STALLED'
                return self._fault('MOTION_PROGRESS_STALLED: no sufficient measured progress within configured window.')
            if at_target:
                if settled_since is None:
                    settled_since = now
                settled=now-settled_since>=self.deploy_settle_s
                fresh_after_settle=(self.visual_servo is None or
                    self.camera_gate.last_sample.capture_s>=settled_since+self.visual_servo.config.settle_wait_s)
                camera_precise=(not self.camera_required or
                    self.camera_gate.status().get('position_error_m',float('inf'))<=final_camera_tolerance)
                if handoff_column is not None:
                    # Also check the commanded motion still pending at the
                    # motor, using the explicitly calibrated deployment column.
                    sample=self.camera_gate.last_sample
                    remaining=target-feedback['actuate_len']
                    held_error=math.sqrt(sum((p-g+j*remaining)**2 for p,g,j in zip(
                        sample.position_m,self.camera_gate.goal.position_m,handoff_column)))
                    self.motion_progress['handoff_predicted_error_m']=held_error
                    camera_precise=camera_precise and held_error<=final_camera_tolerance
                if settled and camera_ready and fresh_after_settle and camera_precise:
                    if self.visual_servo is not None and self.visual_servo.online is not None:
                        try:
                            self.visual_servo.plan(self.camera_gate.last_sample,self.camera_gate.goal,target,
                                settled_since,self._clock(),actual_length_m=feedback['actuate_len'],
                                other_lengths_m=[m.get_current_status()['actuate_len'] for m in self.motors[:3]],
                                allow_motion=False)
                        except Exception as exc:
                            return self._fault(str(exc))
                    self.deployed = bool(deploy)
                    self.deploy_state = 'DEPLOYED' if deploy else 'RETRACTED'
                    if self.deploy_progress_window is not None: self.deploy_progress_window.finish(True)
                    if self.visual_servo is not None:
                        self.visual_endpoint_lengths[bool(deploy)]=target
                        self.visual_servo.active=False
                        self._visual_motion_active=False
                    return self._result(True, 'Arm deployed.' if deploy else 'Arm retracted.')
                if settled and self.visual_servo is not None:
                    try:
                        proposed=self.visual_servo.plan(self.camera_gate.last_sample,self.camera_gate.goal,
                            target,settled_since,self._clock(),actual_length_m=feedback['actuate_len'],
                            other_lengths_m=[m.get_current_status()['actuate_len'] for m in self.motors[:3]])
                    except Exception as exc:
                        return self._fault(str(exc))
                    if proposed is not None:
                        target=proposed
                        self.final_target_actuation_lengths[3]=target
                        start_position=feedback['actuate_len']
                        distance=abs(target-start_position)
                        direction=1.0 if target>=start_position else -1.0
                        needs_progress=distance>self.deploy_tolerance_m
                        required_progress=min(self.progress_min_m,max(0.0,distance-self.deploy_tolerance_m))
                        best_progress=advance_mark=0.0
                        last_progress_time=now
                        settled_since=None
                        self.motion_progress.update({'start_m':start_position,'target_m':target,
                            'nominal_target_m':nominal_target,'required':needs_progress,
                            'threshold_m':required_progress,'best_m':0.0,'seen':not needs_progress})
            else:
                settled_since = None
            self._sleep(self.deploy_period_s)

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
        deploy_motor_torque_gain_scale: Optional[float] = None,
        **control_options,
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
            **control_options,
        )
        self.kinematics = TensegrityArmKinematics(is_debug=is_debug)

    def update_bend(self, theta: float, phi: float):
        """设置弯折角度 (theta: pitch, phi: yaw)。"""
        if self.bend_controller is not None:
            return False  # No unverified angle-to-camera goal map is invented.
        if not self.deployed or self.deploy_state == 'FAULT':
            print("[Arm] Please deploy the arm before setting bend angles.") if self.is_debug else None
            return False

        l1, l2, l3 = self.kinematics.calculate_actuation_lengths_for_bend(theta, phi)
        self.final_target_actuation_lengths = [l1, l2, l3, self.final_target_actuation_lengths[3]]  # 保持伸缩弦长度不变
        return True

    def step(self):
        if self._recovery_active: return False
        """ 根据最终长度目标, 使用tanh函数平滑控制命令, 并打印状态 """
        if self.deploy_state == 'FAULT':
            return False
        if self._cycle_active:
            return False
        if self.bend_controller is not None:
            if self._bend_motion_active:
                return False
            if not self._refresh_all():
                self._fault(self._feedback_failure_message())
            return False  # Hold current validated commands; no legacy target chasing.
        if not self.deployed:
            if not self._refresh_all():
                self._fault(self._feedback_failure_message())
            print("[Arm] Please deploy the arm before stepping.") if self.is_debug else None
            return False
        
        ''' 根据目前长度与目标长度的误差, 使用tanh函数平滑控制命令 '''
        for i in range(3):
            error = self.final_target_actuation_lengths[i] - self.last_commanded_lengths[i]
            k = 50.0           # 增大k可以让控制更接近于阶跃函数
            max_step = 0.002   # 每周期最多改 2 mm
            control_signal = max_step * math.tanh(k * error)
            next_length = self.last_commanded_lengths[i] + control_signal
            if not self.motors[i].set_target_actuation_length(next_length):
                self._fault(self._feedback_failure_message())
                return False
            self.last_commanded_lengths[i] = next_length

        if not self.motors[3].refresh_feedback():
            self._fault(self._feedback_failure_message())
            return False


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

        feedbacks = self.get_feedbacks()
        if not all(fb['feedback_valid'] for fb in feedbacks):
            self._fault(self._feedback_failure_message())
            return False
        total_error = sum(abs(self.final_target_actuation_lengths[i] - feedbacks[i]['actuate_len']) for i in range(3))
        if total_error < 0.003:
            print(f"[Arm] Reached target bend angles within tolerance. Total length error: {total_error:.4f} m") if self.is_debug else None
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
