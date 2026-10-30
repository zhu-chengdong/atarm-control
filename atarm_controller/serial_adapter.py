"""FD-based A1 transport with immutable evidence from the C++ receive path.

No device path is opened here. The default C++ build accepts PTY slaves only.
The official SDK shared library is used for codecs and constants, not I/O.
"""
from __future__ import annotations
import ctypes as C
from dataclasses import dataclass
from enum import Enum
import math
from pathlib import Path
import threading


class MotorType(Enum):
    A1 = 0


class MotorMode(Enum):
    FOC = 1


@dataclass
class MotorCmd:
    motorType: MotorType = MotorType.A1
    mode: int = 10
    id: int = 0
    q: float = 0.0
    dq: float = 0.0
    tau: float = 0.0
    kp: float = 0.0
    kd: float = 0.0


@dataclass
class MotorData:
    motorType: MotorType = MotorType.A1
    motor_id: int = 255
    mode: int = 255
    q: float = math.nan
    dq: float = math.nan
    tau: float = math.nan
    temp: int = 0
    merror: int = 0
    correct: bool = False
    hex_len: int = 0


@dataclass(frozen=True)
class FeedbackSample:
    valid: bool
    status: str
    transport_frame_id: int = 0
    tx_ns: int = 0
    rx_ns: int = 0
    actual_rx_length: int = 0
    discarded_bytes: int = 0
    motor_id: int = 255
    mode: int = 255
    temp: int = 0
    merror: int = 0
    q: float = math.nan
    dq: float = math.nan
    tau: float = math.nan
    crc_ok: bool = False
    freshness_basis: str = 'HOST_VALIDATED_SERIAL_RX'
    device_sequence: object = None
    device_sample_time: object = None


class _Command(C.Structure):
    _fields_ = [('motor_id',C.c_uint8),('mode',C.c_uint8)] + [(name,C.c_double) for name in ('q','dq','tau','kp','kd')]


class _Feedback(C.Structure):
    _fields_ = [(name,C.c_uint64) for name in ('frame_id','tx_ns','rx_ns')] + [
        ('actual_rx_length',C.c_uint32),('discarded_bytes',C.c_uint32),('status',C.c_int32),
        ('motor_id',C.c_uint8),('mode',C.c_uint8),('temp',C.c_int32),('merror',C.c_int32),
        ('q',C.c_double),('dq',C.c_double),('tau',C.c_double)]


class NativeLibrary:
    _instance = None
    _lock = threading.Lock()
    def __init__(self, path=None):
        path = Path(path) if path is not None else Path(__file__).resolve().parents[1] / 'native/build/liba1_serial_adapter.so'
        self.path = path
        self.lib = C.CDLL(str(path))
        self.lib.a1_create_from_fd.argtypes = [C.c_int,C.c_uint64]
        self.lib.a1_create_from_fd.restype = C.c_void_p
        self.lib.a1_destroy.argtypes = [C.c_void_p]
        self.lib.a1_exchange.argtypes = [C.c_void_p,C.POINTER(_Command),C.POINTER(_Feedback)]
        self.lib.a1_exchange.restype = C.c_int
        self.lib.a1_status_name.argtypes = [C.c_int]
        self.lib.a1_status_name.restype = C.c_char_p
        self.lib.a1_gear_ratio.restype = C.c_double
        self.lib.a1_foc_mode.restype = C.c_int
        if self.lib.a1_command_size()!=C.sizeof(_Command) or self.lib.a1_feedback_size()!=C.sizeof(_Feedback):
            raise RuntimeError('Native/Python structure ABI mismatch; refusing transport.')
    def physical_uart_enabled(self):
        try:flag=self.lib.a1_physical_uart_enabled
        except AttributeError:return False
        flag.restype=C.c_int
        return flag()==1
    @classmethod
    def shared(cls):
        with cls._lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance


def queryGearRatio(motor_type):
    if motor_type != MotorType.A1:
        raise ValueError('This adapter supports A1 only.')
    return NativeLibrary.shared().lib.a1_gear_ratio()


def queryMotorMode(motor_type, mode):
    if motor_type != MotorType.A1 or mode != MotorMode.FOC:
        raise ValueError('This adapter supports A1 FOC only.')
    return NativeLibrary.shared().lib.a1_foc_mode()


class NativeSerialAdapter:
    def __init__(self, fd, timeout_s=0.05, *, native_library=None):
        if not math.isfinite(timeout_s) or not 0 < timeout_s <= 5:
            raise ValueError('Finite timeout must be between zero and five seconds.')
        self.native = native_library or NativeLibrary.shared()
        self._lock = threading.Lock()
        self._handle = self.native.lib.a1_create_from_fd(fd,int(timeout_s*1e9))
        if not self._handle:
            raise RuntimeError('Descriptor rejected; offline build accepts an already-open PTY slave only.')
        self.last_sample = FeedbackSample(False,'NO_REPLY')
        self.trace = []

    def exchange(self, command):
        if command.motorType != MotorType.A1 or not 0 <= int(command.id) < 255:
            return FeedbackSample(False,'INVALID_COMMAND')
        raw_command = _Command(int(command.id),int(command.mode),float(command.q),float(command.dq),
                               float(command.tau),float(command.kp),float(command.kd))
        raw = _Feedback()
        with self._lock:
            status = self.native.lib.a1_exchange(self._handle,C.byref(raw_command),C.byref(raw))
            sample = FeedbackSample(valid=status==0,status=self.native.lib.a1_status_name(status).decode('ascii'),
                transport_frame_id=int(raw.frame_id),tx_ns=int(raw.tx_ns),rx_ns=int(raw.rx_ns),
                actual_rx_length=int(raw.actual_rx_length),discarded_bytes=int(raw.discarded_bytes),
                motor_id=int(raw.motor_id),mode=int(raw.mode),temp=int(raw.temp),merror=int(raw.merror),
                q=float(raw.q),dq=float(raw.dq),tau=float(raw.tau),crc_ok=status==0)
            self.last_sample = sample  # Invalid results replace validity; no old success is returned.
            self.trace.append({'status':sample.status,'valid':sample.valid,'frame_id':sample.transport_frame_id,
                'tx_ns':sample.tx_ns,'rx_ns':sample.rx_ns,'actual_rx_length':sample.actual_rx_length,
                'discarded_bytes':sample.discarded_bytes,'motor_id':int(command.id),
                'command_q':float(command.q),'q':sample.q if sample.valid else None,
                'freshness_basis':sample.freshness_basis})
            return sample

    def close(self):
        with self._lock:
            if self._handle:
                self.native.lib.a1_destroy(self._handle)
                self._handle = None
