"""Four calibrated A1 channels through caller-owned validated FD transports.

No device path is opened. NativeSerialAdapter performs real protocol/CRC I/O;
the shipped native build rejects physical UARTs. A caller must provision
transports and installation calibration explicitly before any deployment.
"""
from dataclasses import dataclass
import math
import threading
import time
from .serial_adapter import MotorCmd,NativeSerialAdapter
from .cartesian_control import MotorEvidence,Observation,finite_vector


@dataclass(frozen=True)
class ChannelCalibration:
    bus:str
    motor_id:int
    direction:int
    radius_m:float
    gear_ratio:float
    encoder_reference_rad:float
    length_reference_m:float
    kp:float
    kd:float
    max_feedback_torque_nm:float
    max_temperature_c:float

    def __post_init__(self):
        if not self.bus or type(self.motor_id) is not int or not 0<=self.motor_id<255 or self.motor_id==187:
            raise ValueError('ACTUATOR_INVALID_ID')
        if type(self.direction) is not int or self.direction not in (-1,1):raise ValueError('ACTUATOR_DIRECTION_UNSET')
        for k in ('radius_m','gear_ratio','kp','kd','max_feedback_torque_nm','max_temperature_c'):
            v=getattr(self,k)
            if type(v) not in (int,float) or not math.isfinite(v) or v<=0:raise ValueError('ACTUATOR_INVALID_CALIBRATION_'+k)
        if self.kp>15.9 or self.kd>31.9:raise ValueError('ACTUATOR_CODEC_GAIN_LIMIT')
        if not finite_vector((self.encoder_reference_rad,self.length_reference_m),2):raise ValueError('ACTUATOR_REFERENCE_INVALID')

    def length(self,angle):return self.length_reference_m+self.direction*(angle-self.encoder_reference_rad)/self.gear_ratio*self.radius_m
    def angle(self,length):return self.encoder_reference_rad+self.direction*(length-self.length_reference_m)/self.radius_m*self.gear_ratio


class PtyFdFactory:
    """Accept existing PTY FDs only, independently of the native build guard."""
    def __init__(self,fds,timeout_s=.02):self.fds=dict(fds);self.timeout=timeout_s
    def __call__(self):
        import os
        if not self.fds or any(not os.ttyname(fd).startswith('/dev/pts/') for fd in self.fds.values()):
            raise ValueError('OFFLINE_PTY_DESCRIPTORS_REQUIRED')
        result={}
        try:
            for bus,fd in self.fds.items():result[bus]=NativeSerialAdapter(fd,self.timeout)
            return result
        except Exception:
            for adapter in result.values():adapter.close()
            raise


class FdActuatorIO:
    execution_domain='fd-actuator'
    closed_loop_io_contract='calibrated-a1-v1'

    def __init__(self,config,channels,transport_factory,pose_source,*,speed_m_s,cancel_event=None):
        self.owner=threading.get_ident();self.owner_name=threading.current_thread().name
        self.cfg=config;self.channels=tuple(channels);self.pose_source=pose_source
        if len(channels)!=4 or len({(c.bus,c.motor_id) for c in channels})!=4:
            raise ValueError('ACTUATOR_MAPPING_INVALID')
        if tuple(c.length_reference_m for c in channels)!=config.reference_lengths_m:
            raise ValueError('ACTUATOR_REFERENCE_CONFIG_MISMATCH')
        if not math.isfinite(speed_m_s) or speed_m_s<=0:raise ValueError('ACTUATOR_SPEED_INVALID')
        self.speed=speed_m_s;self.cancel_event=cancel_event or threading.Event()
        self.transports=transport_factory()
        if set(self.transports)!={c.bus for c in channels}:
            for t in self.transports.values():t.close()
            raise ValueError('ACTUATOR_BUS_MAPPING_INVALID')
        self.target=None;self.sent=None;self.holding=False;self.stopped=False;self.last_s=self.now()
        self.trace=[];self.last_samples=[None]*4;self.last_frame_ids=[0]*4
        self.io_threads=set();self.command_count=0;self.last_evidence=None;self.closed=False

    def own(self):
        if threading.get_ident()!=self.owner:raise RuntimeError('ACTUATOR_IO_OWNER_VIOLATION')
        self.io_threads.add(threading.current_thread().name)

    @staticmethod
    def now():return time.monotonic()
    def cancel_requested(self):return self.cancel_event.is_set()

    def exchange(self,i,cmd,*,stopping=False):
        self.own();c=self.channels[i];sample=self.transports[c.bus].exchange(cmd)
        self.trace.append({'channel':i,'bus':c.bus,'motor_id':c.motor_id,'thread':threading.current_thread().name,
            'stop':stopping,'holding':cmd.kp>0,'q_command_rad':cmd.q,'frame_id':sample.transport_frame_id,
            'valid':sample.valid,'status':sample.status,'tx_ns':sample.tx_ns,'rx_ns':sample.rx_ns})
        self.last_samples[i]=sample
        return sample

    def poll_motors(self):
        self.own()
        if self.stopped:raise RuntimeError('ACTUATOR_EXPLICIT_RESTART_REQUIRED')
        now=self.now();dt=min(max(now-self.last_s,0.),self.cfg.poll_s*2);self.last_s=now
        if self.holding:
            delta=tuple(b-a for a,b in zip(self.sent,self.target));largest=max(abs(x) for x in delta)
            scale=min(1.,self.speed*dt/max(largest,1e-15));self.sent=tuple(a+scale*d for a,d in zip(self.sent,delta))
        positions=[];frames=[];times=[]
        for i,c in enumerate(self.channels):
            if self.cancel_requested():raise RuntimeError('ACTUATOR_CANCELLED')
            cmd=MotorCmd(id=c.motor_id,q=c.angle(self.sent[i]) if self.holding else 0.,
                         kp=c.kp if self.holding else 0.,kd=c.kd if self.holding else 0.)
            sample=self.exchange(i,cmd)
            if not sample.valid or not sample.crc_ok or sample.motor_id!=c.motor_id or sample.mode!=cmd.mode:
                raise RuntimeError('ACTUATOR_FEEDBACK_'+sample.status)
            if sample.transport_frame_id<=self.last_frame_ids[i] or not 0<sample.tx_ns<=sample.rx_ns:
                raise RuntimeError('ACTUATOR_FEEDBACK_REPLAY')
            self.last_frame_ids[i]=sample.transport_frame_id
            if self.now()-sample.rx_ns/1e9>self.cfg.max_motor_age_s:raise RuntimeError('ACTUATOR_FEEDBACK_STALE')
            if abs(sample.tau)>c.max_feedback_torque_nm or sample.temp>c.max_temperature_c:
                raise RuntimeError('ACTUATOR_FEEDBACK_LOAD_LIMIT')
            length=c.length(sample.q)
            if not self.cfg.min_lengths_m[i]<=length<=self.cfg.max_lengths_m[i]:raise RuntimeError('ACTUATOR_MEASURED_LIMIT')
            positions.append(length);frames.append(sample.transport_frame_id);times.append(sample.rx_ns/1e9)
        self.last_evidence=MotorEvidence(tuple(positions),tuple(frames),tuple(times),(True,)*4)
        return self.last_evidence

    def read(self):
        if self.holding:
            pose=self.pose_source()
            if not 0<=self.now()-pose.capture_s<=self.cfg.max_pose_age_s:raise ValueError('ACTUATOR_POSE_STALE')
        motors=self.poll_motors();return Observation(self.pose_source(),motors)

    def arm_reference(self,evidence):
        self.own()
        if self.stopped:raise RuntimeError('ACTUATOR_EXPLICIT_RESTART_REQUIRED')
        if any(abs(x-y)>self.cfg.motor_tolerance_m for x,y in zip(evidence.positions_m,self.cfg.reference_lengths_m)):
            raise ValueError('ACTUATOR_MECHANICAL_REFERENCE_MISMATCH')
        # Hold the measured position; never re-zero an encoder or move home.
        self.sent=self.target=evidence.positions_m;self.holding=True;self.last_s=self.now()

    def command(self,target):
        self.own()
        if not self.holding or self.stopped or self.cancel_requested():raise RuntimeError('ACTUATOR_NOT_ARMED')
        if not finite_vector(target,4) or any(not lo<=x<=hi for x,lo,hi in zip(target,self.cfg.min_lengths_m,self.cfg.max_lengths_m)):
            raise ValueError('ACTUATOR_COMMAND_LIMIT')
        if self.last_evidence is None or max(abs(x-y) for x,y in zip(target,self.last_evidence.positions_m))>self.cfg.max_motor_step_m+1e-10:
            raise ValueError('ACTUATOR_COMMAND_STEP_LIMIT')
        self.target=tuple(target);self.command_count+=1;return True

    def advance(self,dt):
        self.own()
        if not math.isfinite(dt) or dt<=0:raise ValueError('ACTUATOR_INVALID_WAIT')
        self.cancel_event.wait(dt)

    def stop(self):
        self.own();self.stopped=True;self.holding=False;receipts=[]
        for i in (3,0,1,2):
            c=self.channels[i]
            try:
                s=self.exchange(i,MotorCmd(id=c.motor_id),stopping=True)
                receipts.append({'actuator_index':i,'bus':c.bus,'motor_id':c.motor_id,'reply_valid':s.valid,
                    'status':s.status,'transport_frame_id':s.transport_frame_id,'tx_ns':s.tx_ns,'rx_ns':s.rx_ns})
            except Exception as exc:receipts.append({'actuator_index':i,'reply_valid':False,'error':str(exc)})
        return receipts

    def close(self):
        self.own()
        if not self.closed:
            for t in self.transports.values():t.close()
            self.closed=True
