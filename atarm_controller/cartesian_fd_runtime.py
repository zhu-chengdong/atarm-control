"""Asynchronous Cartesian jobs with all actuator traffic owned by one thread."""
from copy import deepcopy
from dataclasses import asdict
from collections import deque
import json
import threading
import time
from .cartesian_control import CartesianController,CartesianConfig,PositionGoal
from .actuator_io import ChannelCalibration,FdActuatorIO
from .ros_motion_worker import finite_json


def load_session(path):
    with open(path,encoding='utf-8') as stream:d=json.load(stream)
    if d.get('schema_version')!=1 or not d.get('calibration_id'):
        raise ValueError('ACTUATOR_EXPLICIT_CALIBRATION_REQUIRED')
    cfg=CartesianConfig(**d['controller'])
    channels=tuple(ChannelCalibration(**c) for c in d['channels'])
    return d,cfg,channels


class FdJobRunner:
    def __init__(self,cfg,channels,transport_factory,vision,*,speed_m_s,calibration_id):
        self.cfg=cfg;self.vision=vision;self.reference=vision.reference;self.calibration_id=calibration_id
        self.channels=channels;self.factory=transport_factory;self.speed=speed_m_s
        self.lock=threading.RLock();self.wake=threading.Event();self.quit=threading.Event();self.interrupt=threading.Event()
        self.pending=None;self.counter=0;self.seen=set();self.ready=False;self.fault=None;self.io=None
        self.last={'state':'UNCONFIRMED','success':False,'complete':False}
        self.cached=None;self.stable=deque();self.completed=[];self.audit={}
        self.thread=threading.Thread(target=self.run,name='cartesian-fd-owner',daemon=True);self.thread.start()

    def reply(self,rid,accepted,reason,**extra):
        return dict(request_id=rid,accepted=accepted,success=accepted,complete=False,reason=reason,**extra)

    def submit_json(self,payload):
        rid=None
        try:
            d=json.loads(payload)
            if not isinstance(d,dict):raise ValueError('Goal object required')
            rid=d.get('request_id')
            if not isinstance(rid,str) or not 1<=len(rid)<=64:rid=None;raise ValueError('Invalid ID')
            goal=PositionGoal(**{k:v for k,v in d.items() if k!='request_id'})
        except (ValueError,TypeError,KeyError) as exc:return self.reply(rid,False,'INVALID_GOAL',detail=str(exc))
        with self.lock:
            if self.quit.is_set():return self.reply(rid,False,'SHUTTING_DOWN')
            if rid in self.seen:return self.reply(rid,False,'DUPLICATE_ID')
            if goal.key!=self.reference:return self.reply(rid,False,'REFERENCE_MISMATCH')
            if self.pending:return self.reply(rid,False,'BUSY')
            if self.fault:return self.reply(rid,False,'EXPLICIT_RESTART_REQUIRED')
            if not self.ready:return self.reply(rid,False,'REFERENCE_NOT_CONFIRMED')
            if len(self.seen)>=256:return self.reply(rid,False,'REQUEST_LIMIT_RESTART_REQUIRED')
            self.seen.add(rid);self.counter+=1;self.interrupt.clear()
            self.pending={'kind':'goal','goal':goal,'request_id':rid,'id':self.counter}
            self.last={'state':'ACCEPTED','success':False,'complete':False,'request_id':rid,'job_id':self.counter}
            self.wake.set();return self.reply(rid,True,'ACCEPTED',job_id=self.counter)

    def confirm_reference(self):
        with self.lock:
            if self.quit.is_set():return self.reply(None,False,'SHUTTING_DOWN')
            if self.pending:return self.reply(None,False,'BUSY')
            if self.fault:return self.reply(None,False,'EXPLICIT_RESTART_REQUIRED')
            if self.ready:return self.reply(None,False,'ALREADY_CONFIRMED')
            self.pending={'kind':'confirm'};self.wake.set()
            return self.reply(None,True,'REFERENCE_CHECK_ACCEPTED')

    def cancel(self):
        with self.lock:
            if not self.pending or self.pending['kind']!='goal' or self.pending.get('finishing'):
                return self.reply(None,False,'NO_CANCELLABLE_GOAL')
            self.interrupt.set();self.wake.set();return self.reply(self.pending['request_id'],True,'CANCEL_ACCEPTED')

    def snapshot(self):
        with self.lock:return deepcopy({'execution_domain':'fd-actuator','calibration_id':self.calibration_id,
            'ready':self.ready,'fault':self.fault,'reference':dict(zip(('parent_frame','child_frame','map_epoch'),self.reference)),
            'job':self.last,'observation':self.cached,'vision':self.vision.diagnostics()})

    def observe(self,obs):
        with self.lock:
            self.cached={'position_m':obs.pose.position_m,'pose_frame_id':obs.pose.frame_id,'capture_s':obs.pose.capture_s,
                'motor_positions_m':obs.motors.positions_m,'motor_frame_ids':obs.motors.frame_ids,
                'motor_received_s':obs.motors.captured_s,'time_domain':'host_monotonic_seconds'}
        if not self.stable or self.stable[-1].pose.frame_id!=obs.pose.frame_id:
            self.stable.append(obs)
            cutoff=obs.pose.capture_s-self.cfg.stable_s
            while len(self.stable)>self.cfg.filter_frames and self.stable[1].pose.capture_s<=cutoff:
                self.stable.popleft()

    def latch(self,reason,receipts=None):
        if receipts is None:receipts=self.io.stop() if self.io else []
        with self.lock:
            self.fault=reason;self.ready=False;self.pending=None
            self.last={'state':'FAULT','success':False,'complete':False,'reason':reason,'stop_receipts':receipts}

    def idle(self):
        obs=self.io.read()
        if not 0<=self.io.now()-obs.pose.capture_s<=self.cfg.max_pose_age_s:raise ValueError('IDLE_POSE_STALE')
        self.observe(obs)
        return obs

    def confirm(self):
        try:
            deadline=self.io.now()+self.cfg.settle_timeout_s
            while True:
                obs=self.idle()
                if len(self.stable)>=self.cfg.filter_frames and self.stable[-1].pose.capture_s-self.stable[0].pose.capture_s>=self.cfg.stable_s:
                    break
                if self.io.now()>=deadline:raise ValueError('REFERENCE_STABLE_WINDOW_NOT_READY')
                if self.quit.wait(self.cfg.poll_s):raise ValueError('REFERENCE_CONFIRM_SHUTDOWN')
            p=obs.pose.position_m
            if any(sum((a-b)**2 for a,b in zip(x.pose.position_m,p))**.5>2*self.cfg.camera_noise_bound_m for x in self.stable):
                raise ValueError('REFERENCE_POSE_UNSTABLE')
            self.io.arm_reference(obs.motors)
            with self.lock:self.ready=True;self.last={'state':'READY','success':False,'complete':False}
        except Exception as exc:
            # A failed reference assertion never creates a zero or sends motion.
            if str(exc).startswith('REFERENCE_') or str(exc)=='ACTUATOR_MECHANICAL_REFERENCE_MISMATCH':
                with self.lock:self.last={'state':'UNCONFIRMED','success':False,'complete':False,'reason':str(exc)}
            else:self.latch(str(exc))
        finally:
            with self.lock:self.pending=None

    def execute(self,job):
        controller=CartesianController(self.cfg,self.io);result=controller.move_position(job['goal'])
        # Atomic completion versus accepted cancel; no serial I/O while the
        # callback-facing lock is held. Once finishing, keep the job busy.
        with self.lock:
            cancelled=self.interrupt.is_set();self.last['state']='FINISHING'
            job['finishing']=True
        if cancelled and result.get('success'):
            result=dict(result,success=False,complete=False,state='CANCELLED',stop_receipts=self.io.stop())
        elif cancelled:result=dict(result,success=False,complete=False,state='CANCELLED')
        with self.lock:
            if not result.get('success'):self.fault=result.get('reason','CANCELLED');self.ready=False
            self.last={'request_id':job['request_id'],'job_id':job['id'],**result}
            self.completed.append({'job':deepcopy(self.last),'history':deepcopy(controller.history),'updates':deepcopy(controller.updates)})
            self.pending=None

    def run(self):
        try:
            self.io=FdActuatorIO(self.cfg,self.channels,self.factory,self.vision,speed_m_s=self.speed,cancel_event=self.interrupt)
            # Observe all I/O including motion reads without letting ROS access
            # transports or a simulated truth function.
            read=self.io.read
            def monitored():
                obs=read();self.observe(obs);return obs
            self.io.read=monitored
            while not self.quit.is_set():
                with self.lock:job=self.pending
                if job:
                    if job['kind']=='confirm':self.confirm()
                    else:
                        with self.lock:self.last['state']='RUNNING'
                        self.execute(job)
                elif not self.fault:
                    try:self.idle()
                    except ValueError as exc:
                        if str(exc)!='ROS_POSE_NOT_READY':self.latch(str(exc))
                    except Exception as exc:self.latch(str(exc))
                self.wake.wait(self.cfg.poll_s);self.wake.clear()
        except Exception as exc:self.latch('WORKER_ERROR: '+str(exc))
        finally:
            receipts=[]
            if self.io:
                receipts=self.io.stop();self.io.close()
            self.audit=finite_json({'stopped':self.io.stopped if self.io else False,'closed':self.io.closed if self.io else False,
                'io_threads':sorted(self.io.io_threads) if self.io else [],'shutdown_stop_receipts':receipts,
                'trace':self.io.trace if self.io else [],'jobs':self.completed,'last_state':self.snapshot()})

    def close(self):
        self.interrupt.set();self.quit.set();self.wake.set();self.thread.join(5.)
        if self.thread.is_alive():raise RuntimeError('FD_WORKER_SHUTDOWN_TIMEOUT')
