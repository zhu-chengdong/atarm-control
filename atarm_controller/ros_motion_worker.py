"""One owner for every motor exchange; ROS callbacks only submit or cancel."""
from copy import deepcopy
import math,threading,time

def finite_json(value):
    if isinstance(value,float) and not math.isfinite(value):return None
    if isinstance(value,dict):return {k:finite_json(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [finite_json(v) for v in value]
    return value

def component_summary(obj):
    if obj is None:return {'enabled':False}
    fields=('active','state','phase','run_id','attempt_id','stop_requested','fault','steps','adjustments')
    result={k:getattr(obj,k) for k in fields if hasattr(obj,k)}
    if hasattr(obj,'result'):
        result['result']={k:v for k,v in obj.result.items() if k in ('success','complete','message','state','phase','run_id','attempt_id','ready_for_retry')}
    return result

class RosMotionWorker:
    def __init__(self,arm,period_s=.02):
        self.arm=arm;self.period=period_s;self.lock=threading.RLock();self.wake=threading.Event();self.quit=threading.Event()
        self.job=None;self.counter=0;self.last={'state':'IDLE','complete':False};self.cached={};self.history=[];self.next_snapshot=0.
        self.original_sleep=arm._sleep;arm._sleep=self._sleep;self.aborting=False
        self.thread=threading.Thread(target=self._run,name='atarm-single-motor-owner',daemon=True);self.thread.start()
    def submit(self,name,call):
        with self.lock:
            if self.quit.is_set():return {'success':False,'complete':False,'message':'ROS_WORKER_SHUTTING_DOWN'}
            if self.job is not None:return {'success':False,'complete':False,'message':'ROS_COMMAND_BUSY','job_id':self.job['id']}
            self.counter+=1;self.job={'id':self.counter,'name':name,'call':call,'cancel':threading.Event(),'submitted_s':time.monotonic(),'started':False}
            self.last={'job_id':self.counter,'name':name,'state':'ACCEPTED','success':False,'complete':False}
            self.wake.set();return {'success':True,'accepted':True,'complete':False,'job_id':self.counter,'message':'ROS_COMMAND_ACCEPTED'}
    def cancel(self):
        with self.lock:
            if self.job is None:return {'success':False,'message':'ROS_NO_ACTIVE_COMMAND','complete':False}
            self.job['cancel'].set();self.job.setdefault('cancel_received_s',time.monotonic());self.wake.set()
            return {'success':True,'complete':False,'message':'ROS_CANCEL_REQUESTED','job_id':self.job['id']}
    def _snapshot(self,force=False):
        now=time.monotonic()
        if not force and now<self.next_snapshot:return
        self.next_snapshot=now+.04
        arm=self.arm
        data=finite_json({'state':arm.deploy_state,'deployed':arm.deployed,'motion':arm.last_motion_result,
            'feedbacks':arm.get_feedbacks(),'targets':arm.final_target_actuation_lengths,
            'camera_feedback':arm.camera_status(),'visual_servo':component_summary(arm.visual_servo),
            'bend_visual':component_summary(arm.bend_controller),'camera_cycle':component_summary(arm.cycle),
            'camera_recovery':component_summary(arm.recovery),'recovery_probe':component_summary(arm.motion_probe)})
        with self.lock:self.cached=data
    def snapshot(self):
        with self.lock:return {**deepcopy(self.cached),'ros_job':deepcopy(self.last)}
    def _sleep(self,seconds):
        self._snapshot()
        job=self.job
        if job is not None and not self.aborting:
            if job['cancel'].wait(max(0,seconds)):
                self.aborting=True;raise RuntimeError('ROS_COMMAND_CANCELLED')
        else:self.original_sleep(seconds)
    def _stop(self,reason):
        self.arm._fault(reason)
        return [dict(self.arm.motors[i].request_zero_effort_stop(),actuator_index=i) for i in (3,0,1,2)]
    def _run(self):
        try:
            while not self.quit.is_set():
                with self.lock:job=self.job
                if job is None:
                    self.arm.step();self._snapshot();self.wake.wait(self.period);self.wake.clear();continue
                self.aborting=False;job['started']=True
                with self.lock:self.last.update({'state':'RUNNING','started_s':time.monotonic()})
                receipts=[]
                try:
                    if job['cancel'].is_set():raise RuntimeError('ROS_COMMAND_CANCELLED')
                    result=job['call']()
                    if not isinstance(result,dict):result={'success':bool(result),'complete':bool(result),'message':'ROS_COMMAND_FINISHED'}
                except Exception as exc:
                    if not job['cancel'].is_set():
                        self.aborting=True;receipts=self._stop('ROS_WORKER_COMMAND_ERROR: '+str(exc))
                    result={'success':False,'complete':False,'message':str(exc)}
                with self.lock:
                    cancelled=job['cancel'].is_set()
                    if cancelled:self.last['state']='STOPPING'
                if cancelled:
                    self.aborting=True;receipts=self._stop('ROS_COMMAND_CANCELLED')
                    result={'success':False,'complete':False,'message':'ROS_COMMAND_CANCELLED'}
                elif result.get('success') is not True and self.arm.deploy_state!='FAULT':
                    # Validation failures do not invent a successful movement.
                    result=dict(result,complete=False)
                self._snapshot(force=True)
                while True:
                    with self.lock:
                        # Atomic completion versus cancellation, with every
                        # serial operation outside the callback-facing lock.
                        late_cancel=job['cancel'].is_set() and not cancelled
                        if late_cancel:self.last['state']='STOPPING'
                        else:
                            self.last={'job_id':job['id'],'name':job['name'],'state':'CANCELLED' if cancelled else 'SUCCEEDED' if result.get('success') else 'FAILED',
                                'success':result.get('success') is True,'complete':result.get('complete') is True,'result':finite_json(result),
                                'submitted_s':job['submitted_s'],'ended_s':time.monotonic(),'cancel_received_s':job.get('cancel_received_s'),
                                'stop_receipts':receipts}
                            self.history.append(deepcopy(self.last));self.job=None
                            break
                    self.aborting=True;receipts=self._stop('ROS_COMMAND_CANCELLED')
                    result={'success':False,'complete':False,'message':'ROS_COMMAND_CANCELLED'};cancelled=True
                self._snapshot(force=True)
        except Exception as exc:
            with self.lock:self.last={'state':'WORKER_FAILED','success':False,'complete':False,'message':str(exc)}
            try:self._stop('ROS_WORKER_FAILED: '+str(exc))
            finally:self.quit.set()
        finally:
            self.aborting=True
            try:self._stop('ROS_NODE_SHUTDOWN')
            finally:self.arm.close()
    def close(self):
        self.cancel();self.quit.set();self.wake.set();self.thread.join(timeout=5.)
        if self.thread.is_alive():raise RuntimeError('ROS_WORKER_SHUTDOWN_TIMEOUT')
