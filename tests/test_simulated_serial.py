"""Real C++ receive path + OS pseudo-terminals + original service functions.

Only /dev/pts/* created by this process is opened. No ROS runtime, SDK serial,
physical device, network, install, or firmware operation is used.
"""
from __future__ import annotations
import argparse
from contextlib import contextmanager, redirect_stdout
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import importlib
import io
import json
import math
import os
import platform
from pathlib import Path
import select
import struct
import subprocess
import sys
import threading
import time
import traceback
import types

sys.dont_write_bytecode=True
PROJECT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PROJECT))
from atarm_controller.serial_adapter import (
    NativeSerialAdapter, NativeLibrary, MotorCmd, MotorData, MotorType, MotorMode,
    queryMotorMode, queryGearRatio)


def wire_crc(data, words):
    # Independent byte/bit implementation; verified against official codec output.
    crc=0xffffffff
    for offset in range(0,words*4,4):
        word=int.from_bytes(data[offset:offset+4],'little')
        for bit in range(31,-1,-1):
            feedback=((crc>>31)^((word>>bit)&1))&1
            crc=(crc<<1)&0xffffffff
            if feedback:
                crc^=0x04c11db7
    return crc


def frame(motor_id=2, mode=10, position_raw=0, error=0):
    result=bytearray(78)
    result[:4]=bytes((0xfe,0xee,motor_id,0))
    result[4]=mode; result[6]=25; result[7]=error
    struct.pack_into('<i',result,30,position_raw)
    struct.pack_into('<I',result,74,wire_crc(result,18))
    return bytes(result)


class Board:
    def __init__(self, timeout_s=0.05):
        self.master,slave=os.openpty()
        self.path=os.ttyname(slave)
        assert self.path.startswith('/dev/pts/')
        self.adapter=NativeSerialAdapter(slave,timeout_s)
        os.close(slave)
        self.timeout_s=timeout_s
        self.position={0:0,1:0,2:0}
        self.mode='normal'
        self.motor_modes={}
        self.forced_position_offset_raw=0
        self.bad_goal_raw=0
        self.fail_after_goal=None
        self.goal_raw=0
        self.goal_seen=False
        self.reported_offset_raw=0
        self.freeze_raw=0
        self.progress_deploy=True
        self.frozen_goal_raw=int(round((-0.05*9.1/0.026)*16384/6.2832))
        self.logs=[]; self.errors=[]
        self.late_jam=None
        self.stop=threading.Event()
        self.thread=threading.Thread(target=self._run,daemon=True)
        self.thread.start()
    def reconnect_adapter(self):
        fd=os.open(self.path,os.O_RDWR|os.O_NOCTTY)
        try:
            self.adapter=NativeSerialAdapter(fd,self.timeout_s)
        finally:
            os.close(fd)
    def preload(self, payload):
        os.write(self.master,payload)
        time.sleep(0.005)
    def _run(self):
        buffer=b''
        while not self.stop.is_set():
            try:
                ready,_,_=select.select([self.master],[],[],0.02)
                if not ready:
                    continue
                chunk=os.read(self.master,512)
                buffer+=chunk
                while len(buffer)>=34:
                    request,buffer=buffer[:34],buffer[34:]
                    assert request[:2]==b'\xfe\xee'
                    assert struct.unpack_from('<I',request,30)[0]==wire_crc(request,7)
                    motor_id=request[2]; mode=request[4]
                    target=struct.unpack_from('<i',request,16)[0]
                    kp=struct.unpack_from('<h',request,20)[0]
                    if self.late_jam is not None and kp: self.late_jam.before_command(self,motor_id,target)
                    fault=self.motor_modes.get(motor_id,self.mode)
                    if self.fail_after_goal and kp:
                        if self.goal_seen:
                            fault=self.fail_after_goal
                        elif abs(target-self.goal_raw)<=1:
                            self.goal_seen=True
                    if kp and fault in ('normal','split','noise','wrong_id','truncated','delay'):
                        self.position[motor_id]=target
                    if kp and fault=='lag_follow':
                        limit=int(0.001*9.1/0.026*16384/6.2832)
                        delta=max(-limit,min(limit,target-self.position[motor_id]))
                        self.position[motor_id]+=delta
                    if kp and fault=='external_drift':
                        self.position[motor_id]=target+self.forced_position_offset_raw
                    measured=self.position[motor_id]
                    if fault=='frozen_goal':
                        measured=self.frozen_goal_raw
                    if fault=='coherent_false' and kp:
                        measured=target
                    if fault=='drift_idle':
                        self.reported_offset_raw+=int(0.0004*9.1/0.026*16384/6.2832)
                        measured+=self.reported_offset_raw
                    if fault=='jump_under_command' and kp:
                        measured=int(-0.005*9.1/0.026*16384/6.2832)
                    if fault=='freeze_midway' and kp:
                        measured=max(self.freeze_raw,target) if self.progress_deploy else min(self.freeze_raw,target)
                    if fault=='bad_crc':
                        measured=self.bad_goal_raw
                    response=frame(motor_id,mode,measured,error=7 if fault=='motor_error' else 0)
                    if fault=='bad_crc':
                        response=response[:74]+bytes((response[74]^1,))+response[75:]
                    if fault=='wrong_id':
                        response=frame((motor_id+1)%3,mode,measured)
                    self.logs.append({'request_hex':request.hex(),'fault':fault,'motor_id':motor_id,
                        'target_raw':target,'physical_raw':self.position[motor_id],
                        'response_hex':response.hex() if fault!='timeout' else None})
                    if fault=='timeout':
                        continue
                    if fault=='delay':
                        time.sleep(self.timeout_s*2.5)
                    if fault=='truncated':
                        response=response[:30]
                    if fault=='noise':
                        response=b'\x11\x22\x00'+response
                    if fault=='split':
                        for part in (response[:1],response[1:9],response[9:31],response[31:]):
                            os.write(self.master,part)
                            time.sleep(0.0005)
                    else:
                        os.write(self.master,response)
            except OSError as exc:
                if not self.stop.is_set() and exc.errno not in (5,9):
                    self.errors.append(repr(exc))
                time.sleep(0.001)
            except Exception:
                self.errors.append(traceback.format_exc())
                return
    def close(self):
        self.stop.set()
        self.thread.join(timeout=1)
        self.adapter.close()
        os.close(self.master)
        assert not self.thread.is_alive()
        assert not self.errors,self.errors


def install_legacy_imports_and_ros_stubs():
    # Enables the pre-integration red test; final controller no longer imports it.
    sdk=types.ModuleType('unitree_actuator_sdk')
    for name,value in (('MotorCmd',MotorCmd),('MotorData',MotorData),('MotorType',MotorType),
                       ('MotorMode',MotorMode),('queryMotorMode',queryMotorMode),('queryGearRatio',queryGearRatio)):
        setattr(sdk,name,value)
    def forbidden(*args,**kwargs):
        raise AssertionError('No real SDK SerialPort or ROS runtime may start')
    sdk.SerialPort=forbidden
    sys.modules['unitree_actuator_sdk']=sdk
    ros=types.ModuleType('rclpy'); ros.init=ros.spin=ros.shutdown=forbidden
    sys.modules['rclpy']=ros
    rn=types.ModuleType('rclpy.node')
    class Node:
        __init__=forbidden
    rn.Node=Node; sys.modules['rclpy.node']=rn
    rq=types.ModuleType('rclpy.qos'); rq.QoSProfile=object; sys.modules['rclpy.qos']=rq
    for parent in ('std_msgs','std_srvs'):
        p=types.ModuleType(parent); p.__path__=[]; sys.modules[parent]=p
    class Message:
        pass
    messages=types.ModuleType('std_msgs.msg')
    messages.Bool=messages.Float64MultiArray=messages.String=Message
    sys.modules['std_msgs.msg']=messages
    srv=types.ModuleType('std_srvs.srv'); srv.Trigger=types.SimpleNamespace(Request=object,Response=object)
    sys.modules['std_srvs.srv']=srv


class Rig:
    def __init__(self):
        self.active=Board(); self.deploy=Board()
        self.arm=None; self.records=[]
    def factory(self,label):
        return {'sim://active':self.active.adapter,'sim://deploy':self.deploy.adapter}[label]
    def construct(self,guard_overrides=None,deploy_timeout_s=0.3,control_options=None):
        lib=importlib.import_module('atarm_controller.arm_controller_lib')
        options={}
        if hasattr(lib,'MotionGuardConfig'):
            options={'feedback_max_speed_m_s':4.0,'feedback_step_margin_m':0.003,
                     'uncommanded_tolerance_m':0.0015,'progress_timeout_s':0.06}
            options.update(guard_overrides or {})
        if hasattr(lib,'CameraCompletionGate'):
            options.update({'camera_required':False})  # Explicit motor-only regression/limitation cases.
            options.update(control_options or {})
        self.arm=lib.TensegrityArm('sim://active','sim://deploy',is_debug=False,
            deploy_motor_direction=-1,deploy_motor_torque_gain_scale=1.0,serial_factory=self.factory,
            deploy_timeout_s=deploy_timeout_s,deploy_period_s=0.002,deploy_speed_m_s=1.0,
            deploy_settle_s=0.006,feedback_max_age_s=0.3,**options)
        return self.arm
    def home(self):
        assert self.arm.confirm_retracted_reference(mechanically_confirmed=True)['success']
    def call(self,deploy):
        node=importlib.import_module('atarm_controller.arm_controller_node')
        obj=object.__new__(node.UnitreeTensegrityControllerNode)
        obj.arm=self.arm
        obj.get_logger=lambda:types.SimpleNamespace(info=lambda value:self.records.append(value))
        response=types.SimpleNamespace(success=False,message='')
        result=(obj._deploy_service if deploy else obj._retract_service)(None,response)
        return {'success':result.success,'message':result.message}
    def physical_mm(self):
        return -self.deploy.position[2]*6.2832/16384/9.1*0.026*1000
    def close(self):
        if self.arm is not None:
            self.arm.close()
        self.active.close(); self.deploy.close()


TRACE_ROOT=None
DETAILS={}
def save_rig(name,rig,detail):
    DETAILS[name]=detail
    (TRACE_ROOT/(name+'.json')).write_text(json.dumps({'detail':detail,
        'active_wire':rig.active.logs,'deploy_wire':rig.deploy.logs,
        'active_receive':rig.active.adapter.trace,'deploy_receive':rig.deploy.adapter.trace},indent=2)+'\n')


def integrated(name,deploy=True,fault='normal',reboot=False,deploy_first=False):
    rig=Rig()
    try:
        rig.construct(); rig.home()
        if not deploy or reboot:
            assert rig.call(True)['success']
        if reboot:
            rig.arm.close()
            rig.active.reconnect_adapter(); rig.deploy.reconnect_adapter()
            rig.construct()
            assert rig.arm.deployed is None
            if deploy_first:
                assert not rig.call(True)['success']
        if fault=='queued_cache':
            target=int(round(((-0.05 if deploy else 0)*9.1/0.026)*16384/6.2832))
            rig.deploy.mode='timeout'
            rig.deploy.preload(frame(2,10,target))
        elif fault=='cached_object':
            old=rig.deploy.adapter.last_sample
            class Cached:
                def exchange(self,command):
                    return old
                def close(self):
                    pass
            rig.arm.motors[3].serial=Cached()
        else:
            if fault=='frozen_goal':
                rig.deploy.frozen_goal_raw=int(round(((-0.05 if deploy else 0)*9.1/0.026)*16384/6.2832))
            if fault=='bad_crc':
                rig.deploy.bad_goal_raw=int(round(((-0.05 if deploy else 0)*9.1/0.026)*16384/6.2832))
            rig.deploy.mode=fault
        before=rig.physical_mm(); mark=len(rig.deploy.logs)
        previous_rx=rig.arm.motors[3].get_current_status().get('rx_monotonic_ns')
        started=time.monotonic()
        response=rig.call(deploy)
        elapsed=time.monotonic()-started
        after=rig.physical_mm()
        sample=rig.deploy.adapter.last_sample
        detail={'request':'deploy' if deploy else 'retract','fault':fault,'response':response,
            'camera_required':getattr(rig.arm,'camera_required',False),
            'before_mm':before,'after_mm':after,'state':rig.arm.deploy_state,
            'elapsed_s':elapsed,'new_tx_count':len(rig.deploy.logs)-mark,
            'last_native_status':sample.status,'last_native_frame_id':sample.transport_frame_id,
            'controller_feedback':rig.arm.motors[3].get_current_status()}
        save_rig(name,rig,detail)
        if reboot:
            assert not response['success'] and rig.arm.deploy_state=='UNKNOWN'
            assert detail['new_tx_count']==0
        elif fault=='normal':
            assert response['success'],detail
            assert abs(after-(50 if deploy else 0))<0.05,detail
        elif fault=='coherent_false':
            # Explicit protocol limitation: no device timestamp/counter exists.
            assert response['success'] and abs(after-before)<0.05,detail
            detail['known_protocol_limitation']=True
            save_rig(name,rig,detail)
        elif fault=='frozen_goal':
            assert not response['success'] and rig.arm.deploy_state=='FAULT',detail
            assert abs(after-before)<0.05,detail
        else:
            assert not response['success'] and rig.arm.deploy_state=='FAULT',detail
            assert not detail['controller_feedback']['feedback_valid'] or fault=='stall',detail
            if fault!='stall':
                assert detail['controller_feedback']['rx_monotonic_ns']==previous_rx,detail
            if fault in ('timeout','bad_crc','queued_cache','cached_object','stall'):
                assert abs(after-before)<0.05,detail
            if fault=='queued_cache':
                assert any(t['discarded_bytes']>=78 for t in rig.deploy.adapter.trace)
            if fault=='bad_crc':
                assert sample.status=='BAD_CRC'
            if fault=='cached_object':
                assert 'event' in response['message'].lower() or 'frame' in response['message'].lower()
    finally:
        rig.close()


def codec_reference():
    result=subprocess.run([str(PROJECT/'native/build/sdk_codec_probe')],capture_output=True,text=True,timeout=5,check=True)
    text=result.stdout
    assert 'gear=9.1 mode=10 command_len=34' in text
    for index,expected in ((-1,1),(30,0),(71,0),(72,1),(73,1)):
        assert ('changed=%s accepted=%s'%(index,expected)) in text
    data=bytes.fromhex(next(line.split('=',1)[1] for line in text.splitlines() if line.startswith('command_hex=')))
    assert wire_crc(data,7)==struct.unpack_from('<I',data,30)[0]
    DETAILS['official_codec_reference']={'stdout':text,'physical_sdk_serial_used':False}


def native_case(name,kind):
    board=Board(timeout_s=0.03 if kind=='delay' else 0.05)
    try:
        command=MotorCmd(id=2,mode=10)
        if kind=='identical':
            a=board.adapter.exchange(command); b=board.adapter.exchange(command)
            assert a.valid and b.valid and a.q==b.q==0
            assert b.transport_frame_id>a.transport_frame_id
            assert a.device_sequence is b.device_sequence is None
        elif kind=='drain':
            board.preload(frame(2,10,123456))
            a=board.adapter.exchange(command)
            assert a.valid and a.q==0 and a.discarded_bytes>=78
        elif kind=='closed':
            board.adapter.close()
            a=board.adapter.exchange(command)
            assert not a.valid and a.status=='CLOSED'
        elif kind=='invalid_command':
            command.q=math.nan
            a=board.adapter.exchange(command)
            assert not a.valid and a.status=='INVALID_COMMAND' and not board.logs
        else:
            board.mode=kind
            start=time.monotonic(); a=board.adapter.exchange(command); elapsed=time.monotonic()-start
            if kind in ('split','noise'):
                assert a.valid and a.actual_rx_length==78
            else:
                assert not a.valid
                assert elapsed<0.5
                if kind=='delay':
                    time.sleep(0.09)
                    count=len(board.logs)
                    b=board.adapter.exchange(command)
                    assert not b.valid and b.status=='QUARANTINED' and len(board.logs)==count
        DETAILS[name]={'trace':board.adapter.trace,'wire':board.logs}
    finally:
        board.close()


def controller_status_evidence():
    rig=Rig()
    try:
        rig.construct(); rig.home()
        node=importlib.import_module('atarm_controller.arm_controller_node')
        obj=object.__new__(node.UnitreeTensegrityControllerNode); obj.arm=rig.arm
        messages={}
        for key in ('actuation_lengths_curr_pub','motor_angles_curr_pub','actuation_lengths_trgt_pub','controller_state_pub'):
            setattr(obj,key,types.SimpleNamespace(publish=lambda msg,k=key:messages.__setitem__(k,msg.data)))
        obj._publish_status_callback()
        state=json.loads(messages['controller_state_pub'])
        assert state['device_sequence']==state['device_sample_time']==[None]*4
        assert state['freshness_basis']==['HOST_VALIDATED_SERIAL_RX']*4
        before=state['transport_frame_id']
        assert all(value>0 for value in before)
        time.sleep(0.31)
        obj._publish_status_callback()
        assert json.loads(messages['controller_state_pub'])['feedback_valid']==[False]*4
        assert all(math.isnan(value) for value in messages['actuation_lengths_curr_pub'])
        obj._step_timer_callback()
        obj._publish_status_callback()
        after=json.loads(messages['controller_state_pub'])
        assert after['feedback_valid']==[True]*4
        assert all(a>b for a,b in zip(after['transport_frame_id'],before))
        DETAILS['controller_status_evidence']={'before':state,'after_refresh':after}
    finally:
        rig.close()


def failure_after_goal(name,deploy,fault):
    rig=Rig()
    try:
        rig.construct(); rig.home()
        if not deploy:
            assert rig.call(True)['success']
        goal=int((-0.05 if deploy else 0)*9.1/0.026*16384/6.2832)
        rig.deploy.goal_raw=goal
        rig.deploy.bad_goal_raw=goal
        rig.deploy.fail_after_goal=fault
        response=rig.call(deploy)
        detail={'request':'deploy' if deploy else 'retract','fault':fault,
            'response':response,'state':rig.arm.deploy_state,'physical_after_mm':rig.physical_mm(),
            'goal_seen':rig.deploy.goal_seen,'last_native_status':rig.deploy.adapter.last_sample.status,
            'controller_feedback':rig.arm.motors[3].get_current_status()}
        save_rig(name,rig,detail)
        assert rig.deploy.goal_seen and abs(rig.physical_mm()-(50 if deploy else 0))<0.05,detail
        assert not response['success'] and rig.arm.deploy_state=='FAULT',detail
        assert detail['last_native_status']==('TIMEOUT' if fault=='timeout' else 'BAD_CRC'),detail
        assert not detail['controller_feedback']['feedback_valid'],detail
    finally:
        rig.close()


def uncommanded_drift(name,active_hold):
    rig=Rig()
    try:
        rig.construct(); rig.home()
        if active_hold:
            assert rig.call(False)['success']
        rig.deploy.mode='drift_idle'
        for _ in range(10):
            rig.arm.step()
            if rig.arm.deploy_state=='FAULT':
                break
        fault=rig.arm.motors[3].get_current_status()['motion_guard']['fault']
        detail={'state':rig.arm.deploy_state,'fault':fault,'active_hold':active_hold,
                'physical_after_mm':rig.physical_mm()}
        save_rig(name,rig,detail)
        assert rig.arm.deploy_state=='FAULT' and fault['code']=='UNCOMMANDED_DISPLACEMENT',detail
        assert abs(rig.physical_mm())<0.05
        mark=len(rig.deploy.logs)
        assert not rig.call(True)['success'] and len(rig.deploy.logs)==mark
        assert not rig.arm.motors[3].set_target_actuation_length(0.01,retry_on_timeout=True)
        assert len(rig.deploy.logs)==mark
        assert not rig.arm.confirm_retracted_reference(mechanically_confirmed=False)['success']
        assert rig.arm.motors[3].motion_guard.fault is not None
        rig.deploy.mode='normal'
        assert rig.arm.confirm_retracted_reference(mechanically_confirmed=True)['success']
        assert rig.arm.motors[3].motion_guard.fault is None
        assert rig.call(False)['success']
    finally:
        rig.close()


def discontinuity_with_issued_command():
    rig=Rig()
    try:
        rig.construct({'feedback_max_speed_m_s':0.02,'feedback_step_margin_m':0.0005})
        rig.home(); rig.deploy.mode='jump_under_command'
        lib=importlib.import_module('atarm_controller.arm_controller_lib')
        try:
            rig.arm.apply_deploy_micro_adjust(0.01)
        except lib.ControlError:
            pass
        else:
            raise AssertionError('Implausibly fast measured step was accepted')
        fault=rig.arm.motors[3].get_current_status()['motion_guard']['fault']
        detail={'state':rig.arm.deploy_state,'fault':fault,'physical_after_mm':rig.physical_mm()}
        save_rig('reject_position_discontinuity',rig,detail)
        assert fault['code']=='POSITION_DISCONTINUITY' and rig.arm.deploy_state=='FAULT',detail
        assert abs(rig.physical_mm())<0.05
    finally:
        rig.close()


def stalled_progress(name,deploy):
    rig=Rig()
    try:
        rig.construct(deploy_timeout_s=0.6); rig.home()
        if not deploy:
            assert rig.call(True)['success']
        rig.deploy.mode='freeze_midway'
        rig.deploy.freeze_raw=int(-0.025*9.1/0.026*16384/6.2832)
        rig.deploy.progress_deploy=deploy
        started=time.monotonic(); response=rig.call(deploy); elapsed=time.monotonic()-started
        detail={'state':rig.arm.deploy_state,'response':response,'elapsed_s':elapsed,
            'motion_progress':getattr(rig.arm,'motion_progress',None),
            'motion_guard_fault':rig.arm.motors[3].get_current_status()['motion_guard']['fault']}
        save_rig(name,rig,detail)
        assert not response['success'] and rig.arm.deploy_state=='FAULT',detail
        assert 'MOTION_PROGRESS_STALLED' in response['message'] and elapsed<0.6,detail
        assert detail['motion_guard_fault'] is None,detail
    finally:
        rig.close()


def legitimate_lag(name,deploy):
    rig=Rig()
    try:
        rig.construct(); rig.home()
        if not deploy:
            assert rig.call(True)['success']
        rig.deploy.mode='lag_follow'
        response=rig.call(deploy)
        detail={'response':response,'state':rig.arm.deploy_state,'after_mm':rig.physical_mm(),
            'motion_progress':rig.arm.motion_progress}
        save_rig(name,rig,detail)
        assert response['success'] and abs(rig.physical_mm()-(50 if deploy else 0))<0.05,detail
    finally:
        rig.close()


def repeated_endpoint_requests():
    rig=Rig()
    try:
        rig.construct(); rig.home()
        assert rig.call(False)['success']
        assert not rig.arm.motion_progress['required']
        assert rig.call(True)['success']
        before=rig.physical_mm()
        assert rig.call(True)['success']
        assert not rig.arm.motion_progress['required']
        assert abs(rig.physical_mm()-before)<0.05
        DETAILS['repeated_endpoint_requests']={'state':rig.arm.deploy_state,
            'motion_progress':rig.arm.motion_progress,'before_mm':before,'after_mm':rig.physical_mm()}
    finally:
        rig.close()


def run_main():
    global TRACE_ROOT
    p=argparse.ArgumentParser(); p.add_argument('--out',type=Path,required=True); p.add_argument('--case'); p.add_argument('--prefix')
    args=p.parse_args(); TRACE_ROOT=args.out.resolve()
    TRACE_ROOT.mkdir(parents=True,exist_ok=False)
    install_legacy_imports_and_ros_stubs()
    cases=[('official_codec_reference',codec_reference),('controller_status_evidence',controller_status_evidence)]
    for kind in ('identical','drain','split','noise','delay','closed','invalid_command','wrong_id','motor_error','truncated'):
        cases.append(('native_'+kind,lambda k=kind:native_case('native_'+k,k)))
    for fault in ('normal','timeout','bad_crc','queued_cache','cached_object','stall','wrong_id','truncated'):
        for deploy in (True,False):
            name=('deploy_' if deploy else 'retract_')+fault
            cases.append((name,lambda n=name,d=deploy,f=fault:integrated(n,d,f)))
    cases += [('reboot_immediate_retract',lambda:integrated('reboot_immediate_retract',False,reboot=True)),
              ('reboot_deploy_then_retract',lambda:integrated('reboot_deploy_then_retract',False,reboot=True,deploy_first=True)),
              ('reject_premature_target_feedback',lambda:integrated('reject_premature_target_feedback',True,'frozen_goal')),
              ('reject_premature_retract_feedback',lambda:integrated('reject_premature_retract_feedback',False,'frozen_goal')),
              ('limitation_coherent_false_trajectory',lambda:integrated('limitation_coherent_false_trajectory',True,'coherent_false'))]
    for deploy in (True,False):
        for fault in ('timeout','bad_crc'):
            name=('deploy_' if deploy else 'retract_')+'after_goal_'+fault
            cases.append((name,lambda n=name,d=deploy,f=fault:failure_after_goal(n,d,f)))
    cases += [('uncommanded_passive_drift',lambda:uncommanded_drift('uncommanded_passive_drift',False)),
              ('uncommanded_hold_drift',lambda:uncommanded_drift('uncommanded_hold_drift',True)),
              ('reject_position_discontinuity',discontinuity_with_issued_command),
              ('repeated_endpoint_requests',repeated_endpoint_requests)]
    for deploy in (True,False):
        prefix='deploy_' if deploy else 'retract_'
        cases.append((prefix+'progress_freeze',lambda d=deploy,n=prefix+'progress_freeze':stalled_progress(n,d)))
        cases.append((prefix+'legitimate_lag',lambda d=deploy,n=prefix+'legitimate_lag':legitimate_lag(n,d)))
    import camera_cases
    cases += camera_cases.cases(sys.modules[__name__])
    import servo_cases
    cases += servo_cases.cases(sys.modules[__name__])
    import adaptive_cases
    cases += adaptive_cases.cases(sys.modules[__name__])
    import bend_cases
    cases += bend_cases.cases(sys.modules[__name__])
    import bend_model_cases
    cases += bend_model_cases.cases(sys.modules[__name__])
    import cycle_cases
    cases += cycle_cases.cases(sys.modules[__name__])
    cases += [('cycle_repeated_scalar_hold',lambda:cycle_cases.repeated_endpoint_hold(sys.modules[__name__]))]
    import recovery_cases
    cases += recovery_cases.cases(sys.modules[__name__])
    import probe_cases
    cases += probe_cases.cases(sys.modules[__name__])
    import late_jam_cases
    cases += late_jam_cases.cases(sys.modules[__name__])
    import intermittent_cases
    cases += intermittent_cases.cases(sys.modules[__name__])
    import window_cases
    cases += window_cases.cases(sys.modules[__name__])
    import transient_cases
    cases += transient_cases.cases(sys.modules[__name__])
    import pause_budget_cases
    cases += pause_budget_cases.cases(sys.modules[__name__])
    import repeated_pause_cases
    cases += repeated_pause_cases.cases(sys.modules[__name__])
    import wide400_cases
    cases += wide400_cases.cases(sys.modules[__name__])
    import early_progress_cases
    cases += early_progress_cases.cases(sys.modules[__name__])
    import confirm_pause_cases
    cases += confirm_pause_cases.cases(sys.modules[__name__])
    import budget_cases
    cases += budget_cases.cases(sys.modules[__name__])
    if args.case:
        cases=[pair for pair in cases if pair[0]==args.case]
        assert cases,'Unknown test case'
    if args.prefix:
        cases=[pair for pair in cases if pair[0].startswith(args.prefix)]
        assert cases,'Unknown test prefix'
    results=[]; captured=io.StringIO()
    with redirect_stdout(captured):
        for name,action in cases:
            try:
                action(); results.append({'case':name,'passed':True,
                    'category':'known_protocol_limitation' if name.startswith('limitation_') else 'acceptance'})
            except Exception:
                results.append({'case':name,'passed':False,'category':'known_protocol_limitation' if name.startswith('limitation_') else 'acceptance','error':traceback.format_exc()})
    report={'verified_at_utc':datetime.now(timezone.utc).isoformat(),'cases':results,'details':DETAILS,
        'summary':{'count':len(results),'passed':sum(r['passed'] for r in results),
            'acceptance_cases':sum(r['category']=='acceptance' for r in results),
            'acceptance_passed':sum(r['category']=='acceptance' and r['passed'] for r in results),
            'known_protocol_limitations':sum(r['category']=='known_protocol_limitation' for r in results),
            'unsafe_success_observed_in_limit_case':DETAILS.get('limitation_coherent_false_trajectory',{}).get('known_protocol_limitation',False)},
        'environment':{'os_serial':'Linux OS pseudo-terminals','native_cpp_receive_path':True,
            'python_version':sys.version,'platform':platform.platform(),
            'sdk_codec_executed_offline':True,'sdk_serial_constructed':False,'ros_runtime_started':False,
            'physical_device_opened':False,'device_counter_simulated':False,
            'native_library':str(NativeLibrary.shared().path)},
        'harness_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'executed_source_sha256':{str(p.relative_to(PROJECT)):hashlib.sha256(p.read_bytes()).hexdigest()
            for folder in ('atarm_controller','tests') for p in sorted((PROJECT/folder).glob('*.py'))},
        'native_binary_sha256':{str(p.relative_to(PROJECT)):hashlib.sha256(p.read_bytes()).hexdigest()
            for folder in ('native/build','native/vendor/lib') for p in sorted((PROJECT/folder).glob('*')) if p.is_file()}}
    (TRACE_ROOT/'results.json').write_text(json.dumps(report,indent=2)+'\n')
    (TRACE_ROOT/'stdout.txt').write_text(captured.getvalue())
    print(json.dumps({'summary':report['summary'],'cases':results},indent=2))
    return 0 if all(r['passed'] for r in results) else 1


if __name__=='__main__':
    raise SystemExit(run_main())
