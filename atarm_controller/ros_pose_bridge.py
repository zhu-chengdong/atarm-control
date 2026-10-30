"""Pair metric PoseStamped and source metadata; expose immutable capture evidence."""
from collections import OrderedDict
from dataclasses import replace
import json,math,threading,time
from .camera_feedback import CameraPose,_valid_geometry

class RosPoseBridge:
    def __init__(self,parent_frame,child_frame,map_epoch,*,max_age_s=.1,clock_slack_s=.02):
        if not all(isinstance(x,str) and x for x in (parent_frame,child_frame,map_epoch)):raise ValueError('Explicit pose frames and map epoch required')
        if any(not math.isfinite(v) or v<=0 for v in (max_age_s,clock_slack_s)):raise ValueError('Positive finite pose age and clock slack required')
        self.parent=parent_frame;self.child=child_frame;self.epoch=map_epoch;self.max_age=max_age_s;self.clock_slack=clock_slack_s
        self.lock=threading.RLock();self.poses=OrderedDict();self.meta=OrderedDict();self.latest=None;self.offset=None
        self.error=None;self.accepted=0;self.pose_callbacks=0;self.status_callbacks=0;self.duplicates=0
        self.last_stamp=None;self.last_source_id=None;self.history=[]
    def _fail(self,message):
        self.error=message;raise ValueError(message)
    def invalidate(self,message):
        with self.lock:self.error=message
    def pose(self,message,ros_now_ns,mono_now=None):
        with self.lock:
            self.pose_callbacks+=1
            if not 0<=message.header.stamp.nanosec<1000000000:self._fail('ROS_POSE_NANOSECOND_FIELD_INVALID')
            stamp=int(message.header.stamp.sec)*1000000000+int(message.header.stamp.nanosec)
            p=message.pose.position;q=message.pose.orientation
            value=(message.header.frame_id,(p.x,p.y,p.z),(q.x,q.y,q.z,q.w))
            self._insert(self.poses,stamp,value)
            self._commit(stamp,ros_now_ns,time.monotonic() if mono_now is None else mono_now)
    def status(self,message,ros_now_ns,mono_now=None):
        with self.lock:
            self.status_callbacks+=1
            try:value=json.loads(message.data)
            except Exception:self._fail('ROS_POSE_STATUS_INVALID_JSON')
            required=('stamp_ns','frame_id','tracking_ok','map_epoch','metric','child_frame')
            if not isinstance(value,dict) or any(k not in value for k in required):self._fail('ROS_POSE_STATUS_FIELDS_MISSING')
            stamp=value['stamp_ns'];frame=value['frame_id']
            if type(stamp) is not int or stamp<=0 or type(frame) is not int or frame<0:self._fail('ROS_POSE_SOURCE_ID_INVALID')
            if self.last_stamp is None or stamp>self.last_stamp:
                if value['tracking_ok'] is not True:self._fail('ROS_POSE_TRACKING_LOST')
                if value['metric'] is not True:self._fail('ROS_POSE_METRIC_SCALE_REQUIRED')
                if value['map_epoch']!=self.epoch:self._fail('ROS_POSE_MAP_EPOCH_CHANGED')
                if value['child_frame']!=self.child:self._fail('ROS_POSE_CHILD_FRAME_MISMATCH')
            self._insert(self.meta,stamp,value)
            self._commit(stamp,ros_now_ns,time.monotonic() if mono_now is None else mono_now)
    def _insert(self,pending,stamp,value):
        if self.error:return
        if stamp in pending and pending[stamp]!=value:self._fail('ROS_POSE_STAMP_MUTATED')
        pending[stamp]=value
        while len(pending)>32:pending.popitem(last=False)
    def _commit(self,stamp,ros_now,mono_now):
        if self.error or stamp not in self.poses or stamp not in self.meta:return
        parent,p,q=self.poses[stamp];m=self.meta[stamp]
        if self.last_stamp is not None and stamp<=self.last_stamp:
            if stamp==self.last_stamp and m['frame_id']==self.last_source_id:
                prior=self.latest
                if (parent,p,q)!=(prior.parent_frame,prior.position_m,prior.quaternion_xyzw):self._fail('ROS_POSE_REPLAY_MUTATED')
                self.duplicates+=1;return
            self._fail('ROS_POSE_SOURCE_TIME_REPLAY')
        if self.last_source_id is not None and m['frame_id']<=self.last_source_id:self._fail('ROS_POSE_SOURCE_FRAME_REPLAY')
        if parent!=self.parent:self._fail('ROS_POSE_PARENT_FRAME_MISMATCH')
        age=(ros_now-stamp)/1e9
        if age<0:self._fail('ROS_POSE_SOURCE_TIME_IN_FUTURE')
        if age>self.max_age:self._fail('ROS_POSE_SOURCE_STALE')
        offset=mono_now-ros_now/1e9
        if self.offset is None:self.offset=offset
        elif abs(offset-self.offset)>self.clock_slack:self._fail('ROS_POSE_CLOCK_JUMP')
        sample=CameraPose(m['frame_id'],stamp/1e9+self.offset,self.parent,self.child,self.epoch,True,p,q)
        if not _valid_geometry(sample):self._fail('ROS_POSE_GEOMETRY_INVALID')
        self.latest=sample;self.last_stamp=stamp;self.last_source_id=m['frame_id'];self.accepted+=1
        self.history.append({'source_frame_id':sample.frame_id,'stamp_ns':stamp,'capture_s':sample.capture_s,'received_s':mono_now})
        self.history=self.history[-4096:]
        for pending in (self.poses,self.meta):
            for key in list(pending):
                if key<stamp:pending.pop(key)
    def __call__(self):
        with self.lock:
            if self.error:raise ValueError(self.error)
            if self.latest is None:raise ValueError('ROS_POSE_NOT_READY')
            return self.latest
    def diagnostics(self):
        with self.lock:
            return {'accepted':self.accepted,'pose_callbacks':self.pose_callbacks,'status_callbacks':self.status_callbacks,
                'duplicates':self.duplicates,'fault':self.error,'latest_source_frame':self.last_source_id,
                'latest_capture_s':None if self.latest is None else self.latest.capture_s}
