"""Metric external pose contract and explicit, epoch-bound rigid transforms.

Does not run SLAM, estimate a scale or silently query a TF tree. T_a_b maps
b coordinates into a. Source timestamps and IDs survive both transforms.
"""
from dataclasses import replace
import math
from .ros_pose_bridge import RosPoseBridge


def vector(x,n):
    if len(x)!=n or any(type(v) not in (int,float) or not math.isfinite(v) for v in x):
        raise ValueError('VISION_INVALID_TRANSFORM')
    return tuple(x)


def quaternion(x):
    q=vector(x,4)
    if abs(sum(v*v for v in q)-1)>1e-6:raise ValueError('VISION_NONUNIT_TRANSFORM')
    return q


def multiply(a,b):
    x,y,z,w=a;X,Y,Z,W=b
    return (w*X+x*W+y*Z-z*Y,w*Y-x*Z+y*W+z*X,w*Z+x*Y-y*X+z*W,w*W-x*X-y*Y-z*Z)


def rotate(q,v):
    return multiply(multiply(q,(*v,0.)),(-q[0],-q[1],-q[2],q[3]))[:3]


class ExternalVision:
    def __init__(self,config,max_age_s):
        self.config=dict(config)
        required=('source_parent','camera_child','target_parent','control_child','map_epoch',
                  'base_from_source_translation','base_from_source_quaternion',
                  'camera_from_control_translation','camera_from_control_quaternion')
        if any(k not in config for k in required):raise ValueError('VISION_CONFIG_INCOMPLETE')
        if any(not isinstance(config[k],str) or not config[k] for k in required[:5]):
            raise ValueError('VISION_REFERENCE_INVALID')
        self.base_t=vector(config['base_from_source_translation'],3)
        self.base_q=quaternion(config['base_from_source_quaternion'])
        self.tip_t=vector(config['camera_from_control_translation'],3)
        self.tip_q=quaternion(config['camera_from_control_quaternion'])
        self.reference=(config['target_parent'],config['control_child'],config['map_epoch'])
        self.max_age_s=max_age_s;self.startup_stale_drops=0
        self.bridge=RosPoseBridge(config['source_parent'],config['camera_child'],config['map_epoch'],max_age_s=max_age_s)

    def startup_stale(self,stamp,ros_now):
        # Reliable DDS discovery can deliver old history before the first fresh
        # pair. Before ANY accepted frame, drop history without creating a pose
        # or clearing a fault. After acquisition, every source fault latches.
        if self.bridge.accepted==0 and self.bridge.error is None and type(stamp) is int and stamp>0 and (ros_now-stamp)/1e9>self.max_age_s:
            self.startup_stale_drops+=1;return True
        return False

    def pose(self,message,ros_now_ns,mono_now=None):
        stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
        if not self.startup_stale(stamp,ros_now_ns):self.bridge.pose(message,ros_now_ns,mono_now)

    def status(self,message,ros_now_ns,mono_now=None):
        import json
        try:stamp=json.loads(message.data).get('stamp_ns')
        except (ValueError,AttributeError):stamp=None
        if not self.startup_stale(stamp,ros_now_ns):self.bridge.status(message,ros_now_ns,mono_now)

    def __call__(self):
        p=self.bridge()
        # RosPoseBridge checks geometry, but normalize within its accepted
        # quaternion tolerance before using the rotation as a rigid transform.
        n=math.sqrt(sum(x*x for x in p.quaternion_xyzw));q=tuple(x/n for x in p.quaternion_xyzw)
        tip=rotate(q,self.tip_t)
        source_point=tuple(x+y for x,y in zip(p.position_m,tip))
        turned=rotate(self.base_q,source_point)
        return replace(p,parent_frame=self.reference[0],child_frame=self.reference[1],
                       position_m=tuple(x+y for x,y in zip(self.base_t,turned)),
                       quaternion_xyzw=multiply(multiply(self.base_q,q),self.tip_q))

    def diagnostics(self):return dict(self.bridge.diagnostics(),startup_stale_drops=self.startup_stale_drops)
