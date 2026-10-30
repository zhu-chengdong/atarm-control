"""Focused source-contract and command-ownership checks, separate from DDS tests."""
import argparse,json,sys,threading,time
from pathlib import Path
from types import SimpleNamespace as S
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from atarm_controller.ros_pose_bridge import RosPoseBridge
from atarm_controller.ros_motion_worker import RosMotionWorker
def pose(stamp=1000000000,parent='map',position=(0.,0.,0.)):
    return S(header=S(stamp=S(sec=stamp//1000000000,nanosec=stamp%1000000000),frame_id=parent),
        pose=S(position=S(x=position[0],y=position[1],z=position[2]),orientation=S(x=0.,y=0.,z=0.,w=1.)))
def meta(stamp=1000000000,frame=1,**changes):
    d=dict(stamp_ns=stamp,frame_id=frame,tracking_ok=True,map_epoch='epoch',metric=True,child_frame='camera');d.update(changes)
    return S(data=json.dumps(d))
def bridge():return RosPoseBridge('map','camera','epoch',max_age_s=.1)
rows=[]
def check(name,fn):
    fn();rows.append({'case':name,'passed':True})
def pair(b,stamp=1000000000,frame=1,now=1000000000,mono=20.,**changes):
    b.pose(pose(stamp),now,mono);b.status(meta(stamp,frame,**changes),now,mono)
def rejected(name,fn):
    try:fn()
    except ValueError:return
    raise AssertionError(name+' unexpectedly accepted')
def good():
    b=bridge();b.status(meta(),1000000000,20.);b.pose(pose(),1000000000,20.)
    assert b().capture_s==20. and b.accepted==1
    pair(b);assert b.accepted==1 and b.duplicates>0
    pair(b,1010000000,2,1010000000,20.01);assert b.accepted==2
check('paired_capture_time_and_duplicate_cache',good)
for name,changes in [('tracking_lost',{'tracking_ok':False}),('epoch_reset',{'map_epoch':'new'}),
                     ('nonmetric',{'metric':False}),('wrong_child',{'child_frame':'bad'})]:
    check(name,lambda n=name,c=changes:rejected(n,lambda:pair(bridge(),**c)))
check('stale_source',lambda:rejected('stale',lambda:pair(bridge(),now=1200000000)))
check('future_source',lambda:rejected('future',lambda:pair(bridge(),now=900000000)))
def missing():
    b=bridge();b.pose(pose(),1000000000,20.);rejected('missing companion',b)
check('missing_companion_not_fabricated',missing)
def replay():
    b=bridge();pair(b);rejected('source frame replay',lambda:pair(b,1010000000,1,1010000000,20.01))
check('source_frame_replay',replay)
def clock_jump():
    b=bridge();pair(b);rejected('clock jump',lambda:pair(b,1010000000,2,1010000000,21.))
check('clock_jump',clock_jump)
def wrong_parent():
    b=bridge();b.status(meta(),1000000000,20.);rejected('wrong parent',lambda:b.pose(pose(parent='bad'),1000000000,20.))
check('wrong_parent_frame',wrong_parent)
def mutated():
    b=bridge();pair(b);rejected('mutated',lambda:b.pose(pose(position=(.1,0.,0.)),1000000000,20.))
check('mutated_duplicate',mutated)
def nonfinite():
    b=bridge();b.status(meta(),1000000000,20.);rejected('nonfinite',lambda:b.pose(pose(position=(float('nan'),0.,0.)),1000000000,20.))
check('invalid_geometry',nonfinite)

class Motor:
    def __init__(self,arm,index):self.arm=arm;self.index=index
    def request_zero_effort_stop(self):
        self.arm.energized=False;self.arm.stops.append((self.index,threading.current_thread().name));return {'reply_valid':True,'transport_frame_id':len(self.arm.stops)}
class Arm:
    def __init__(self):
        self._sleep=time.sleep;self.deploy_state='RETRACTED';self.deployed=False;self.last_motion_result={};self.final_target_actuation_lengths=[0.]*4
        self.visual_servo=self.bend_controller=self.motion_probe=None
        self.cycle=self.recovery=S(active=False,state='IDLE',phase='IDLE',result={});self.stops=[];self.energized=False
        self.motors=[Motor(self,i) for i in range(4)]
    def step(self):return False
    def camera_status(self):return {}
    def get_feedbacks(self):return []
    def _fault(self,reason):self.deploy_state='FAULT'
    def close(self):pass
def done(w):
    until=time.monotonic()+2.
    while time.monotonic()<until:
        d=w.snapshot().get('ros_job',{})
        if d.get('state') in ('FAILED','CANCELLED','SUCCEEDED'):return d
        time.sleep(.001)
    raise AssertionError('Worker did not finish')
def crash():
    a=Arm();w=RosMotionWorker(a)
    def operation():a.energized=True;raise RuntimeError('injected command error')
    try:
        assert w.submit('crash',operation)['complete'] is False
        d=done(w);assert d['state']=='FAILED' and a.deploy_state=='FAULT' and not a.energized and len(d['stop_receipts'])==4
    finally:w.close()
check('unexpected_command_exception_stops_all_motors',crash)
def cancelled(late=False,shutdown=False):
    a=Arm();w=RosMotionWorker(a);entered=threading.Event();release=threading.Event()
    def operation():
        a.energized=True;entered.set()
        if late:release.wait(1.)
        else:a._sleep(.5)
        return {'success':True,'complete':True}
    w.submit('motion',operation);assert entered.wait(1.)
    assert not w.submit('overlap',lambda:None)['success']
    if shutdown:w.close();assert not w.thread.is_alive() and not a.energized and len(a.stops)>=4;return
    try:
        assert w.cancel()['success'];release.set();d=done(w)
        assert d['state']=='CANCELLED' and not d['complete'] and not a.energized and len(d['stop_receipts'])==4
    finally:w.close()
check('cancellation_and_overlapping_command_rejection',cancelled)
check('cancellation_wins_before_completion_commit',lambda:cancelled(late=True))
check('shutdown_cancels_and_stops',lambda:cancelled(shutdown=True))
p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);args=p.parse_args();args.out.mkdir()
(args.out/'results.json').write_text(json.dumps({'scope':'pure adapter and worker checks, no ROS runtime','count':len(rows),'passed':len(rows),'cases':rows},indent=2)+'\n')
print(json.dumps({'count':len(rows),'passed':len(rows)}))
