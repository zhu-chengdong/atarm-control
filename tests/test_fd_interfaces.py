"""Offline contract checks without importing ROS or loading the native library."""
import json,math,os,sys,tempfile,threading,time,unittest
from pathlib import Path
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from atarm_controller.external_vision import ExternalVision
from atarm_controller.actuator_io import ChannelCalibration,FdActuatorIO,PtyFdFactory
from atarm_controller.cartesian_control import CartesianController
from atarm_controller.cartesian_simulation import simulation_config
from atarm_controller.serial_adapter import FeedbackSample
from atarm_controller.camera_feedback import CameraPose
from atarm_controller.cartesian_fd_runtime import FdJobRunner


def vision_config():
    return dict(source_parent='map',camera_child='camera',target_parent='base',control_child='tcp',map_epoch='epoch',
        base_from_source_translation=[1,2,3],base_from_source_quaternion=[0,0,math.sin(math.pi/4),math.cos(math.pi/4)],
        camera_from_control_translation=[.02,0,0],camera_from_control_quaternion=[0,0,0,1])


def messages(stamp=1_000_000_000,frame=1,**options):
    pose=NS(header=NS(stamp=NS(sec=stamp//10**9,nanosec=stamp%10**9),frame_id='map'),
        pose=NS(position=NS(x=.1,y=0.,z=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.)))
    meta=dict(stamp_ns=stamp,frame_id=frame,tracking_ok=True,metric=True,child_frame='camera',map_epoch='epoch');meta.update(options)
    return pose,NS(data=json.dumps(meta))


class Transport:
    def __init__(self):self.counter=0;self.angle={};self.calls=[];self.bad=False;self.closed=False
    def exchange(self,cmd):
        self.calls.append(cmd);self.counter+=1
        if cmd.kp:self.angle[cmd.id]=cmd.q
        now=time.monotonic_ns()
        return FeedbackSample(not self.bad,'BAD_CRC' if self.bad else 'OK',transport_frame_id=self.counter,
            tx_ns=now-1000,rx_ns=now,motor_id=cmd.id,mode=cmd.mode,q=self.angle.get(cmd.id,0.),dq=0.,tau=0.,
            temp=25,merror=0,crc_ok=not self.bad)
    def close(self):self.closed=True


def channels():
    a=[ChannelCalibration('active',i,s,.026,9.1,0.,0.,.06,.08,5.,60.) for i,s in zip((1,0,2),(1,1,-1))]
    return a+[ChannelCalibration('deploy',2,-1,.026,9.1,0.,.04,.06,.08,5.,60.)]


class Contracts(unittest.TestCase):
    def test_transform_preserves_capture_and_ids(self):
        v=ExternalVision(vision_config(),.15);p,m=messages();v.status(m,1_010_000_000,2.01);v.pose(p,1_010_000_000,2.01)
        q=v();self.assertAlmostEqual(q.position_m[0],1);self.assertAlmostEqual(q.position_m[1],2.12)
        self.assertEqual(q.frame_id,1);self.assertAlmostEqual(q.capture_s,2.);self.assertEqual(q.parent_frame,'base')
        self.assertEqual(q.child_frame,'tcp');self.assertEqual(q.map_epoch,'epoch')

    def test_missing_pair_never_creates_observation(self):
        v=ExternalVision(vision_config(),.15);p,_=messages();v.pose(p,1_010_000_000,2.01)
        with self.assertRaisesRegex(ValueError,'NOT_READY'):v()

    def test_source_contract_faults_latch(self):
        for fields in ({'tracking_ok':False},{'metric':False},{'map_epoch':'other'},{'child_frame':'other'}):
            v=ExternalVision(vision_config(),.15);p,m=messages(**fields)
            with self.assertRaises(ValueError):v.status(m,1_010_000_000,2.01)
            with self.assertRaises(ValueError):v()

    def test_stale_replay_and_wrong_parent(self):
        for kind in ('stale','wrong_parent','replay'):
            v=ExternalVision(vision_config(),.15);p,m=messages()
            if kind=='wrong_parent':p.header.frame_id='other'
            if kind in ('replay','stale'):
                v.pose(p,1_010_000_000,2.01);v.status(m,1_010_000_000,2.01)
                p,m=messages(stamp=900_000_000,frame=0) if kind=='replay' else messages(stamp=1_020_000_000,frame=2)
            with self.assertRaises(ValueError):
                now=1_200_000_000 if kind=='stale' else 1_010_000_000
                v.pose(p,now,2.01);v.status(m,now,2.01)

    def test_startup_history_dropped_without_arming_or_fault_reset(self):
        v=ExternalVision(vision_config(),.15);p,m=messages()
        v.pose(p,1_200_000_000,2.2);v.status(m,1_200_000_000,2.2)
        self.assertEqual(v.diagnostics()['accepted'],0)
        with self.assertRaisesRegex(ValueError,'NOT_READY'):v()
        p,m=messages(stamp=1_210_000_000,frame=2)
        v.pose(p,1_220_000_000,2.22);v.status(m,1_220_000_000,2.22)
        self.assertEqual(v().frame_id,2);self.assertEqual(v.diagnostics()['startup_stale_drops'],2)

    def test_bad_transform_and_mapping_rejected(self):
        c=vision_config();c['base_from_source_quaternion']=[0,0,0,0]
        with self.assertRaises(ValueError):ExternalVision(c,.15)
        with self.assertRaises(ValueError):ChannelCalibration('active',1,0,.026,9.1,0.,0.,.06,.08,5.,60.)

    def test_passive_start_and_calibrated_conversion(self):
        buses={'active':Transport(),'deploy':Transport()};cfg=simulation_config()
        io=FdActuatorIO(cfg,channels(),lambda:buses,lambda:None,speed_m_s=.04)
        evidence=io.poll_motors();self.assertTrue(all(c.kp==c.kd==0 for t in buses.values() for c in t.calls))
        io.arm_reference(evidence);self.assertTrue(io.command((.001,-.0005,-.0005,.041)))
        for _ in range(5):time.sleep(.012);e=io.poll_motors()
        for x,y in zip(e.positions_m,(.001,-.0005,-.0005,.041)):self.assertAlmostEqual(x,y,places=7)
        self.assertGreater(buses['active'].angle[2],0)  # sign=-1, negative length => positive motor angle
        io.stop();io.close();self.assertTrue(all(t.closed for t in buses.values()))

    def test_feedback_failure_is_not_old_success(self):
        buses={'active':Transport(),'deploy':Transport()};io=FdActuatorIO(simulation_config(),channels(),lambda:buses,lambda:None,speed_m_s=.04)
        io.poll_motors();buses['active'].bad=True
        with self.assertRaisesRegex(RuntimeError,'BAD_CRC'):io.poll_motors()
        receipt=io.stop();self.assertEqual(len(receipt),4);self.assertFalse(all(x['reply_valid'] for x in receipt));io.close()

    def test_owner_and_reference_are_not_guessed(self):
        buses={'active':Transport(),'deploy':Transport()};io=FdActuatorIO(simulation_config(),channels(),lambda:buses,lambda:None,speed_m_s=.04)
        buses['active'].angle[1]=1.
        with self.assertRaisesRegex(ValueError,'REFERENCE_MISMATCH'):io.arm_reference(io.poll_motors())
        errors=[]
        def other():
            try:io.poll_motors()
            except Exception as e:errors.append(str(e))
        t=threading.Thread(target=other);t.start();t.join();self.assertEqual(errors,['ACTUATOR_IO_OWNER_VIOLATION']);io.stop();io.close()

    def test_unknown_domain_stays_disabled(self):
        class Unknown:execution_domain='physical'
        with self.assertRaisesRegex(ValueError,'PHYSICAL_IO_DISABLED'):CartesianController(simulation_config(),Unknown())

    def test_reference_window_waits_without_reset_or_motion(self):
        class Vision:
            reference=('base','tcp','epoch')
            def __call__(self):
                now=time.monotonic();frame=int(now/.02)
                return CameraPose(frame,frame*.02,'base','tcp','epoch',True,(.3,.05,.18),(0,0,0,1))
            def diagnostics(self):return {}
        buses={'active':Transport(),'deploy':Transport()}
        runner=FdJobRunner(simulation_config(),channels(),lambda:buses,Vision(),speed_m_s=.04,calibration_id='unit-test')
        try:
            self.assertTrue(runner.confirm_reference()['accepted'])
            end=time.monotonic()+2.
            while not runner.snapshot()['ready'] and time.monotonic()<end:time.sleep(.01)
            self.assertTrue(runner.snapshot()['ready'],runner.snapshot())
            self.assertTrue(all(abs(c.q)<1e-12 for b in buses.values() for c in b.calls))
        finally:runner.close()
        self.assertEqual(runner.audit['io_threads'],['cartesian-fd-owner'])

    def test_worker_initialization_failure_clears_pending_and_rejects(self):
        entered=threading.Event();release=threading.Event()
        class Vision:
            reference=('base','tcp','epoch')
            def diagnostics(self):return {}
        def broken_factory():
            entered.set();release.wait(1.);raise RuntimeError('injected factory failure')
        runner=FdJobRunner(simulation_config(),channels(),broken_factory,Vision(),speed_m_s=.04,calibration_id='unit-test')
        self.assertTrue(entered.wait(1.));self.assertTrue(runner.confirm_reference()['accepted']);release.set()
        runner.thread.join(2.)
        try:
            self.assertIsNone(runner.pending)
            self.assertIn('injected factory failure',runner.snapshot()['fault'])
            g={'request_id':'after_failure','position_m':[.3,.05,.18],'parent_frame':'base','child_frame':'tcp','map_epoch':'epoch'}
            self.assertEqual(runner.submit_json(json.dumps(g))['reason'],'EXPLICIT_RESTART_REQUIRED')
        finally:runner.close()


if __name__=='__main__':unittest.main()
