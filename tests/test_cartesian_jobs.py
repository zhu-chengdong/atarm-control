"""Local job-protocol and concurrency checks, explicitly without ROS stubs."""
import json
from pathlib import Path
import sys
import threading
import time
import unittest

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from atarm_controller.cartesian_jobs import CartesianJobRunner


def payload(r, name='goal_1', **options):
    d = dict(zip(('parent_frame', 'child_frame', 'map_epoch'), r.reference))
    d.update(request_id=name, position_m=[.3124208, .03748, .1949592]); d.update(options)
    return json.dumps(d)


def wait(r, timeout=3.):
    start = time.monotonic()
    while time.monotonic()-start < timeout:
        d = r.snapshot()
        if d['job']['state'] in ('SUCCEEDED', 'FAULT', 'CANCELLED'):
            return d
        time.sleep(.002)
    raise AssertionError('Worker did not finish')


class JobTests(unittest.TestCase):
    def make(self, **kw):
        r = CartesianJobRunner(paced=False, **kw); self.addCleanup(r.close)
        return r

    def test_acceptance_is_not_completion(self):
        r = self.make(); ack = r.submit_json(payload(r))
        self.assertTrue(ack['accepted']); self.assertFalse(ack['complete'])
        s = wait(r); self.assertTrue(s['job']['complete']); self.assertLess(s['job']['simulation_truth_error_m'], .0005)
        self.assertEqual(s['job']['io_threads'], ['cartesian-sim-owner'])

    def test_invalid_schema_and_reference(self):
        r = self.make()
        for msg in ('null', '[]', '{', '{"request_id":"nan","position_m":[NaN,0,0]}',
                    payload(r, 'x', position_m=[True, 0, 0]), payload(r, 'x', extra=1), payload(r, 'x', map_epoch='old')):
            self.assertFalse(r.submit_json(msg)['accepted'])
        self.assertFalse(r.plant.commands)
        ack = r.submit_json(payload(r, 'nan_correlated', position_m=[float('nan'), 0, 0]))
        self.assertFalse(ack['accepted']); self.assertEqual(ack['request_id'], 'nan_correlated')

    def test_duplicate_busy_reset_and_cancel(self):
        r = self.make(); reached = threading.Event(); release = threading.Event()
        def block_once():
            if not reached.is_set():
                reached.set(); release.wait(1.)
        r.plant.on_command = block_once
        self.assertTrue(r.submit_json(payload(r))['accepted']); self.assertTrue(reached.wait(1.))
        self.assertEqual(r.submit_json(payload(r))['reason'], 'ROS_CARTESIAN_DUPLICATE_ID')
        self.assertEqual(r.submit_json(payload(r, 'new'))['reason'], 'ROS_CARTESIAN_BUSY')
        self.assertFalse(r.reset()['accepted']); self.assertTrue(r.cancel()['accepted']); release.set()
        d = wait(r); self.assertEqual(d['job']['state'], 'CANCELLED'); self.assertFalse(d['job']['complete'])
        self.assertEqual(r.submit_json(payload(r, 'retry'))['reason'], 'ROS_CARTESIAN_RESET_REQUIRED')
        self.assertTrue(r.reset()['accepted']); self.assertEqual(r.reference[2], 'sim_map_1')
        self.assertTrue(r.submit_json(payload(r, 'after_reset'))['accepted']); self.assertTrue(wait(r)['job']['complete'])

    def test_fault_latches(self):
        for fault in ('tracking_lost', 'frozen_pose', 'map_reset', 'motor_stall'):
            with self.subTest(fault=fault):
                r = self.make(fault=fault); r.submit_json(payload(r)); d = wait(r)
                self.assertEqual(d['job']['state'], 'FAULT'); self.assertTrue(d['needs_reset'])
                self.assertFalse(d['job']['complete']); self.assertEqual(len(d['job']['result']['stop_receipts']), 4)

    def test_shutdown_joins_running_worker(self):
        r = CartesianJobRunner(simulation_speed=1.)
        r.submit_json(payload(r)); time.sleep(.03); r.close()
        self.assertFalse(r.thread.is_alive()); self.assertTrue(r.plant.stopped)

    def test_core_has_no_hardware_import(self):
        self.assertNotIn('atarm_controller.serial_adapter', sys.modules)
        self.assertNotIn('atarm_controller.arm_controller_lib', sys.modules)
        self.assertNotIn('rclpy', sys.modules)


if __name__ == '__main__':
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(JobTests)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    if len(sys.argv) > 1:
        Path(sys.argv[1]).write_text(json.dumps({'scope': 'pure job runner, no ROS middleware', 'test_methods': result.testsRun,
            'passed': result.wasSuccessful(), 'failures': [(str(t), e) for t, e in result.failures],
            'errors': [(str(t), e) for t, e in result.errors]}, indent=2), encoding='utf-8')
    sys.exit(0 if result.wasSuccessful() else 1)
