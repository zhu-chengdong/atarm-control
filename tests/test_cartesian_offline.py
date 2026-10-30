"""Independent truth checks for the new controller; no SDK/ROS/device imports."""
import json
import math
from pathlib import Path
import sys
import unittest
from dataclasses import replace

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from atarm_controller.cartesian_control import CartesianController, PositionGoal
from atarm_controller.cartesian_simulation import SimulatedPlant, simulation_config
from atarm_controller.cartesian_math import norm, sub, singular_values, least_squares


RESULTS = []


def goal(plant, u=(.012, -.008, .014), **kwargs):
    return PositionGoal(plant.forward(u), 'sim_base', 'camera_optical', 'sim_map_0', **kwargs)


def run_case(name, *, u=(.012, -.008, .014), config=None, plant=None, expect_success=True, expected_reasons=None, setup=None):
    p = plant or SimulatedPlant()
    c = CartesianController(config or simulation_config(), p)
    g = goal(p, u)
    if setup is not None:
        g = setup(c, p, g) or g
    result = c.move_position(g)
    true_error = norm(sub(p.true_position(), g.position_m))
    assert result['success'] is expect_success, (name, result)
    assert result['complete'] is expect_success, (name, result)
    if expect_success:
        assert true_error <= c.cfg.position_tolerance_m, (name, 'independent truth missed target', true_error)
        assert p.stop_calls == 0
    else:
        assert result['reason'] in expected_reasons, (name, result)
        assert p.stop_calls == 1
        count = len(p.commands)
        rerun = c.move_position(g)
        assert not rerun['success'] and rerun['reason'] == 'CARTESIAN_EXPLICIT_RECOVERY_REQUIRED'
        assert len(p.commands) == count, 'Commands sent after fault'
    assert all(all(lo-1e-12 <= q <= hi+1e-12 for lo, q, hi in zip(c.cfg.min_lengths_m, row['target_q'], c.cfg.max_lengths_m)) for row in p.commands)
    RESULTS.append({'name': name, 'expected_success': expect_success, 'result': result,
        'independent_truth_error_m': true_error, 'goal_position_m': g.position_m, 'end_position_m': p.true_position(),
        'steps': len(c.history), 'simulated_elapsed_s': p.now()-10., 'status': c.status(), 'commands': p.commands})
    return c, p, result


class CartesianTests(unittest.TestCase):
    def test_linear_target_requires_both_blocks(self):
        c, _, _ = run_case('linear_alternating', plant=SimulatedPlant(nonlinear=False))
        self.assertEqual({e['block'] for e in c.history}, {'bend', 'extend'})
        for h in c.history:
            self.assertEqual(h['du'][2] == 0, h['block'] == 'bend')

    def test_nonlinear_targets(self):
        for i, u in enumerate(((.012, -.008, .014), (-.016, .009, .018), (.015, .014, -.012), (-.012, -.015, -.015))):
            with self.subTest(u=u):
                run_case('nonlinear_alternating_'+str(i), u=u)

    def test_simultaneous_four_motor_allocation(self):
        c, _, _ = run_case('nonlinear_coordinated', config=simulation_config(strategy='coordinated'))
        self.assertEqual({x['block'] for x in c.history}, {'coordinated'})
        self.assertTrue(any(abs(x['du'][2]) > 0 and norm(x['du'][:2]) > 0 for x in c.history))

    def test_noise_and_model_mismatch(self):
        for seed in (7, 19, 41):
            with self.subTest(seed=seed):
                run_case('noise_'+str(seed), plant=SimulatedPlant(noise_m=.00003, seed=seed))

    def test_disabled_learning_comparison(self):
        run_case('fixed_jacobian_comparison', config=simulation_config(online_update=False))

    def test_multi_goal_model_reuse(self):
        p = SimulatedPlant(); c = CartesianController(simulation_config(), p)
        for i, u in enumerate(((.010, .008, .010), (-.012, .008, .015), (.008, -.012, -.010))):
            g = goal(p, u); result = c.move_position(g)
            self.assertTrue(result['success'], result)
            self.assertLessEqual(norm(sub(p.true_position(), g.position_m)), c.cfg.position_tolerance_m)
        RESULTS.append({'name': 'multi_goal_sequence', 'expected_success': True, 'steps': len(p.commands), 'result': result,
                        'independent_truth_error_m': norm(sub(p.true_position(), g.position_m)), 'status': c.status()})

    def test_physical_io_rejected(self):
        p = SimulatedPlant(); p.execution_domain = 'physical'
        with self.assertRaisesRegex(ValueError, 'PHYSICAL_IO_DISABLED'):
            CartesianController(simulation_config(), p)
        self.assertFalse(p.commands)

    def test_singular_initial_model_rejected(self):
        with self.assertRaisesRegex(ValueError, 'MODEL_RANK_OR_CONDITION'):
            simulation_config(initial_jacobian=((1., 0., 0.), (0., 1., 0.), (0., 0., 0.)))

    def test_unreachable_target(self):
        run_case('unreachable_target', u=(.12, .10, .13), expect_success=False,
                 expected_reasons={'CARTESIAN_NO_FEASIBLE_DESCENT_STEP', 'CARTESIAN_NO_PROGRESS', 'CARTESIAN_ITERATION_LIMIT'})

    def test_faults(self):
        reasons = {
            'frozen_pose': {'CARTESIAN_POSE_STALE_OR_FUTURE'},
            'tracking_lost': {'CARTESIAN_TRACKING_LOST'},
            'map_reset': {'CARTESIAN_REFERENCE_CHANGED'},
            'pose_jump': {'CARTESIAN_POSE_DISCONTINUITY'},
            'motor_invalid': {'CARTESIAN_MOTOR_FEEDBACK_INVALID'},
            'motor_stale': {'CARTESIAN_MOTOR_FEEDBACK_STALE'},
            'motor_limit': {'CARTESIAN_MEASURED_LIMIT'},
            'common_mode': {'CARTESIAN_ACTUATION_OUTSIDE_CONTROL_SUBSPACE'},
            'motor_stall': {'CARTESIAN_SETTLE_TIMEOUT'},
            'fake_motor_success': {'CARTESIAN_NO_PROGRESS'},
            'command_rejected': {'CARTESIAN_COMMAND_REJECTED'},
            'rank_loss': {'CARTESIAN_NO_PROGRESS', 'CARTESIAN_NO_FEASIBLE_DESCENT_STEP', 'CARTESIAN_ITERATION_LIMIT'},
        }
        for fault, allowed in reasons.items():
            with self.subTest(fault=fault):
                run_case(fault, plant=SimulatedPlant(fault=fault), expect_success=False, expected_reasons=allowed)

    def test_cancel_and_fault_latch(self):
        def setup(c, p, g):
            p.on_command = lambda: c.cancel() if len(p.commands) == 2 else None
        run_case('cancel_in_motion', setup=setup, expect_success=False, expected_reasons={'CARTESIAN_CANCELLED'})

    def test_unconfirmed_stop_is_not_reported_as_acknowledged(self):
        def setup(c, p, g):
            p.on_command = c.cancel
        _, _, r = run_case('stop_not_acknowledged', plant=SimulatedPlant(fault='stop_unconfirmed'), setup=setup,
                          expect_success=False, expected_reasons={'CARTESIAN_CANCELLED'})
        self.assertTrue(all(x['reply_valid'] is False for x in r['stop_receipts']))

    def test_orientation_is_constraint_not_six_dof_goal(self):
        def setup(c, p, g):
            return replace(g, orientation_reference=(0., 0., 0., 1.), orientation_limit_rad=.001)
        run_case('orientation_constraint', setup=setup, expect_success=False, expected_reasons={'CARTESIAN_ORIENTATION_CONSTRAINT'})

    def test_overlapping_request(self):
        nested = []
        def setup(c, p, g):
            p.on_command = lambda: nested.append(c.move_position(g))
        run_case('overlap_rejected', setup=setup)
        self.assertTrue(nested and all(x['reason'] == 'CARTESIAN_BUSY' for x in nested))

    def test_time_limit(self):
        run_case('total_time_limit', config=simulation_config(timeout_s=.5), expect_success=False,
                 expected_reasons={'CARTESIAN_TIME_LIMIT'})

    def test_model_validity_region(self):
        run_case('model_validity_limit', config=simulation_config(max_model_radius_m=.003), expect_success=False,
                 expected_reasons={'CARTESIAN_NO_FEASIBLE_DESCENT_STEP'})

    def test_new_reference_session_required(self):
        p = SimulatedPlant(); c = CartesianController(simulation_config(), p)
        self.assertTrue(c.move_position(goal(p))['success'])
        count = len(p.commands)
        r = c.move_position(replace(goal(p), map_epoch='different_map'))
        self.assertEqual(r['reason'], 'CARTESIAN_NEW_REFERENCE_SESSION_REQUIRED')
        self.assertEqual(len(p.commands), count)

    def test_initial_goal_needs_fresh_verification(self):
        _, p, r = run_case('already_at_goal', u=(0., 0., 0.))
        self.assertFalse(p.commands)
        self.assertGreater(r['verification']['first_frame'], 333)

    def test_simulator_never_fabricates_future_capture(self):
        p = SimulatedPlant()
        for _ in range(1000):
            p.advance(.01)
            self.assertLessEqual(p.read().pose.capture_s, p.now())

    def test_solver_known_matrices(self):
        self.assertEqual(singular_values(((3., 0., 0.), (0., 2., 0.), (0., 0., 1.))), (1., 2., 3.))
        a = ((1., .2, -.1), (.1, 2., .3), (.2, -.1, 1.5)); x = (.1, -.3, .2)
        b = tuple(sum(v*t for v, t in zip(row, x)) for row in a)
        self.assertLess(norm(sub(least_squares(a, b), x)), 1e-12)

    def test_high_rate_camera_stability_window(self):
        run_case('camera_100hz', plant=SimulatedPlant(camera_period_s=.01))

    def test_cancel_accepted_before_completion_cannot_be_success(self):
        def setup(c, p, g):
            original = c.sample
            count = [0]
            def wrapped(*args, **kwargs):
                result = original(*args, **kwargs)
                count[0] += 1
                if count[0] == 2:
                    c.cancel()
                return result
            c.sample = wrapped
        run_case('cancel_before_success_commit', u=(0., 0., 0.), setup=setup, expect_success=False,
                 expected_reasons={'CARTESIAN_CANCELLED'})


if __name__ == '__main__':
    output = Path(sys.argv[1]) if len(sys.argv) > 1 else None
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(CartesianTests)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    if output:
        output.mkdir(parents=True, exist_ok=True)
        data = {'scope': 'Windows Python, deterministic simulated time, independent synthetic plant, no SDK/serial/ROS/device access',
                'unit_test_methods': result.testsRun, 'failure_count': len(result.failures), 'error_count': len(result.errors),
                'successful': result.wasSuccessful(), 'cases': RESULTS,
                'failures': [(str(test), error) for test, error in result.failures],
                'errors': [(str(test), error) for test, error in result.errors]}
        (output/'results.json').write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
    sys.exit(0 if result.wasSuccessful() else 1)
