"""Run a 3D position goal against the synthetic plant, without installation."""
import argparse
import csv
import json
from pathlib import Path
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from atarm_controller.cartesian_control import CartesianController, PositionGoal
from atarm_controller.cartesian_simulation import SimulatedPlant, simulation_config
from atarm_controller.cartesian_math import norm, sub


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--target', nargs=3, type=float, default=(.3124208, .03748, .1949592), metavar=('X', 'Y', 'Z'), help='metric position in sim_base')
    p.add_argument('--strategy', choices=('alternating', 'coordinated'), default='alternating')
    p.add_argument('--fixed-jacobian', action='store_true')
    p.add_argument('--seed', type=int, default=7)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    # The only provider available in this entry point is the synthetic plant.
    plant = SimulatedPlant(seed=args.seed)
    controller = CartesianController(simulation_config(strategy=args.strategy, online_update=not args.fixed_jacobian), plant)
    goal = PositionGoal(tuple(args.target), 'sim_base', 'camera_optical', 'sim_map_0')
    result = controller.move_position(goal)
    data = {'scope': 'simulation only; not real arm accuracy', 'goal': list(args.target), 'result': result,
            'independent_truth_error_m': norm(sub(goal.position_m, plant.true_position())),
            'true_final_position_m': plant.true_position(), 'controller': controller.status()}
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'result.json').write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
    with (args.output/'steps.csv').open('w', newline='', encoding='utf-8-sig') as f:
        w = csv.writer(f); w.writerow(['step', 'block', 'time_sim_s', 'position_error_before_m', 'position_error_after_m', 'q0_m', 'q1_m', 'q2_m', 'q3_m'])
        for i, h in enumerate(controller.history, 1):
            w.writerow([i, h['block'], h['at_s'], h['error_before_m'], h.get('error_after_m'), *h['target_q']])
    print(json.dumps({'state': result['state'], 'reason': result['reason'], 'steps': len(controller.history),
                      'independent_truth_error_mm': data['independent_truth_error_m']*1000, 'output': str(args.output)}))
    return 0 if result['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
