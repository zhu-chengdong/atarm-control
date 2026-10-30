"""Real rclpy/DDS clients and separate simulation-node processes, no hardware."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
TARGET = [.3124208, .03748, .1949592]


def spin_until(node, predicate, seconds=12.):
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.02)
        if predicate():
            return
    raise AssertionError('DDS wait timed out')


def invoke(node, client):
    sent = time.perf_counter_ns(); future = client.call_async(Trigger.Request())
    spin_until(node, future.done)
    response = future.result()
    return {'success': response.success, 'body': json.loads(response.message), 'sent_ns': sent, 'received_ns': time.perf_counter_ns()}


def case(name, out, *, fault='', strategy='alternating', cancel=False, unreachable=False, shutdown=False):
    directory = out/name; directory.mkdir()
    audit = directory/'node_exit_audit.json'; log = (directory/'node.log').open('wb')
    namespace = '/cart_'+name
    env = dict(os.environ, PYTHONPATH=str(PROJECT)+os.pathsep+os.environ.get('PYTHONPATH', ''), ATARM_SIM_AUDIT_PATH=str(audit))
    argv = [sys.executable, '-B', '-m', 'atarm_controller.cartesian_sim_node', '--ros-args', '-r', '__ns:='+namespace,
            '-p', 'simulation_speed:=4.0', '-p', 'strategy:='+strategy, '-p', 'test_fault:='+('""' if not fault else fault)]
    server = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT, env=env)
    node = Node('client_'+name); states = []; acks = []
    prefix = namespace+'/atarm/cartesian_sim'
    pub = node.create_publisher(String, prefix+'/target', 10)
    node.create_subscription(String, prefix+'/ack', lambda m: acks.append(json.loads(m.data)), 10)
    node.create_subscription(String, prefix+'/state', lambda m: states.append(json.loads(m.data)), 10)
    cancel_client = node.create_client(Trigger, prefix+'/cancel'); reset_client = node.create_client(Trigger, prefix+'/reset')
    result = {'case': name, 'controller_pid': server.pid, 'client_pid': os.getpid()}
    def send(request):
        count = len(acks); m = String(); m.data = json.dumps(request, allow_nan=True); pub.publish(m)
        spin_until(node, lambda: len(acks) > count and any(a.get('request_id') == request.get('request_id') for a in acks[count:]))
        return next(a for a in acks[count:] if a.get('request_id') == request.get('request_id'))
    try:
        spin_until(node, lambda: server.poll() is not None or (states and pub.get_subscription_count() > 0 and cancel_client.service_is_ready() and reset_client.service_is_ready()))
        assert server.poll() is None, 'ROS node exited before discovery'
        g = dict(states[-1]['reference'], request_id=name+'_goal', position_m=[.8, .8, .8] if unreachable else TARGET)
        result['acceptance'] = send(g)
        assert result['acceptance']['accepted'] and not result['acceptance']['complete']
        if name == 'complete':
            result['duplicate'] = send(g); assert result['duplicate']['reason'] == 'ROS_CARTESIAN_DUPLICATE_ID'
            result['busy'] = send(dict(g, request_id='overlap')); assert result['busy']['reason'] == 'ROS_CARTESIAN_BUSY'
            result['reset_while_busy'] = invoke(node, reset_client); assert not result['reset_while_busy']['success']
        if cancel or shutdown:
            spin_until(node, lambda: states[-1]['job']['state'] == 'RUNNING' and states[-1].get('observation') and
                       math.dist(states[-1]['observation']['position_m'], [.3, .05, .18]) > .001)
            if cancel:
                result['cancel'] = invoke(node, cancel_client); assert result['cancel']['success']
            else:
                result['shutdown_sent_ns'] = time.perf_counter_ns(); server.send_signal(signal.SIGINT)
        if not shutdown:
            spin_until(node, lambda: states[-1]['job'].get('request_id') == g['request_id'] and states[-1]['job']['state'] in ('SUCCEEDED', 'FAULT', 'CANCELLED'))
            result['final'] = states[-1]
            job = result['final']['job']; expected = 'CANCELLED' if cancel else 'FAULT' if fault or unreachable else 'SUCCEEDED'
            assert job['state'] == expected, job
            assert job['complete'] is (expected == 'SUCCEEDED')
            assert job['io_threads'] == ['cartesian-sim-owner']
            if expected == 'SUCCEEDED':
                # Independent oracle recomputed from the four motor positions.
                q0, q1, q2, q3 = job['final_motor_positions_m']
                a, b, s = (2*q0-q1-q2)/3, (q1-q2)/math.sqrt(3), q3-.04
                truth = [.3+.85*a+.18*b+.22*s+3*a*s+1.2*b*b,
                         .05-.12*a+1.05*b-.16*s+2*b*s-1.5*a*a,
                         .18+.12*a-.08*b+.9*s+2.8*a*a+2*b*b-1.5*s*a]
                result['independent_truth_error_m'] = math.dist(truth, g['position_m'])
                assert result['independent_truth_error_m'] <= .0005
            else:
                receipts = job['result']['stop_receipts']; assert len(receipts) == 4 and all(x['reply_valid'] for x in receipts)
                result['retry_without_reset'] = send(dict(g, request_id='retry'))
                assert result['retry_without_reset']['reason'] == 'ROS_CARTESIAN_RESET_REQUIRED'
            if cancel:
                result['cancel_request_to_sim_stop_ms'] = (job['stop_wall_ns']-result['cancel']['sent_ns'])/1e6
                assert 0 <= result['cancel_request_to_sim_stop_ms'] < 500
            if name == 'complete':
                result['invalid'] = send(dict(g, request_id='bad_nan', position_m=[float('nan'), 0, 0]))
                assert not result['invalid']['accepted']
                result['reset'] = invoke(node, reset_client); assert result['reset']['success']
                new_epoch = result['reset']['body']['map_epoch']
                result['old_map'] = send(dict(g, request_id='old_map'))
                assert result['old_map']['reason'] == 'ROS_CARTESIAN_REFERENCE_MISMATCH'
                again = dict(g, request_id='after_reset', map_epoch=new_epoch)
                assert send(again)['accepted']
                spin_until(node, lambda: states[-1]['job'].get('request_id') == 'after_reset' and states[-1]['job']['state'] == 'SUCCEEDED')
                result['after_reset'] = states[-1]
        result['running_status_count'] = sum(x['job']['state'] == 'RUNNING' for x in states)
        # Full movements must publish progress repeatedly. Promptly stopping
        # faults can legitimately finish before three 20-Hz status periods.
        if not (fault or cancel or shutdown):
            assert result['running_status_count'] >= 3
        elif cancel or shutdown:
            assert result['running_status_count'] >= 1
        assert states[-1]['state_sequence'] > states[0]['state_sequence']
        assert all(b['state_sequence'] > a['state_sequence'] for a, b in zip(states, states[1:]))
        assert server.pid != os.getpid()
        result['passed'] = True
    except Exception:
        result.update(passed=False, error=traceback.format_exc())
    finally:
        if server.poll() is None and not result.get('shutdown_sent_ns'):
            server.send_signal(signal.SIGINT)
        try:
            server.wait(timeout=8.)
        except subprocess.TimeoutExpired:
            server.kill(); server.wait(); result.update(passed=False, shutdown_error='Node did not exit')
        log.close(); node.destroy_node()
        result['server_returncode'] = server.returncode
        if server.returncode != 0:
            result.update(passed=False, abnormal_server_exit=True)
        if audit.exists():
            result['exit_audit'] = json.loads(audit.read_text())
            if result['exit_audit']['worker_alive'] or not result['exit_audit']['simulation_stopped'] or result['exit_audit']['hardware_modules_loaded']:
                result.update(passed=False, exit_audit_failed=True)
        else:
            result.update(passed=False, exit_audit_missing=True)
        (directory/'case.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
        (directory/'dds_states.json').write_text(json.dumps(states, indent=2), encoding='utf-8')
        (directory/'dds_acknowledgments.json').write_text(json.dumps(acks, indent=2), encoding='utf-8')
    return result


def main():
    p = argparse.ArgumentParser(); p.add_argument('--out', type=Path, required=True); p.add_argument('--case', default='all')
    a = p.parse_args(); a.out.mkdir()
    rclpy.init()
    try:
        plans = [('complete', {}), ('coordinated', {'strategy':'coordinated'}), ('cancel', {'cancel':True}),
                 ('tracking_lost', {'fault':'tracking_lost'}), ('frozen_pose', {'fault':'frozen_pose'}),
                 ('map_reset', {'fault':'map_reset'}), ('unreachable', {'unreachable':True}), ('shutdown', {'shutdown':True})]
        if a.case != 'all' and a.case not in dict(plans):
            raise ValueError('Unknown case')
        results = [case(name, a.out, **options) for name, options in plans if a.case in ('all', name)]
    finally:
        rclpy.shutdown()
    summary = {'scope': 'actual ROS2 Jazzy/rclpy/DDS, separate node/client processes, synthetic camera and motors',
               'ros_distro': os.environ.get('ROS_DISTRO'), 'rclpy_file': rclpy.__file__, 'cases': results,
               'passed': all(x['passed'] for x in results), 'count': len(results),
               'source_sha256': {str(x.relative_to(PROJECT)): hashlib.sha256(x.read_bytes()).hexdigest()
                                  for x in (PROJECT/'atarm_controller').glob('*.py')}}
    summary['source_sha256']['tests/ros_cartesian_matrix.py'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    (a.out/'results.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    print(json.dumps({'passed': summary['passed'], 'cases': [{'case': x['case'], 'passed': x['passed'], 'error': x.get('error')} for x in results]}))
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
