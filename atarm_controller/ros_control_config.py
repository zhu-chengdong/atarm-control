"""Explicit JSON calibration/limits for the ROS entry point; no guessed arm model."""
import json
from pathlib import Path
from .camera_feedback import CameraGoal,CameraGateConfig
from .camera_cycle import CyclePlan,CycleConfig
from .camera_recovery import RecoveryConfig
from .recovery_probe import ProbeConfig,ProbeWaypoint
from .visual_servo import VisualServoConfig
from .coupled_bend import BendServoConfig
from .online_jacobian import JacobianUpdateConfig
from .progress_window import WindowProgressConfig,PauseBudgetConfig

def load_control_config(path):
    data=json.loads(Path(path).read_text(encoding='utf-8'))
    if data.get('schema_version')!=1:raise ValueError('Unsupported ROS control configuration schema')
    ref=data['reference'];keys={k:ref[k] for k in ('parent_frame','child_frame','map_epoch')}
    def goal(d):return CameraGoal(**keys,**d)
    goals={False:goal(data['goals']['retracted']),True:goal(data['goals']['deployed'])}
    plan=CyclePlan(goals[True],tuple(goal(x) for x in data['goals']['bend']))
    def servo(cls,d):
        d=dict(d)
        if isinstance(d.get('online_update'),dict):d['online_update']=JacobianUpdateConfig(**d['online_update'])
        return cls(**d)
    options={'camera_goals':goals,'cycle_plan':plan,'bend_goal_template':plan.neutral_goal,
        'camera_config':CameraGateConfig(**data['camera_config']),
        'bend_camera_config':CameraGateConfig(**data['bend_camera_config']),
        'visual_servo_config':servo(VisualServoConfig,data['visual_servo_config']),
        'bend_servo_config':servo(BendServoConfig,data['bend_servo_config']),
        'cycle_config':CycleConfig(**data['cycle_config']),
        'recovery_config':RecoveryConfig(**data['recovery_config']),
        'window_progress_config':WindowProgressConfig(**data['window_progress_config']),
        'pause_budget_config':PauseBudgetConfig(**data['pause_budget_config'])}
    if data.get('probe_config'):
        p=dict(data['probe_config']);p['waypoints']=tuple(ProbeWaypoint(**x) for x in p['waypoints']);options['probe_config']=ProbeConfig(**p)
    return options,keys
