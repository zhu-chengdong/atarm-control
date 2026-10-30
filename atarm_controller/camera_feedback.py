"""Independent pose completion evidence, without ROS or camera device access.

Input poses must be metric and transformed into the goal frame, with capture
timestamps mapped to the controller's monotonic clock. Receipt time is not a
substitute for capture time. This module does not estimate poses or control a
visual servo. Source independence is a deployment requirement, not something a
dataclass can prove.
"""
from dataclasses import dataclass, asdict
import math


@dataclass(frozen=True)
class CameraPose:
    frame_id: int
    capture_s: float
    parent_frame: str
    child_frame: str
    map_epoch: str
    tracking_ok: bool
    position_m: tuple
    quaternion_xyzw: tuple = (0.0, 0.0, 0.0, 1.0)

    def __post_init__(self):
        object.__setattr__(self,'position_m',tuple(self.position_m))
        object.__setattr__(self,'quaternion_xyzw',tuple(self.quaternion_xyzw))


@dataclass(frozen=True)
class CameraGoal:
    parent_frame: str
    child_frame: str
    map_epoch: str
    position_m: tuple
    quaternion_xyzw: tuple = (0.0, 0.0, 0.0, 1.0)

    def __post_init__(self):
        object.__setattr__(self,'position_m',tuple(self.position_m))
        object.__setattr__(self,'quaternion_xyzw',tuple(self.quaternion_xyzw))


@dataclass(frozen=True)
class CameraGateConfig:
    max_age_s: float = 0.1
    position_tolerance_m: float = 0.002
    orientation_tolerance_rad: float = 0.05
    settle_s: float = 0.1
    progress_min_m: float = 0.001
    progress_timeout_s: float = 1.0
    max_speed_m_s: float = 0.5
    step_margin_m: float = 0.002

    def __post_init__(self):
        for name, value in asdict(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name+' must be finite and positive')


def _valid_geometry(pose):
    if len(pose.position_m) != 3 or len(pose.quaternion_xyzw) != 4:
        return False
    if not all(math.isfinite(v) for v in (*pose.position_m, *pose.quaternion_xyzw)):
        return False
    norm=sum(v*v for v in pose.quaternion_xyzw)
    return abs(norm-1.0) <= 0.01


def _distance(a, b):
    return math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))


class CameraCompletionGate:
    def __init__(self, config=None):
        self.config=config or CameraGateConfig()
        self.reset()

    def reset(self):
        """Only explicit reference recovery may discard previous source history."""
        self.last_sample=None
        self.goal=None
        self.fault=None
        self.ready=False
        self.evidence={'enabled':True,'ready':False,'fault':None}

    def _fail(self, code):
        self.fault=code
        self.ready=False
        self.evidence.update({'ready':False,'fault':code})
        raise ValueError(code)

    def begin(self, goal, sample, now):
        if self.fault:
            self._fail(self.fault)
        if not isinstance(goal,CameraGoal) or not _valid_geometry(goal):
            self._fail('CAMERA_GOAL_INVALID')
        if not all((goal.parent_frame,goal.child_frame,goal.map_epoch)):
            self._fail('CAMERA_GOAL_FRAME_INVALID')
        self.goal=goal
        self.started=now
        self.last_progress=now
        self.initial_error=None
        self.best_reduction=0.0
        self.advance_mark=0.0
        self.stable_start=None
        self.stable_frames=0
        self.ready=False
        self.evidence={'enabled':True,'ready':False,'fault':None,'goal':asdict(goal)}
        return self.observe(sample,now)

    def observe(self, sample, now):
        if self.fault:
            self._fail(self.fault)
        if not isinstance(sample,CameraPose):
            self._fail('CAMERA_FEEDBACK_MISSING')
        if not sample.tracking_ok:
            self._fail('CAMERA_TRACKING_LOST')
        if not _valid_geometry(sample):
            self._fail('CAMERA_POSE_INVALID')
        if (sample.parent_frame,sample.child_frame,sample.map_epoch) != (
                self.goal.parent_frame,self.goal.child_frame,self.goal.map_epoch):
            self._fail('CAMERA_REFERENCE_MISMATCH')
        if not isinstance(sample.frame_id,int) or sample.frame_id < 0:
            self._fail('CAMERA_FRAME_ID_INVALID')
        if not math.isfinite(sample.capture_s) or not math.isfinite(now):
            self._fail('CAMERA_TIMESTAMP_INVALID')
        age=now-sample.capture_s
        if age < 0:
            self._fail('CAMERA_TIMESTAMP_IN_FUTURE')
        if age > self.config.max_age_s:
            self._fail('CAMERA_FEEDBACK_STALE')
        previous=self.last_sample
        new=previous is None or sample.frame_id>previous.frame_id
        if previous is not None:
            if sample.frame_id<previous.frame_id or (not new and sample!=previous):
                self._fail('CAMERA_FRAME_REPLAY_OR_MUTATION')
            if new:
                dt=sample.capture_s-previous.capture_s
                if dt <= 0:
                    self._fail('CAMERA_CAPTURE_TIME_NOT_ADVANCING')
                allowed=self.config.max_speed_m_s*dt+self.config.step_margin_m
                if _distance(sample.position_m,previous.position_m)>allowed:
                    self._fail('CAMERA_POSE_DISCONTINUITY')
        error=_distance(sample.position_m,self.goal.position_m)
        dot=abs(sum(a*b for a,b in zip(sample.quaternion_xyzw,self.goal.quaternion_xyzw)))
        # Normalize tolerated small quaternion norm errors; q and -q are equal.
        norm=math.sqrt(sum(v*v for v in sample.quaternion_xyzw)*sum(v*v for v in self.goal.quaternion_xyzw))
        angle=2*math.acos(min(1.0,dot/norm))
        if self.initial_error is None:
            self.initial_error=error
        self.best_reduction=max(self.best_reduction,self.initial_error-error)
        if new and self.best_reduction-self.advance_mark>=self.config.progress_min_m:
            self.advance_mark=self.best_reduction
            self.last_progress=now
        inside=error<=self.config.position_tolerance_m and angle<=self.config.orientation_tolerance_rad
        if not inside:
            self.stable_start=None
            self.stable_frames=0
        elif new:
            if self.stable_start is None:
                self.stable_start=sample.capture_s
            self.stable_frames+=1
        stable_span=0.0 if self.stable_start is None else sample.capture_s-self.stable_start
        # Polling one cached good pose never manufactures multiple stable frames.
        self.ready=bool(new and inside and self.stable_frames>=2 and stable_span>=self.config.settle_s)
        self.evidence.update({'ready':self.ready,'frame_id':sample.frame_id,'capture_s':sample.capture_s,
            'age_s':age,'new_frame':new,'position_error_m':error,'orientation_error_rad':angle,
            'best_progress_m':self.best_reduction,'seconds_without_progress':now-self.last_progress,
            'stable_frames':self.stable_frames,'stable_span_s':stable_span})
        self.last_sample=sample
        if (self.initial_error>self.config.position_tolerance_m and error>self.config.position_tolerance_m
                and now-self.last_progress>=self.config.progress_timeout_s):
            self._fail('CAMERA_PROGRESS_STALLED')
        return self.ready

    def status(self):
        return dict(self.evidence)
