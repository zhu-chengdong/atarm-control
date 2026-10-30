"""Fixed-goal progress windows; independent of learned Jacobians and retries."""
from dataclasses import dataclass,asdict,replace
from statistics import median
from copy import deepcopy
import math


@dataclass(frozen=True)
class WindowProgressConfig:
    window_s: float=0.12
    max_window_s: float=0.5
    grace_s: float=0.02
    allowed_pause_s: float=0.04
    minimum_fraction: float=0.2
    closing_fraction: float=0.25
    noise_bound_m: float=0.00004
    bad_windows: int=2
    min_frames: int=6
    expected_deploy_rate_m_s: float=0.2
    reference_deploy_speed_m_s: float=1.0
    expected_bend_rate_m_s: float=0.04
    reference_bend_speed_m_s: float=0.6
    reference_bend_step_m: float=0.0025
    severe_fraction: float=0.0
    severe_confirm_s: float=0.2
    severe_confirm_frames: int=3
    confirm_sampling_slack_s: float=0.0
    def __post_init__(self):
        for k,v in asdict(self).items():
            if k in ('severe_fraction','confirm_sampling_slack_s') and not isinstance(v,bool) and v==0: continue
            if isinstance(v,bool) or not math.isfinite(v) or v<=0: raise ValueError('Invalid progress-window '+k)
        if any(not isinstance(v,int) for v in (self.bad_windows,self.min_frames,self.severe_confirm_frames)) or self.min_frames<6 or self.severe_confirm_frames<3:
            raise ValueError('Integer window count and at least six source frames required')
        if self.max_window_s<self.window_s or self.allowed_pause_s>=self.window_s/2:
            raise ValueError('Invalid progress-window timing')
        if max(self.minimum_fraction,self.closing_fraction)>1: raise ValueError('Invalid progress fraction')
        if self.severe_fraction>=1 or self.severe_fraction and self.severe_confirm_s>=self.window_s:
            raise ValueError('Severe confirmation must be a fraction below one and shorter than its window')
        if self.severe_fraction and self.severe_confirm_s<=self.allowed_pause_s+self.confirm_sampling_slack_s:
            raise ValueError('Confirmation must leave positive motion time after its pause allowance')


@dataclass(frozen=True)
class PauseBudgetConfig:
    """One explicit pause allowance, with time to measure renewed progress."""
    pause_s: float=0.08
    sampling_slack_s: float=0.02
    def __post_init__(self):
        for name,value in asdict(self).items():
            if isinstance(value,bool) or not math.isfinite(value) or value<=0:
                raise ValueError('Invalid pause budget '+name)
    def window(self,cfg):
        if not isinstance(cfg,WindowProgressConfig): raise ValueError('Pause budget requires short-window protection')
        duration=max(cfg.window_s,2*(self.pause_s+self.sampling_slack_s))
        return replace(cfg,allowed_pause_s=self.pause_s,confirm_sampling_slack_s=self.sampling_slack_s,
            window_s=duration,max_window_s=max(duration,cfg.max_window_s))
    def timeout(self,original_s,progress_m,minimum_camera_rate_m_s,camera_per_motor=1.0):
        values=(original_s,progress_m,minimum_camera_rate_m_s,camera_per_motor)
        if not all(math.isfinite(v) and v>0 for v in values): raise ValueError('Invalid pause-budget rate or threshold')
        return max(original_s,self.pause_s+self.sampling_slack_s+progress_m*camera_per_motor/minimum_camera_rate_m_s)


class GoalProgressWindow:
    def __init__(self,cfg):
        if not isinstance(cfg,WindowProgressConfig): raise ValueError('Explicit WindowProgressConfig required')
        self.cfg=cfg;self.reset()
    def reset(self):
        self.active=False;self.state='IDLE';self.events=[];self.samples=[];self.fault=None
        self.bad_count=0;self.last_frame=None;self.last_capture=None;self.goal=None;self.rate=None
        self.severe_pending=None;self.severe_events=[];self.trigger_mode=None
    def begin(self,goal,tolerance_m,now,expected_rate_m_s):
        if not math.isfinite(expected_rate_m_s) or expected_rate_m_s<=0: raise ValueError('WINDOW_RATE_INVALID')
        self.reset();self.goal=goal;self.tolerance=tolerance_m;self.started=now
        self.rate=expected_rate_m_s*self.cfg.minimum_fraction;self.active=True;self.state='WARMUP'
    def finish(self,success,message=''):
        if self.active:
            self.active=False;self.state='COMPLETED' if success else 'ABORTED'
            self.severe_pending=None
            if not success: self.fault=message
    def status(self):
        return {'enabled':True,'active':self.active,'state':self.state,'fault':self.fault,
            'bad_count':self.bad_count,'required_rate_m_s':self.rate,
            'goal':None if self.goal is None else asdict(self.goal),'events':deepcopy(self.events),'limits':asdict(self.cfg),
            'trigger_mode':self.trigger_mode,'severe_events':deepcopy(self.severe_events),
            'severe_pending':None if self.severe_pending is None else {
                k:v for k,v in self.severe_pending.items() if k!='samples'}}
    def _confirm_severe(self,entry):
        pending=self.severe_pending
        if pending is None: return
        pending['samples'].append(entry)
        if entry['s']-pending['armed_capture_s']<self.cfg.severe_confirm_s or len(pending['samples'])<self.cfg.severe_confirm_frames:
            return
        tail=pending['samples'][-3:]
        last=median(e['error_m'] for e in tail);dt=median(e['s'] for e in tail)-pending['anchor_capture_s']
        # One fixed pause credit in this independent confirmation period.
        # Neither a pause nor small progress changes its armed deadline.
        motion_dt=max(0,dt-self.cfg.allowed_pause_s-self.cfg.confirm_sampling_slack_s)
        required=min(self.rate*motion_dt,self.cfg.closing_fraction*max(0,pending['anchor_error_m']-self.tolerance))
        upper=pending['anchor_error_m']-last+2*self.cfg.noise_bound_m
        resolved=required>2*self.cfg.noise_bound_m;bad=resolved and upper<required
        proof={k:v for k,v in pending.items() if k!='samples'}
        proof.update({'last_frame':entry['frame'],'last_capture_s':entry['s'],
            'new_frames':len(pending['samples']),'effective_dt_s':dt,'last_error_m':last,
            'pause_allowance_s':self.cfg.allowed_pause_s,'effective_motion_s':motion_dt,
            'sampling_slack_s':self.cfg.confirm_sampling_slack_s,
            'required_reduction_m':required,'progress_upper_bound_m':upper,'resolved':resolved,
            'decision':'STOP' if bad else 'RECOVERED' if resolved else 'UNRESOLVED'})
        self.severe_events.append(proof);self.severe_pending=None
        if bad:
            self.trigger_mode='SEVERE_THEN_LOW_RECOVERY';self.fault='WINDOW_PROGRESS_TOO_SMALL'
            self.state='FAULT';self.active=False;raise ValueError(self.fault)
        if resolved:
            self.bad_count=0;self.samples=self.samples[-3:]
    def observe(self,pose):
        if not self.active: return
        if self.last_frame is not None and pose.frame_id<=self.last_frame: return
        if self.last_capture is not None and pose.capture_s<=self.last_capture:
            raise ValueError('WINDOW_CAPTURE_TIME_NOT_ADVANCING')
        self.last_frame=pose.frame_id;self.last_capture=pose.capture_s
        if pose.capture_s<self.started+self.cfg.grace_s: return
        error=math.sqrt(sum((a-b)**2 for a,b in zip(pose.position_m,self.goal.position_m)))
        if error<=self.tolerance+self.cfg.noise_bound_m:
            self.state='NEAR_GOAL';self.bad_count=0;self.samples=[];self.severe_pending=None;return
        self.samples.append({'frame':pose.frame_id,'s':pose.capture_s,'error_m':error})
        self._confirm_severe(self.samples[-1])
        if len(self.samples)<self.cfg.min_frames: return
        span=self.samples[-1]['s']-self.samples[0]['s']
        if span<self.cfg.window_s: return
        a=self.samples[:3];b=self.samples[-3:]
        dt=median(x['s'] for x in b)-median(x['s'] for x in a)
        if dt<self.cfg.window_s/2: return
        first=median(x['error_m'] for x in a);last=median(x['error_m'] for x in b)
        required=min(self.rate*max(0,dt-self.cfg.allowed_pause_s),
            self.cfg.closing_fraction*max(0,first-self.tolerance))
        upper=first-last+2*self.cfg.noise_bound_m
        lower=first-last-2*self.cfg.noise_bound_m
        resolved=required>2*self.cfg.noise_bound_m
        if not resolved and span<self.cfg.max_window_s:
            self.state='RESOLUTION_WAIT';return
        bad=resolved and upper<required
        self.bad_count=self.bad_count+1 if bad else 0
        event={'first_frame':a[0]['frame'],'last_frame':b[-1]['frame'],'first_capture_s':a[0]['s'],
            'last_capture_s':b[-1]['s'],'effective_dt_s':dt,'first_error_m':first,'last_error_m':last,
            'observed_reduction_m':first-last,'progress_upper_bound_m':upper,'required_reduction_m':required,
            'progress_lower_bound_m':lower,
            'resolved':resolved,'bad':bad,'bad_count':self.bad_count}
        self.events.append(event);self.state='CHECKING' if resolved else 'UNRESOLVED'
        self.samples=self.samples[-3:]
        if self.bad_count>=self.cfg.bad_windows:
            self.trigger_mode='CONSECUTIVE_WINDOWS'
            self.fault='WINDOW_PROGRESS_TOO_SMALL';self.state='FAULT';self.active=False
            raise ValueError(self.fault)
        # Suspicion may be armed when the uncertainty interval includes severe
        # insufficiency. Stopping still needs independent fresh confirmation
        # and its optimistic progress bound, so uncertainty alone cannot stop.
        if bad and self.cfg.severe_fraction and lower<=self.cfg.severe_fraction*required:
            self.severe_pending={'armed_frame':pose.frame_id,'armed_capture_s':pose.capture_s,
                'anchor_error_m':last,'anchor_capture_s':median(x['s'] for x in b),
                'primary_ratio':upper/required,'primary_progress_lower_m':lower,
                'primary_severe_threshold_m':self.cfg.severe_fraction*required,'samples':[]}
