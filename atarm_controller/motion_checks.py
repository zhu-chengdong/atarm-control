"""Command-relative plausibility checks; not independent position sensing."""
from __future__ import annotations
from dataclasses import dataclass, asdict
import math


class MotionGuardError(RuntimeError):
    pass


@dataclass(frozen=True)
class MotionGuardConfig:
    max_speed_m_s: float = 1.0
    step_margin_m: float = 0.0005
    uncommanded_tolerance_m: float = 0.0015

    def __post_init__(self):
        for name, value in asdict(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name+' must be finite and positive.')


class MotionEvidenceGuard:
    def __init__(self, config):
        self.config=config
        self.last_position=None
        self.last_rx_ns=None
        self.last_target=None
        self.origin=None
        self.anchor=None
        self.holding=True
        self.fault=None
        self.last_diagnostics={}

    def _reject(self,code,reason,details):
        self.fault={'code':code,'reason':reason,**details}
        raise MotionGuardError(code+': '+reason)

    def observe(self,position,rx_ns,target=None,*,rebaseline=False):
        if not math.isfinite(position) or (target is not None and not math.isfinite(target)):
            raise MotionGuardError('Nonfinite motion evidence.')
        if rebaseline or self.last_position is None:
            self.last_position=position; self.last_rx_ns=rx_ns
            self.last_target=None
            self.origin=self.anchor=position; self.holding=True
            self.fault=None
            self.last_diagnostics={'position_m':position,'reference_acquisition':rebaseline}
            return
        if self.fault is not None:
            raise MotionGuardError(self.fault['code']+': latched; explicit reference recovery required.')
        dt=(rx_ns-self.last_rx_ns)/1e9
        if dt<0:
            self._reject('NON_MONOTONIC_FEEDBACK','receive time moved backwards',{'dt_s':dt})
        active=target is not None
        new_target=active and (self.last_target is None or abs(target-self.last_target)>1e-7)
        origin,anchor,holding=self.origin,self.anchor,self.holding
        if active:
            if new_target:
                if holding or self.last_target is None:
                    origin=anchor
                elif (target-self.last_position)*(self.last_target-self.last_position)<0:
                    # A commanded reversal starts from the last observed position.
                    origin=self.last_position
                holding=False
            if holding:
                lo=hi=anchor
            else:
                lo,hi=min(origin,target),max(origin,target)
        else:
            if self.last_target is not None:
                anchor=self.last_position
            holding=True; origin=anchor; lo=hi=anchor
        margin=self.config.uncommanded_tolerance_m
        details={'previous_m':self.last_position,'reported_m':position,'dt_s':dt,
                 'target_m':target,'allowed_min_m':lo-margin,'allowed_max_m':hi+margin,
                 'position_command_active':active,'new_target':new_target}
        if not lo-margin<=position<=hi+margin:
            self._reject('UNCOMMANDED_DISPLACEMENT',
                'measurement is outside motion permitted by issued commands',details)
        allowed=self.config.max_speed_m_s*dt+self.config.step_margin_m
        if abs(position-self.last_position)>allowed:
            self._reject('POSITION_DISCONTINUITY','position change exceeds configured speed and allowance',
                         {**details,'allowed_step_m':allowed})
        if active and not holding and abs(position-target)<=margin:
            holding=True; anchor=target; origin=target
        self.last_position=position; self.last_rx_ns=rx_ns
        self.last_target=target
        self.origin,self.anchor,self.holding=origin,anchor,holding
        self.last_diagnostics={**details,'allowed_step_m':allowed,'holding':holding}

    def status(self):
        return {'limits':asdict(self.config),'fault':self.fault,
                'holding':self.holding,'last_check':self.last_diagnostics}
