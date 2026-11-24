"""Piecewise-linear ideal synchronous buck, switched twice per 20 us PWM period.

States are iL in A and ideal capacitor voltage vc in V.  Positive iL flows
from the switch node to the output.  Source and FETs are ideal, and the
high-side FET conducts first in each active PWM period.  Rload can be changed
at period boundaries.  DCR and capacitor ESR are the only dissipative
parasitics.  During shutdown ideal switch body diodes carry residual current.
"""
from functools import lru_cache
from math import cos, exp, expm1, isfinite, sin, sqrt


class Plant:
    PERIOD = 20e-6
    SAMPLE_AT = 5e-6

    def __init__(self, L=470e-6, C=220e-6, dcr=0.10, esr=0.04, load=18.75):
        if not all(isfinite(x) for x in (L, C, dcr, esr)):
            raise ValueError("Model parameters must be finite")
        if min(L, C) <= 0 or min(dcr, esr) < 0:
            raise ValueError("L and C must be positive; dcr and esr nonnegative")
        self.L, self.C, self.dcr, self.esr = L, C, dcr, esr
        self.il = self.vc = 0.0
        self.load = load

    @property
    def load(self):
        return self._load

    @load.setter
    def load(self, value):
        if not isfinite(value) or value <= 0:
            raise ValueError("A finite positive resistive load is required")
        self._load = value
        self._k = value / (value + self.esr)
        a = -(self.dcr + self._k * self.esr) / self.L
        b = -self._k / self.L
        c = self._k / self.C
        d = -1 / ((value + self.esr) * self.C)
        h = (a + d) / 2
        delta2 = ((a - d) / 2)**2 + b*c
        @lru_cache(maxsize=16384)
        def transition(dt):
            if delta2 == 0:
                w0, w1 = exp(h*dt), dt*exp(h*dt)
            elif delta2 < 0:
                w = sqrt(-delta2)
                e = exp(h*dt)
                w0, w1 = e*cos(w*dt), e*sin(w*dt)/w
            else:
                w = sqrt(delta2)
                hi = exp((h+w)*dt)
                lo = exp((h-w)*dt)
                w0, w1 = (hi+lo)/2, -hi*expm1(-2*w*dt)/(2*w)
            return w0+w1*(a-h), w1*b, w1*c, w0+w1*(d-h)
        self._transition = transition

    @property
    def vout(self):
        return self._k * (self.vc + self.esr*self.il)

    def _next(self, switch_v, dt):
        e00, e01, e10, e11 = self._transition(dt)
        i0 = switch_v / (self.load+self.dcr)
        v0 = self.load*i0
        di, dv = self.il-i0, self.vc-v0
        return i0+e00*di+e01*dv, v0+e10*di+e11*dv

    def _advance(self, switch_v, dt):
        if dt > 0:
            self.il, self.vc = self._next(switch_v, dt)

    def _gates_off(self, vin, dt):
        # At zero current, the output capacitor drains through ESR + load
        # unless a body diode must conduct because the output is outside the rails.
        if self.il == 0 and 0 <= self.vout <= vin:
            self.vc *= exp(-dt / ((self.load+self.esr)*self.C))
            return
        forward = self.il > 0 or (self.il == 0 and self.vout < 0)
        switch_v = 0.0 if forward else vin
        il, vc = self._next(switch_v, dt)
        if (forward and il >= 0) or (not forward and il <= 0):
            self.il, self.vc = il, vc
            return
        # For the modeled hardware the natural LC period greatly exceeds dt.
        # Locate the first zero crossing and release the conducting body diode.
        low, high = 0.0, dt
        for _ in range(50):
            middle = (low+high)/2
            i, _ = self._next(switch_v, middle)
            if (i > 0) == forward:
                low = middle
            else:
                high = middle
        self.il, self.vc = self._next(switch_v, high)
        self.il = 0.0
        if dt-high > 1e-15:
            self._gates_off(vin, dt-high)

    def cycle(self, vin, duty, enabled=True):
        """Advance a full period; return simultaneous (vout, il) at 5 us."""
        if not isfinite(vin) or vin < 0 or not isfinite(duty) or not 0 <= duty <= 1:
            raise ValueError("Require finite Vin >= 0 and a duty between 0 and 1")
        sample, period = self.SAMPLE_AT, self.PERIOD
        if not enabled:
            self._gates_off(vin, sample)
            values = self.vout, self.il
            self._gates_off(vin, period-sample)
            return values
        on = period*duty
        self._advance(vin, min(on, sample))
        self._advance(0.0, max(0.0, sample-on))
        values = self.vout, self.il
        self._advance(vin, max(0.0, on-sample))
        self._advance(0.0, period-max(sample, on))
        return values
