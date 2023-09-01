from collections import deque, namedtuple
from datetime import datetime, timedelta
from typing import Optional


Gains = namedtuple("Gains", list("pid"))


class PIDController(object):
    
    def __init__(self, p_gain=0, i_gain=0, d_gain=0, setpoint=None):
        self.gains = Gains(p=p_gain, i=i_gain, d=d_gain)
        self.setpoint = setpoint
        self._integrator = Integrator(0.0, "center-approx")
        self._differentiator = Differentiator(0.0)
        
    @property
    def gains(self):
        return self._gains
    
    @gains.setter
    def gains(self, _gains):
        if all(0 <= getattr(_gains, x) <= 100 for x in "pid"):
            self._gains = _gains
        else:
            raise ValueError(f"Invalid gains: {_gains}")
            
    def get_output(self, input_value):
        self._ensure_setpoint_set()
        current_time: datetime = datetime.now()
        current_error: float = self.setpoint - input_value
        p = self._gains.p * current_error
        i = self._gains.i * self._integrator.compute(current_error, current_time)
        d = self._gains.d * self._differentiator.compute(current_error, current_time)
        return p + i + d, current_error

    def _ensure_setpoint_set(self):
        if self.setpoint is None:
            raise Exception("Setpoint not set")


class Integrator(object):
    """
    Abstracts numerical integration methods.
    """

    def __init__(self, initial_value: float=0.0, approximation_scheme: str="center-approx") -> None:
        """
        Initialize an integrator object with an initial value.
        """
        self._approximation_scheme: str = approximation_scheme
        self._last_integrand: Optional[float] = None
        self._value: float = initial_value
        self._last_compute_time: Optional[datetime] = None

    def compute(self, integrand: float, current_time: datetime) -> float:
        """
        Will be a dispatch method that computes according to the desired approximation scheme.
        """
        return {
            "center-approx": self._compute_next_center_approx_riemann_sum_term
        }[self._approximation_scheme](integrand, current_time)

    def _compute_next_center_approx_riemann_sum_term(self, integrand: float, current_time: datetime) -> float:
        """
        Compute the next term in the Riemann sum using the midpoint approximation.
        """
        if self._last_compute_time:
            average_integrand: float = 0.5 * (integrand + self._last_integrand)
            time_delta_s: float = (current_time - self._last_compute_time).total_seconds()
            self._value += average_integrand * time_delta_s
        self._last_integrand = integrand
        self._last_compute_time = current_time
        return self._value


class Differentiator(object):
    """
    Abstracts numerical differentiation methods.
    """

    def __init__(self, initial_value: float=0.0) -> None:
        """
        Initialize a Differentiator object with an initial value.
        """
        self._last_value: float = initial_value
        self._last_time: Optional[datetime] = None

    def compute(self, current_value: float, current_time: datetime) -> float:
        """
        Compute the slope of the secant between the current input and the previous. If there is no
        previous input, return the current input.
        """
        if self._last_time:
            df: float = current_value - self._last_value
            dt: float = (current_time - self._last_time).total_seconds()
            output: float = df / dt
        else:
            output = self._last_value
        self._last_value = current_value
        self._last_time = current_time
        return output
