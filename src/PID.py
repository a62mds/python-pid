from collections import deque, namedtuple
from datetime import datetime, timedelta
from typing import Optional


Gains = namedtuple("Gains", list("pid"))


class PIDController(object):
    
    def __init__(self, p_gain=0, i_gain=0, d_gain=0, setpoint=None):
        self.gains = Gains(p=p_gain, i=i_gain, d=d_gain)
        self.setpoint = setpoint
        self._integrator = Integrator(0.0, "center-approx")
        self._previous_error_value = None
        self._previous_output_time = None
        
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
        current_time = datetime.now()
        current_error_value = self.setpoint - input_value
        p = self._gains.p * current_error_value
        i = self._gains.i * self._integrator.compute(current_error_value, current_time)
        d = self._gains.d * self._differentiate(current_error_value, current_time)
        self._previous_output_time = current_time
        self._previous_error_value = current_error_value
        return p + i + d, current_error_value

    def _ensure_setpoint_set(self):
        if self.setpoint is None:
            raise Exception("Setpoint not set")
    
    def _differentiate(self, current_error_value, current_time):
        if self._previous_error_value is None or self._previous_output_time is None:
            return 0.0
        de = current_error_value - self._previous_error_value
        dt = (current_time - self._previous_output_time).total_seconds()
        return de / dt


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
