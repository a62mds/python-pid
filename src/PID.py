from collections import deque, namedtuple
from datetime import datetime, timedelta


Gains = namedtuple("Gains", list("pid"))


class PIDController(object):
    
    def __init__(self, p_gain=0, i_gain=0, d_gain=0, setpoint=None):
        self.gains = Gains(p=p_gain, i=i_gain, d=d_gain)
        self.setpoint = setpoint
        self._integral_term_value = None
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
        current_output_time, current_error_value = datetime.now(), self.setpoint - input_value
        p = self._gains.p * current_error_value
        i = self._gains.i * self._integrate(current_error_value, current_output_time)
        d = self._gains.d * self._differentiate(current_error_value, current_output_time)
        self._previous_output_time, self._previous_error_value = current_output_time, current_error_value
        return p + i + d, current_error_value

    def _ensure_setpoint_set(self):
        if self.setpoint is None:
            raise Exception("Setpoint not set")
            
    def _integrate(self, current_error_value, current_time):
        if any(x is None for x in (self._previous_error_value, self._previous_output_time, self._integral_term_value)):
            self._integral_term_value = 0.0
        else:
            e_avg = 0.5 * (current_error_value + self._previous_error_value)
            dt = (current_time - self._previous_output_time).total_seconds()
            self._integral_term_value += e_avg * dt
        return self._integral_term_value
    
    def _differentiate(self, current_error_value, current_time):
        if self._previous_error_value is None or self._previous_output_time is None:
            return 0.0
        de = current_error_value - self._previous_error_value
        dt = (current_time - self._previous_output_time).total_seconds()
        return de / dt
