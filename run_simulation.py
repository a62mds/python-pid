from datetime import datetime, timedelta
import random
import time

import matplotlib.pyplot as plt

from src.PID import PIDController


####
# Define parameters here
### PID controller parameters
P_GAIN = 10
I_GAIN = 100
D_GAIN = 0.1
### Process parameters
SETPOINT = 1.0
INITIAL_PROCESS_VARIABLE = 0
### Simulation parameters
TOTAL_DURATION_S = 5
OUTPUT_INTERVAL_S = 0.001
# Define function used to transform controller output to a process variable value
# - Must have signature def ACTUATE(process_variable, controller_output)
# - Must return an updated value of the process variable
###
def ACTUATE(process_variable: float, controller_output: float)  -> float:
    output = 0.01 * controller_output
    random_fluctuation = 0.001 * random.random()
    return (process_variable + output) * (1.0 + random_fluctuation)
####


def main():
    data = run_simulation(
        PIDController(P_GAIN, I_GAIN, D_GAIN, SETPOINT),
        INITIAL_PROCESS_VARIABLE,
        ACTUATE,
        TOTAL_DURATION_S
    )
    
    plot_data(data, SETPOINT, INITIAL_PROCESS_VARIABLE)
    

def run_simulation(controller, initial_process_variable, actuator_function, total_duration_s):
    total_duration = timedelta(seconds=total_duration_s)
    data = [(0.0, initial_process_variable, 0.0, controller.setpoint - initial_process_variable)]
    process_variable = initial_process_variable
    time_elapsed = timedelta(seconds=0)
    start_time = datetime.now()

    while time_elapsed < total_duration:
        current_time = datetime.now()
        time_elapsed = current_time - start_time
        controller_output, error_value = controller.get_output(process_variable)
        process_variable = actuator_function(process_variable, controller_output)
        data.append((time_elapsed.total_seconds(), process_variable, controller_output, error_value))
        time.sleep(OUTPUT_INTERVAL_S)

    return data


def plot_data(data, setpoint, initial_process_variable):
    ts, pvs, *_ = list(zip(*data))
    plt.plot(ts, pvs)
    plt.hlines(setpoint, min(ts), max(ts), linestyles="dashed")
    plt.vlines(0, initial_process_variable, setpoint, linestyles="dashed")
    plt.xlabel("Time [s]")
    plt.ylabel("Process Variable")
    plt.show()

    
if __name__ == "__main__":
    main()