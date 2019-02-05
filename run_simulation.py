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
### Actuation parameters  
MAX_RANDOM_ERROR = 0.001
OUTPUT_ACTUATION_FACTOR = 0.01
### Simulation parameters
TOTAL_DURATION_S = 5
####


def main():
    data = run_simulation(
        PIDController(P_GAIN, I_GAIN, D_GAIN, SETPOINT),
        INITIAL_PROCESS_VARIABLE,
        lambda pv, co: actuate(pv, co, MAX_RANDOM_ERROR, OUTPUT_ACTUATION_FACTOR),
        TOTAL_DURATION_S
    )
    
    plot_data(data, SETPOINT, INITIAL_PROCESS_VARIABLE)

    
def actuate(process_variable, controller_output, max_random_error, output_actuation_factor):
    output_actuation = process_variable + output_actuation_factor * controller_output
    random_fluctuation = output_actuation * max_random_error * random.random()
    return output_actuation + random_fluctuation
    

def run_simulation(controller, process_variable, actuator_function, total_duration_s):
    total_duration = timedelta(seconds=total_duration_s)
    start_time = datetime.now()
    data = [(0.0, process_variable, 0.0, controller.setpoint - process_variable)]
    time_elapsed = timedelta(seconds=0)
    while time_elapsed < total_duration:
        current_time = datetime.now()
        time_elapsed = current_time - start_time
        controller_output, error_value = controller.get_output(process_variable)
        process_variable = actuator_function(process_variable, controller_output)
        data.append((time_elapsed.total_seconds(), process_variable, controller_output, error_value))
        time.sleep(0.001) # 1 ms
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