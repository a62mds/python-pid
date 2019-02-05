from datetime import datetime, timedelta
import json
import random
import os
import sys
import time

import matplotlib.pyplot as plt

from src.PID import PIDController


# Defaults
# - PID controller parameters
P_GAIN = 10
I_GAIN = 100
D_GAIN = 0.1
# - Process parameters
SETPOINT = 1.0
INITIAL_PROCESS_VARIABLE = 0
# - Simulation parameters
TOTAL_DURATION_S = 5
OUTPUT_INTERVAL_S = 0.001

# Example actuator function used to transform controller output to a process
# variable value
# - Must have signature def ACTUATE(process_variable: float, controller_output: float)
# - Must return an updated value of the process variable
###
def ACTUATE(process_variable: float, controller_output: float)  -> float:
    output = 0.01 * controller_output
    random_fluctuation = 0.001 * random.random()
    return (process_variable + output) * (1.0 + random_fluctuation)
####


def main():
    settings = get_settings()

    controller = PIDController(
        settings["p-gain"],
        settings["i-gain"],
        settings["d-gain"],
        settings["setpoint"]
    )

    print_simulation_header(settings)

    data = run_simulation(
        controller,
        settings["initial-process-variable"],
        ACTUATE,
        settings["total-duration-s"],
        settings["output-interval-s"]
    )
    
    plot_data(data, settings["setpoint"], settings["initial-process-variable"])
    # save_data(data) # save to a simulation-out.json


def get_settings():
    if len(sys.argv) < 2:
        settings = get_default_settings()
    else:
        simulation_json_path = os.path.realpath(sys.argv[1])
        if not os.path.isfile(simulation_json_path):
            print(f"***ERROR: Cannot find file {simulation_json_path}")
            sys.exit(-1)
        else:
            settings = parse_simulation_json(simulation_json_path)
    return settings


def get_default_settings():
    return {
        "p-gain": P_GAIN,
        "i-gain": I_GAIN,
        "d-gain": D_GAIN,
        "setpoint": SETPOINT,
        "initial-process-variable": INITIAL_PROCESS_VARIABLE,
        "total-duration-s": TOTAL_DURATION_S,
        "output-interval-s": OUTPUT_INTERVAL_S
    }


def print_simulation_header(settings):
    print(80*"=")
    print()


def parse_simulation_json(simulation_json_path):
    with open(simulation_json_path, 'r') as simulation_json:
        settings = json.load(simulation_json)["simulation"]
    return settings
    

def run_simulation(controller, initial_process_variable, actuator_function, total_duration_s, output_interval_s):
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
        time.sleep(output_interval_s)

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