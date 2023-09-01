from datetime import datetime, timedelta
import json
import random
import os
import sys
import time

import matplotlib.pyplot as plt

from src.pid import PIDController


# Defaults
# - PID controller parameters
P_GAIN = 10
I_GAIN = 100
D_GAIN = 0.1
# - Simulation parameters
SETPOINT = 1.0
INITIAL_PROCESS_VARIABLE = 0
TOTAL_DURATION_S = 5
OUTPUT_INTERVAL_S = 0.001

# Example actuator function used to transform controller output to a process
# variable value
# - Must have signature def actuate(process_variable: float, controller_output: float)
# - Must return an updated value of the process variable
def actuate(process_variable: float, controller_output: float)  -> float:
    output = 0.01 * controller_output
    random_fluctuation = 0.001 * random.random()
    return (process_variable + output) * (1.0 + random_fluctuation)


def main():
    settings = get_settings()
    controller = PIDController(
        settings["p-gain"],
        settings["i-gain"],
        settings["d-gain"],
        settings["setpoint"]
    )
    print_simulation_header(settings)
    data, run_info = run_simulation(
        controller,
        settings["initial-process-variable"],
        actuate,
        settings["total-duration-s"],
        settings["output-interval-s"]
    )
    print_simulation_run_finished()
    output_data = {"simulation": settings, "run-info": run_info, "data": data}
    os.makedirs("simulation-results", exist_ok=True)
    plot_data(output_data, settings)
    save_data(output_data, settings)


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
    return data, {"start-time": start_time, "end-time": datetime.now()}


def plot_data(output_data, settings):
    data = output_data["data"]
    ts, pvs, *_ = list(zip(*data))
    plt.plot(ts, pvs)
    plt.hlines(settings["setpoint"], min(ts), max(ts), linestyles="dashed")
    plt.vlines(0, settings["initial-process-variable"], settings["setpoint"], linestyles="dashed")
    plt.xlabel("Time [s]")
    plt.ylabel("Process Variable")
    plot_name = f"{get_filename_prefix(output_data['run-info']['start-time'])}.simulation.png"
    plot_path = os.path.join("simulation-results", plot_name)
    plt.savefig(plot_path)


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


def parse_simulation_json(simulation_json_path):
    with open(simulation_json_path, 'r') as simulation_json:
        settings = json.load(simulation_json)["simulation"]
    return settings


def save_data(output_data, settings):
    simulation_out_json_name = f"{get_filename_prefix(output_data['run-info']['start-time'])}.simulation-out.json"
    simulation_out_json_path = os.path.join("simulation-results", simulation_out_json_name)
    output_data['run-info']['start-time'] = datetime.strftime(output_data['run-info']['start-time'], "%Y-%m-%dT%H-%M-%S.%f")
    output_data['run-info']['end-time'] = datetime.strftime(output_data['run-info']['end-time'], "%Y-%m-%dT%H-%M-%S.%f")
    with open(simulation_out_json_path, 'w') as simulation_out_json:
        json.dump(output_data, simulation_out_json, indent=4)


def print_simulation_header(settings):
    print(80*"=")
    print("PIDController Simulation")
    print("\n Controller Parameters:")
    print(f"  P-gain:          {settings['p-gain']}")
    print(f"  I-gain:          {settings['i-gain']}")
    print(f"  D-gain:          {settings['d-gain']}")
    print("\n Simulation Parameters:")
    print(f"  Setpoint:        {settings['setpoint']}")
    print(f"  Initial value:   {settings['initial-process-variable']}")
    print(f"  Duration:        {settings['total-duration-s']} s")
    print(f"  Output interval: {settings['output-interval-s']} s")
    print(f"\n Start Time: {datetime.strftime(datetime.now(), '%H:%M:%S.%f')}")
    print(f"  Running...")


def print_simulation_run_finished():
    print("  Finished running simulation")
    print(f" End Time:   {datetime.strftime(datetime.now(), '%H:%M:%S.%f')}")


def get_filename_prefix(dt):
    return f"{datetime.strftime(dt, '%Y-%m-%dT%H-%M-%S')}"

    
if __name__ == "__main__":
    main()