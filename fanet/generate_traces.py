"""This script generates feasible traces and saves them in the /fanet_deployment/files/traces/ directory.
    If the traces already exist, it does nothing."""

import os
import numpy as np
from fanet.targets_trace import Trace
from fanet.graph import Graph
from fanet.config import FILES_DIR, PARAMETERS

def create_trace(n_targets, observation_period, target_speed, area_size, my_graph):
    """" Creates a trace and saves it in the /fanet_deployment/files/traces/ directory. It only creates traces that are feasible, i.e., all targets are covered by at least one position for the given graph.

    Args:
        n_targets: Number of targets
        observation_period: Number of time steps
        target_speed: Speed of the targets
        area_size: Size of the area
        my_graph: Graph object to determine feasibility.

    Returns:
        True if the trace was created, False if it already existed."""
    file_name = FILES_DIR + "/traces/trace_nt_" + str(n_targets) + "_t_" + str(observation_period) + "_v_" + str(target_speed) + "_i_" + str(n) + ".txt"
    if os.path.isfile(file_name):
        print("Trace already exists: " + file_name)
        return False
    new_trace = Trace(n_targets, observation_period, target_speed, area_size)
    while not my_graph.verify_trace_feasiblity(new_trace.trace_set):
        new_trace = Trace(n_targets, observation_period, target_speed, area_size)
    new_trace.save_trace(file_name)
    return True

if __name__ == "__main__":
    """This script generates feasible traces and saves them in the FILES_DIR/traces/ directory using the default parameters specified in config.py. Every trace generated has to be feasible, i.e., all targets are covered by at least one position for the following graph."""
    my_graph = Graph(size_A = PARAMETERS["area_size"],
                     heights = PARAMETERS["heights"],
                     base_station = PARAMETERS["base_station"],
                     n_positions_per_axis = PARAMETERS["n_positions"][0],
                     communication_range = PARAMETERS["comm_range"],
                     coverage_angle = PARAMETERS["coverage_angle"])


    for n_targets in PARAMETERS["n_targets"]:
        for target_speed in PARAMETERS["targets_speed"]:
            for n in range(PARAMETERS["n_instances"]):
                create_trace(n_targets,
                                PARAMETERS["obsertion_period"],
                                target_speed,
                                PARAMETERS["area_size"],
                                my_graph)
