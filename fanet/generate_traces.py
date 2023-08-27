"""This script generates feasible traces and saves them in the /fanet_deployment/files/traces/ directory.
    If the traces already exist, it does nothing."""

import os
import numpy as np
from fanet.targets_trace import Trace
from fanet.graph import Graph
from fanet.config import FILES_DIR

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
    n_traces = 20
    n_targets_list = [5,10,20,30,40,50]
    observation_period_list = [3,4,5,6,7]
    target_speed_list = [1,5,10]
    area_size_list = [100]
    my_graph = Graph(size_A=100, heights=[45], base_station=(0, 0, 0),n_positions_per_axis=5, communication_range=40, coverage_tan_angle=np.tan(np.pi/6))

    for n_targets in n_targets_list:
        for observation_period in observation_period_list:
            for target_speed in target_speed_list:
                for area_size in area_size_list:
                    for n in range(n_traces):
                        create_trace(n_targets, observation_period, target_speed, area_size, my_graph)
