from typing import Optional
from matplotlib.patches import Circle
import matplotlib.pyplot as plt
from matplotlib import collections as mc
from fanet.targets_trace import TargetsTrace
from fanet.graph import Graph
from fanet.setup.config import FILES_DIR

def plot_trace(trace: TargetsTrace, file_name: Optional[str] = FILES_DIR + "trace_plot.eps") -> None:
    """Plots the trace of targets during the whole observation period. Targets are represented by green Xs and their movements are shown with blue lines between subsequent positions.

    Args:
        trace (TargetsTrace): Trace of targets.
        file_name (Optional[str], optional): Name of the file where the plot will be saved. Defaults to FILES_DIR + "trace_plot.eps".
    """
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set(xlim=(0, trace.area_size), ylim=(0, trace.area_size))
    plt.xticks(fontsize=16)
    plt.yticks(fontsize=16)

    # Ploting the history of the sensors
    for target_trace in trace.trace_set:
        X_target = [target_position[0] for target_position in target_trace]
        Y_target = [target_position[1] for target_position in target_trace]
        edges = []
        for time_step in range(1, trace.observation_period):
            edges.append([target_trace[time_step - 1], target_trace[time_step]])
        lines = mc.LineCollection(edges, linestyle=":", color="blue")  # lines between subsequent positions

        ax.scatter(X_target, Y_target, color="green", marker="x", s=120)  # targets will be green X
        ax.add_collection(lines)

    fig.savefig(file_name, bbox_inches="tight", format="eps")

def plot_trace_with_coverage(trace: TargetsTrace, graph: Graph, file_name: Optional[str] = FILES_DIR + "trace_plot.png") -> None:
    """Plots the targets trace along with the area coverage of the drones.

    Args:
        trace (TargetsTrace): Trace of targets.
        graph (Graph): Graph of the network.
        file_name (Optional[str], optional): Name of the file where the plot will be saved. Defaults to FILES_DIR + "trace_plot.png".
    """
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set(xlim=(0, trace.area_size), ylim=(0, trace.area_size))
    plt.xticks(fontsize=16)
    plt.yticks(fontsize=16)

    # Ploting the history of the sensors
    for target_trace in trace.trace_set:
        X_target = [target_position[0] for target_position in target_trace]
        Y_target = [target_position[1] for target_position in target_trace]
        edges = []
        for time_step in range(1, trace.observation_period):
            edges.append([target_trace[time_step - 1], target_trace[time_step]])

        ax.scatter(X_target, Y_target, color="green", marker="x", s=120)  # targets will be green X
    for position in graph.deployment_positions:
        ax.scatter(position[0], position[1], color="red", marker="o", s=120)
        position_coverage = Circle((position[0],position[1]), graph.coverage_tan_angle*position[2], color="r", alpha=0.1)
        for neighboor in graph.get_positions_in_comm_range(position):
            ax.plot([position[0], neighboor[0]], [position[1], neighboor[1]], color="b", linestyle=":", linewidth=0.5)
        ax.add_patch(position_coverage)
    for neighboor in graph.get_positions_in_comm_range(graph.base_station):
        ax.plot([graph.base_station[0], neighboor[0]], [graph.base_station[1], neighboor[1]], color="b", linestyle=":", linewidth=0.5)

    fig.savefig(file_name, bbox_inches="tight", format="png")

def plot_trace_per_time_step(trace: TargetsTrace, graph: Graph, file_name: Optional[str] = FILES_DIR + "trace_plot.png") -> None:
    """Plots the trace of all targets for each time step.

    Args:
        trace (TargetsTrace): Trace of targets.
        graph (Graph): Graph of the network.
        file_name (Optional[str], optional): Name of the file where the plot will be saved. Defaults to FILES_DIR + "trace_plot.png". This leads to the creation of multiple files named trace_plot_0.png, trace_plot_1.png, etc. for each time step."""
    for time_step in range(trace.observation_period):
        fig, ax = plt.subplots(figsize=(6, 6))
        ax.set(xlim=(0, trace.area_size), ylim=(0, trace.area_size))
        plt.xticks(fontsize=16)
        plt.yticks(fontsize=16)

        # Ploting the history of the sensors
        for target_trace in trace.trace_set:
            X_target = target_trace[time_step][0]
            Y_target = target_trace[time_step][1]
            ax.scatter(X_target, Y_target, color="green", marker="x", s=120)  # targets will be green X
        for position in graph.deployment_positions:
            ax.scatter(position[0], position[1], color="red", marker="o", s=120)
            position_coverage = Circle((position[0],position[1]), graph.coverage_tan_angle*position[2], color="r", alpha=0.1)
            for neighboor in graph.get_positions_in_comm_range(position):
                ax.plot([position[0], neighboor[0]], [position[1], neighboor[1]], color="b", linestyle=":", linewidth=0.5)
            ax.add_patch(position_coverage)
        for neighboor in graph.get_positions_in_comm_range(graph.base_station):
            ax.plot([graph.base_station[0], neighboor[0]], [graph.base_station[1], neighboor[1]], color="b", linestyle=":", linewidth=0.5)

        fig_name = file_name[:-4] + f"_{time_step}.png"
        fig.savefig(fig_name, bbox_inches="tight", format="png")
