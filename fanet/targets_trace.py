from typing import Optional
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import collections as mc

class TargetsTrace:
    def __init__(self, n_targets: Optional[int] = 5, observation_period: Optional[int] = 5, target_speed: Optional[float] = 5, area_size: Optional[float] = 100, time_step_delta: Optional[float] = 1, load_file: Optional[str] = "") -> None:
        """Represents the trajectories of n targets inside an square area A during observation_period time steps. The targets move at a constant speed target_speed. The area A has size area_size^2 and the time between time steps is time_step_delta. If a load file is provided, the trace is loaded from the file. Otherwise, a new trace is generated.

        Args:
            n_targets: Number of targets. Defaults to 5.
            observation_period: Number of time steps. Defaults to 5.
            target_speed: Targets speed in m/s. Defaults to 5.
            area_size: Lenght of the square area A. Defaults to 100.
            time_step_delta: Amount of seconds between time steps. Defaults to 1.
            load_file: path + name of file with trace description. Refer to Trace.save_trace() to see the file format. Defaults to "".
        """
        if load_file == "":
            self.n_targets = n_targets
            self.observation_period = observation_period
            self.target_speed = target_speed
            self.area_size = area_size
            self.time_step_delta = time_step_delta

            self.trace_set = self.generate_all_traces()
        else:
            self.load_trace(load_file)

    def generate_random_direction(self) -> tuple:
        """Generates a random normalized vector of dimension 2, returns as a tuple (x,y)."""
        direction = (np.random.rand(2) - 0.5) * 2
        normalized_direction = tuple(direction / np.linalg.norm(direction))
        return normalized_direction

    def wall_bounce(self, coordinate: float) -> float:
        """ Bounces a coordinate axis from the walls.
            area_size + 1 -> area_size - 1,
            2 * area_size + 1 -> 1,
            -1 -> 1,
            - area_size - 1 -> area_size - 1,
            etc.
        Args:
            coordinate: value of target x or y position.

        Returns:
            Bounced coordinate.
        """
        coordinate = abs(coordinate)
        if coordinate > self.area_size:
            sign = ((coordinate // self.area_size) % 2)
            rest = coordinate % self.area_size
            coordinate = self.area_size * sign  + (-1) ** sign * rest
        return coordinate

    def generate_target_trace(self) -> list:
        """Generates a sequence o`f positions (list of tuples) of one target inside the area A = (x_max, y_max) with speed target_speed. Random way point model"""
        target_trace = [tuple(np.random.rand(
            2) * self.area_size)]  # initial position
        for t in range(self.observation_period - 1):
            # Last position plus random direction times speed times time.
            direction = self.generate_random_direction()
            last_x = target_trace[-1][0]
            last_y = target_trace[-1][1]
            new_x = last_x + direction[0] * self.target_speed * self.time_step_delta
            new_y = last_y + direction[1] * self.target_speed * self.time_step_delta
            # Bounces off the walls.
            new_x = self.wall_bounce(new_x)
            new_y = self.wall_bounce(new_y)
            target_trace += [(new_x, new_y)]
        return target_trace

    def generate_all_traces(self) -> list:
        """Generates a list with the traces of all targets.

        Returns:
            List of traces, where each trace is a list of positions (tuples (x,y)) of one target.
        """
        trace_set = []
        for i in range(self.n_targets):
            trace_set.append(self.generate_target_trace())
        return trace_set

    def plot_trace(self, file_name: Optional[str] = "trace_plot.eps") -> None:
        """Plots the trace of all targets."""
        fig, ax = plt.subplots(figsize=(6, 4))
        ax.set(xlim=(0, self.area_size), ylim=(0, self.area_size))
        plt.xticks(fontsize=16)
        plt.yticks(fontsize=16)

        # Ploting the history of the sensors
        for target_trace in self.trace_set:
            X_target = [target_position[0] for target_position in target_trace]
            Y_target = [target_position[1] for target_position in target_trace]
            edges = []
            for time_step in range(1, self.observation_period):
                edges.append([target_trace[time_step - 1], target_trace[time_step]])
            lines = mc.LineCollection(edges, linestyle=":", color="blue")  # lines between subsequent positions

            ax.scatter(X_target, Y_target, color="green", marker="x", s=120)  # targets will be green X
            ax.add_collection(lines)

        fig.savefig(file_name, bbox_inches="tight", format="eps")

    def get_targets_positions_at_time(self, time_step: int) -> list:
        """Returns the positions of all targets at a given time step.

        Args:
            time_step: Time step

        Returns:
            List of positions of all targets at time_step
        """
        return [target_trace[time_step] for target_trace in self.trace_set]

    def save_trace(self, file_name: str) -> None:
        """Saves the trace to a file.

        Args:
            file_name: path + name of file where the trace will be saved.
        """
        trace_file = open(file_name, "w")

        trace_file.write("n_targets = " + str(self.n_targets) + "\n")
        trace_file.write("observation_period = " + str(self.observation_period) + "\n")
        trace_file.write("target_speed = " + str(self.target_speed) + "\n")
        trace_file.write("area_size = " + str(self.area_size) + "\n")
        trace_file.write("time_step_delta = " + str(self.time_step_delta) + "\n")

        for target_trace in self.trace_set:
            for target_position in target_trace:
                trace_file.write(str(target_position[0]) + " " + str(target_position[1]) + "\n")

        trace_file.close()

    def load_trace(self, file_name: str) -> None:
        """Loads a trace from a file.

        Args:
            file_name: path + name of file where the trace is saved.
        """
        trace_file = open(file_name, "r")

        self.n_targets = int(trace_file.readline().split(" = ")[1])
        self.observation_period = int(trace_file.readline().split(" = ")[1])
        self.target_speed = float(trace_file.readline().split(" = ")[1])
        self.area_size = float(trace_file.readline().split(" = ")[1])
        self.time_step_delta = float(trace_file.readline().split(" = ")[1])

        self.trace_set = []
        for i in range(self.n_targets):
            target_trace = []
            for t in range(self.observation_period):
                target_position = trace_file.readline().split(" ")
                target_trace.append((float(target_position[0]), float(target_position[1])))
            self.trace_set.append(target_trace)

        trace_file.close()
