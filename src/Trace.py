from import_file import *


class Trace:
    def __init__(self, n_targets, observation_period, target_speed, area_size, time_step_delta=1):
        """Represents a set of targets moving on a xy plane with the random way point model. The targets are moving at a constant speed and bounce off the walls of the square area A = (area_size x area_size). The trace is a sequence of positions of the targets at each time step. The trace is generated at initialization.

        Args:
            n_targets (int): Amount of targets
            observation_period (int): Number of time steps
            target_speed (float): Targets speed in m/s
            area_size (float): Lenght of the square area A
            time_step_delta (float, optional): Amount of seconds between time steps. Defaults to 1.
        """
        self.n_targets = n_targets                      
        self.observation_period = observation_period    
        self.target_speed = target_speed                
        self.area_size = area_size                      
        self.time_step_delta = time_step_delta          

        self.generate_all_traces()

    def generate_random_direction(self):
        """Generates a random normalized vector of dimension 2, returns as a tuple."""
        direction = (np.random.rand(2) - 0.5)*2
        normalized_direction = tuple(direction/np.linalg.norm(direction))
        return normalized_direction

    def generate_target_trace(self):
        """Generates a sequence of positions (list of tuples) of n_targets inside the area A = (x_max, y_max) with speed target_speed. Random way point model"""
        target_trace = [tuple(np.random.rand(2)*self.area_size)]  # initial position
        for t in range(self.observation_period - 1):
            # Last position plus random direction times speed times time. Bounces off the walls. 
            direction = self.generate_random_direction()
            last_x = target_trace[-1][0]
            last_y = target_trace[-1][1]
            new_x = last_x + direction[0] * self.target_speed * self.time_step_delta
            new_y = last_y + direction[1] * self.target_speed * self.time_step_delta
            if new_x > self.area_size:
                new_x = self.area_size - (new_x - self.area_size)
            if new_x < 0:
                new_x = abs(new_x)
            if new_y > self.area_size:
                new_y = self.area_size - (new_y - self.area_size)
            if new_y < 0:
                new_y = abs(new_y)
            target_trace += [(new_x, new_y)]
        return target_trace

    def generate_all_traces(self):
        """Generates a list with the traces of all targets. """
        self.trace_set = []
        for i in range(self.n_targets):
            self.trace_set.append(self.generate_target_trace())

    def verify_feasiblity(self, deployment_positions, coverage_range):
        """Verifies a set of deployment positions with a given coverage range can cover all targets at all time steps. If any target cannot be covered at any time step, returns False."""
        for target_trace in self.trace_set:
            for target_position in target_trace:
                feasible = False
                for deployment_position in deployment_positions:
                    if np.linalg.norm(target_position - deployment_position) <= coverage_range:
                        feasible = True
                if not feasible:
                    return False
        return True

    def plot_trace(self, file_name = "trace_plot.eps"):
        """Plots the trace of all targets."""
        fig, ax = plt.subplots(figsize=(6,4))
        ax.set(xlim=(0, self.area_size), ylim=(0, self.area_size))
        plt.xticks(fontsize=16)
        plt.yticks(fontsize=16)

        # Ploting the history of the sensors
        for target_trace in self.trace_set:
            X_target = [target_position[0] for target_position in target_trace]
            Y_target = [target_position[1] for target_position in target_trace]
            edges = []
            for time_step in range(1,self.observation_period):
                edges.append([target_trace[time_step-1],target_trace[time_step]])
            lines = mc.LineCollection(edges , linestyle=":",color = "blue")# lines between subsequent positions
            
            ax.scatter(X_target,Y_target, color='green',marker="x",s=120) # targets will be green X
            ax.add_collection(lines)                      

        fig.savefig(file_name, bbox_inches='tight', format = "eps")

