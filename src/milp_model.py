from import_file import *

class MILPModel:
    def __init__(self, n_available_drones, observation_period, time_step_delta, communication_range, coverage_range, input_graph):
        """Builds the linear program to obtain the optimal deployment of drones to cover all targets at all time steps.

        Args:
            n_available_drones (int): Number of drones available.
            observation_period (int): Amount of time steps.
            time_step_delta (float): Amount of seconds between time steps.
            communication_range (float): The range of drones communication.
            coverage_range (float): The maximmum distance for communication between a drone and a target.
            input_graph (Graph): The topology of the problem with the set of deployment positions and targets coordinates at each time step.
        """        

        self.n_available_drones = n_available_drones
        self.observation_period = observation_period
        self.time_step_delta = time_step_delta
        self.communication_range = communication_range
        self.coverage_range = coverage_range
        self.input_graph = input_graph

