from import_file import *
class Graph:
    def __init__(self, size_A, heights, base_station, n_drones, n_positions_per_axis, observation_period, targets_trace = []):
        """Describes the graph form which we build the linear programs.

        Args:
            size_A (float): Lenght of the square area A
            heights (list): Which heights are allowed
            base_station (tuple): Coordinates of the base station
            n_drones (int): Amount of drones available
            n_positions_per_axis (int): How many slices we want to divide the area into for each axis. The resulting grid will have n_positions_per_axis^2 positions.
            observation_period (int): Amount of time steps
            targets_trace (list, optional): The coordinates of each sensor for each time step. Defaults to [].
        """
        self.size_A = size_A
        self.heights = heights
        self.base_station = base_station
        self.n_drones = n_drones
        self.n_positions_per_axis = n_positions_per_axis
        self.observation_period = observation_period
        self.targets_trace = targets_trace

    def set_position_grid(self):
        """ Sets the set P as all the positions in the grid inside the area A."""
        self.deployment_positions = np.array([(x, y, z) for x in np.arange(0, self.size_A, self.size_A/(self.n_positions_per_axis + 1)) for y in np.arange(0, self.size_A, self.size_A/(self.n_positions_per_axis + 1)) for z in self.heights if x!= 0 and y != 0])

    def get_target_coverage(self, target_position, coverage_range):
        """Returns the set of positions that cover the target position.

        Args:
            target_position (tuple): Coordinates of the target position
            coverage_range (float): Maximum distance from a position to the target position

        Returns:
            list: List of positions that cover the target position
        """
        return [position for position in self.deployment_positions if np.linalg.norm(np.array(position) - np.array(target_position)) <= coverage_range]
    
    def get_positions_in_comm_range(self, position, communication_range):
        """Returns the set of positions that are in communication range with the given position.

        Args:
            position (tuple): Coordinates of the position
            communication_range (float): Maximum distance from a position to the base station

        Returns:
            list: List of positions that are in communication range of the base station
        """
        return [p for p in self.deployment_positions if np.linalg.norm(np.array(p) - np.array(position)) <= communication_range]
    
