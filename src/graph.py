try:
    import numpy as np
    from typing import Optional
except ImportError:
    print("Error while importing basic modules. Please refer to the README.md file for instructions on how to install the required modules.")
    exit(1)

class Graph:
    def __init__(self, size_A: float, heights: float, base_station: tuple, n_drones: int, n_positions_per_axis: int, observation_period: int, communication_range: float, coverage_range: float, targets_trace: Optional[list] = []):
        """Describes the graph form which we build the linear programs.

        Args:
            size_A: Lenght of the square area A
            heights: Which heights are allowed
            base_station: Coordinates of the base station
            n_drones: Amount of drones available
            n_positions_per_axis: How many slices we want to divide the area into for each axis. The resulting grid will have n_positions_per_axis^2 positions.
            observation_period: Amount of time steps
            communication_range: Maximum distance for communications drone to drone and base to drone 
            coverage_range: Maximum distance for communications drone to target
            targets_trace: The coordinates of each sensor for each time step. Defaults to [].
        """
        self.size_A = size_A
        self.heights = heights
        self.base_station = base_station
        self.n_drones = n_drones
        self.n_positions_per_axis = n_positions_per_axis
        self.observation_period = observation_period
        self.communication_range = communication_range
        self.coverage_range = coverage_range
        self.targets_trace = targets_trace

        self.set_position_grid()

    def set_position_grid(self):
        """ Sets the set P as all the positions in the grid inside the area A."""
        self.deployment_positions = np.array([(x, y, z) for x in np.arange(0, self.size_A, self.size_A/(self.n_positions_per_axis + 1)) for y in np.arange(0, self.size_A, self.size_A/(self.n_positions_per_axis + 1)) for z in self.heights if x!= 0 and y != 0])

    def get_target_coverage(self, target_position: tuple) -> list:
        """Returns the set of positions that cover the target position.

        Args:
            target_position: Coordinates of the target position

        Returns:
            List of positions in P that cover the target position
        """
        return [position for position in self.deployment_positions if np.linalg.norm(np.array(position) - np.array(target_position)) <= self.coverage_range]
    
    def get_positions_in_comm_range(self, position: tuple) -> list:
        """Returns the set of positions that are in communication range with the given position.

        Args:
            position: Coordinates of the position
            communication_range: Maximum distance from a position to the base station

        Returns:
            List of positions that are in communication range of the base station
        """
        return [p for p in self.deployment_positions if np.linalg.norm(np.array(p) - np.array(position)) <= self.communication_range and p != position]
    
