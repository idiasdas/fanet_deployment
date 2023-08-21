try:
    import numpy as np
    from typing import Optional
except ImportError:
    print("Error while importing basic modules. Please refer to the README.md file for instructions on how to install the required modules.")
    exit(1)

class Graph:
    def __init__(self, size_A: float, heights: float, base_station: tuple,n_positions_per_axis: int, communication_range: float, coverage_tan_angle: float):
        """Describes the graph form which we build the linear programs.

        Args:
            size_A: Lenght of the square area A
            heights: Which heights are allowed
            base_station: Coordinates of the base station
            n_positions_per_axis: How many slices we want to divide the area into for each axis. The resulting grid will have n_positions_per_axis^2 positions.
            communication_range: Maximum distance for communications drone to drone and base to drone 
            coverage_tan_angle: The tangent of the angle of the drones' coverage. The covered area is a cone with the base on the surface below the drone and the apex on the drone. The higher the drone, the bigger the covered area. To determine the covered area, we multiply the tangent of the angle by the height of the drone.
        """
        self.size_A = size_A
        self.heights = heights
        self.base_station = base_station
        self.n_positions_per_axis = n_positions_per_axis
        self.communication_range = communication_range
        self.coverage_tan_angle = coverage_tan_angle

        self.set_position_grid()

    def set_position_grid(self):
        """ Sets the set P as all the positions in the grid inside the area A."""
        self.deployment_positions = np.array([(x, y, z) for x in np.arange(0, self.size_A, self.size_A/(self.n_positions_per_axis + 1)) for y in np.arange(0, self.size_A, self.size_A/(self.n_positions_per_axis + 1)) for z in self.heights if x!= 0 and y != 0])

    def get_target_coverage(self, target_position: tuple) -> list:
        """Returns the set of positions that cover the target position.

        Args:
            target_position: Coordinates (x,y) of the target position.

        Returns:
            List of positions in P that cover the target position
        """
        return [position for position in self.deployment_positions if self.get_distance((position[0],position[1]), target_position) <= self.coverage_tan_angle*position[2]]
    
    def get_position_coverage(self, position:tuple, targets: list) -> list:
        """Returns the set of targets covered by the position.

        Args:
            position: Coordinates of the position
            targets: List of target positions (tuples (x,y)) at a given time step

        Returns:
            List of targets covered by the position
        """
        return [target for target in targets if self.get_distance((position[0],position[1]),target) <= self.coverage_tan_angle*position[2]]
    
    def get_positions_in_comm_range(self, position: tuple) -> list:
        """Returns the set of positions that are in communication range with the given position.

        Args:
            position: Coordinates of the position
            communication_range: Maximum distance from a position to the base station

        Returns:
            List of positions that are in communication range of the base station
        """
        return [p for p in self.deployment_positions if self.get_distance(p,position) <= self.communication_range and p != position]
    
    def verify_trace_feasiblity(self, targets_trace) -> bool:
        """Verifies if the trace is feasible, i.e., if all targets are covered by at least one position.

        Args:
            targets_trace: List of positions of the targets

        Returns:
            bool: True if every target position is covered by at least one position. False otherwise.
        """
        if targets_trace == []:
            return False

        for target_trace in targets_trace:
            for target_position in target_trace: # Each target visits a sequence of positions depending on the observation period
                if self.get_target_coverage(target_position) == []:
                    return False
        
        return True
    
    def get_distance(self, position1: tuple, position2: tuple) -> float:
        """Returns the distance between two positions.

        Args:
            position1: Coordinates of the first position
            position2: Coordinates of the second position

        Returns:
            Distance between the two positions
        """
        return np.linalg.norm(np.array(position1) - np.array(position2))