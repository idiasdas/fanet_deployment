from import_file import *
class Graph:
    P = []
    def __init__(self, size_A, heights, base_station, n_drones, n_positions_per_axis, observation_period, sensors = []):
        """Describes the graph form which we build the linear programs.

        Args:
            size_A (float): Lenght of the square area A
            heights (list): Which heights are allowed
            base_station (tuple): Coordinates of the base station
            n_drones (int): Amount of drones available
            n_positions_per_axis (int): How many slices we want to divide the area into for each axis. The resulting grid will have n_positions_per_axis^2 positions.
            observation_period (int): Amount of time steps
            sensors (list, optional): The coordinates of each sensor for each time step. Defaults to [].
        """
        self.size_A = size_A
        self.heights = heights
        self.base_station = base_station
        self.n_drones = n_drones
        self.n_positions_per_axis = n_positions_per_axis
        self.observation_period = observation_period
        self.sensors = sensors

    def set_position_grid(self):
        """ Sets the set P as all the positions in the grid inside the area A."""
        P = np.array([(x, y, z) for x in np.arange(0, self.size_A, self.size_A/(self.n_positions_per_axis + 1)) for y in np.arange(0, self.size_A, self.size_A/(self.n_positions_per_axis + 1)) for z in self.heights if x!= 0 and y != 0])
