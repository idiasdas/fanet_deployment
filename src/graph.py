from import_file import *
class Graph:
    P = []
    def __init__(self, x_max, y_max, heights, base_station, n_drones, n_positions_per_axis, observation_period, sensors = []):
        """Describes the graph form which we build the linear programs.

        Args:
            x_max (float): Size of the area in the x axis
            y_max (float): Size of the area in the y axis
            heights (list): Which heights are allowed
            base_station (tuple): Coordinates of the base station
            n_drones (int): Amount of drones available
            n_positions_per_axis (int): How many slices we want to divide the area into
            observation_period (int): Amount of time steps
            sensors (list, optional): The coordinates of each sensor for each time step. Defaults to [].
        """
        self.x_max = x_max
        self.y_max = y_max
        self.heights = heights
        self.base_station = base_station
        self.n_drones = n_drones
        self.n_positions_per_axis = n_positions_per_axis
        self.observation_period = observation_period
        self.sensors = sensors

    def set_position_grid(self):
        """ Sets the set P as all the positions in the grid inside the area"""
        P = np.array([(x, y, z) for x in np.arange(0, self.x_max, self.x_max/(self.n_positions_per_axis + 1)) for y in np.arange(0, self.y_max, self.y_max/(self.n_positions_per_axis + 1)) for z in self.heights if x!= 0 and y != 0])

    