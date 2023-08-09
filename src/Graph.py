    def __init__(self, size_A, heights, base_station, n_drones, n_positions_per_axis, observation_period, targets_trace = []):
            targets_trace (list, optional): The coordinates of each sensor for each time step. Defaults to [].
        self.targets_trace = targets_trace
        self.deployment_positions = np.array([(x, y, z) for x in np.arange(0, self.size_A, self.size_A/(self.n_positions_per_axis + 1)) for y in np.arange(0, self.size_A, self.size_A/(self.n_positions_per_axis + 1)) for z in self.heights if x!= 0 and y != 0])

