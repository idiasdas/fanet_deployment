    def __init__(self, size_A, heights, base_station, n_drones, n_positions_per_axis, observation_period, targets_trace = []):
            targets_trace (list, optional): The coordinates of each sensor for each time step. Defaults to [].
    def __init__(self, size_A, heights, base_station, n_drones, n_positions_per_axis, observation_period, targets_trace = []):

            targets_trace (list, optional): The coordinates of each sensor for each time step. Defaults to [].
        self.targets_trace = targets_trace
