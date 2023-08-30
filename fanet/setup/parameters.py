"""This file constains the default parameters for the experiments.
    If you want to use different parameters, create a new dictionary and update PARAMETERS in config.py accordingly.
"""
import numpy as np
#  Default parameters for the experiements. Parameters of type list have all values considered into the experiments.
DEFAULT_PARAMETERS = {
    # number of available drones: list of integers
    "n_drones": [3, 5, 10],
    # number of targets: list of integers
    "n_targets": [1, 5, 10],
    # speed of the targets in m/s: list of floats
    "targets_speed": [10],
    # number of time steps: integer
    "observation_period": 5,
    # time between each time step in seconds: float
    "time_step_delta": 1,
    # weight between distance and energy: list of floats
    "alpha": [0],
    # normalization factor: float
    "beta": 0.08095,
    # size of the square area in meters: float
    "area_size": 100,
    # number of axis splits to form the grid: list of integers
    "n_positions": [3],
    # number of axis splits to form the grid: list of floats
    "heights": [45],
    # base station position: tuple of floats (x,y,h)
    "base_station": (0, 0, 0),
    # communication range in meters: float
    "comm_range": 60,
    # coverage angle in radians: float
    "coverage_angle": np.pi/6,
    # number of instances to generate for each parameter combination: integer
    "n_instances": 100,
    # cplex maximum memory in MB: integer
    "cplex_workmem_limit": 10000,
    # cplex maximum time in seconds: integer
    "cplex_time_limit": 3*3600,
    # name of the experiment (becomes a folder in FILES_DIR with the results): string
    "experiment_name": "default",
}

TEST_PARAMETERS = {
    "n_drones": [5],
    "n_targets": [10],
    "targets_speed": [10],
    "observation_period": 5,
    "time_step_delta": 1,
    "alpha": [1],
    "beta": 0.08095,
    "area_size": 100,
    "n_positions": [3],
    "heights": [45],
    "base_station": (0, 0, 0),
    "comm_range": 60,
    "coverage_angle": np.pi/6,
    "n_instances": 10,
    "cplex_workmem_limit": 10000,
    "cplex_time_limit": 10,
    "experiment_name": "test",
}

EXPERIMENT_PARAMETERS = {
    "n_drones": [5, 10],
    "n_targets": [10, 20, 30, 40, 50],
    "targets_speed": [10],
    "observation_period": 5,
    "time_step_delta": 1,
    "alpha": [0, 0.5, 1],
    "beta": 0.08095,
    "area_size": 100,
    "n_positions": [3],
    "heights": [45],
    "base_station": (0, 0, 0),
    "comm_range": 60,
    "coverage_angle": np.pi/6,
    "n_instances": 10,
    "cplex_workmem_limit": 10000,
    "cplex_time_limit": 3600,
    "experiment_name": "exp",
}

TEST_TIME_LIMIT = {
    "n_drones": [10],
    "n_targets": [50],
    "targets_speed": [10],
    "observation_period": 5,
    "time_step_delta": 1,
    "alpha": [0.5],
    "beta": 0.08095,
    "area_size": 100,
    "n_positions": [10],
    "heights": [45],
    "base_station": (0, 0, 0),
    "comm_range": 60,
    "coverage_angle": np.pi/6,
    "n_instances": 10,
    "cplex_workmem_limit": 10000,
    "cplex_time_limit": 300,
    "experiment_name": "exp",
}
