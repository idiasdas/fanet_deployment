"""This file constains the default parameters for the experiments.
    If you want to use different parameters, create a new dictionary and update PARAMETERS in config.py accordingly.
"""
import numpy as np
#  Default parameters for the experiements.Parameters of type list have all values considered into the experiments.
DEFAULT_PARAMETERS = {
    # number of available drones: list of integers
    "n_drones": [3, 4, 5],
    # number of targets: list of integers
    "n_targets": [1,5,10,20,30,40,50],
    # speed of the targets in m/s: list of floats
    "targets_speed": [10],
    # number of time steps: integer
    "obsertion_period": 5,
    # time between each time step in seconds: float
    "time_step_delta": 1,
    # weight between distance and energy: list of floats
    "alpha": [0],
    # normalization factor: float
    "beta": 0.08095,
    # size of the square area in meters: float
    "area_size": 100,
    # number of axis splits to form the grid: list of integers
    "n_positions":[3],
    # number of axis splits to form the grid: list of floats
    "heights":[45],
    # base station position: tuple of floats (x,y,h)
    "base_station": (0, 0, 0),
    # communication range in meters: float
    "comm_range": 60,
    # coverage angle in radians: float
    "coverage_angle": np.pi/6,
    # number of instances to generate for each parameter combination: integer
    "n_instances":100,
}