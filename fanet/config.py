import os
import numpy as np

#  Default parameters for the experiements.Parameters of type list have all values considered into the experiments.
default_parameters = {
    "n_drones": [3, 4, 5],               # list of integers: number of available drones
    "n_targets": [1,5,10,20,30,40,50],   # list of integers: number of targets
    "obsertion_period": [5],             # list of integers: number of time steps
    "time_step_delta": [1],              # list of floats:   time between each time step in seconds
    "alpha": [0, 0.5, 1],                # list of floats:   weight between distance and energy
    "beta": 0.08095,                     # float:            normalization factor
    "area_size": [100],                  # list of floats:   size of the area in meters
    "comm_range": [60],                  # list of floats:   communication range in meters
    "coverage_angle": np.pi/6,           # float:            coverage angle in radians
    "n_positions":[3],                   # list of integers: number of axis splits to form the grid
    "n_instances":100,                   # integer:          number of instances to generate for each parameter combination
}

# Path to the root directory of the project
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# Path where tests results are stored
TESTS_OUTPUT_DIR = os.path.join(BASE_DIR, "tests", "out")
# Path where files are stored such as traces, solutions and figures.
FILES_DIR = os.path.join(BASE_DIR, "files")

# IF FOR SOME REASON THEY CHANGE THE DEFINITION OF THE CONSTANTS IN THE CPLEX MODULE, WE CAN CHANGE THEM HERE
# Constants for variables type
BINARY_VARIABLE = "B"
INTEGER_VARIABLE = "I"
CONTINUOUS_VARIABLE = "C"
# Constants for constraints sense"
GREATER_EQUAL = "G"
EQUAL = "E"
LESS_EQUAL = "L"
# Constants for CPLEX solution status
OPTIMAL_SOLUTION = 101
INFEASIBLE_SOLUTION = 103