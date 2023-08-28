from fanet.setup.config import *
from fanet.setup.cplex_constants import *

def test_config() -> None:
    """Tests if the config file is correct."""
    assert isinstance(PARAMETERS, dict)
    assert isinstance(BASE_DIR, str)
    assert isinstance(TESTS_OUTPUT_DIR, str)
    assert isinstance(FILES_DIR, str)
    assert isinstance(BINARY_VARIABLE, str)
    assert isinstance(INTEGER_VARIABLE, str)
    assert isinstance(CONTINUOUS_VARIABLE, str)
    assert isinstance(GREATER_EQUAL, str)
    assert isinstance(EQUAL, str)
    assert isinstance(LESS_EQUAL, str)
    assert isinstance(OPTIMAL_SOLUTION, int)
    assert isinstance(INFEASIBLE_SOLUTION, int)

def test_config_parameters() -> None:
    """Tests if the PARAMETERS dictionary is correct."""
    assert isinstance(PARAMETERS["n_drones"], list)
    for n_drones in PARAMETERS["n_drones"]:
        assert isinstance(n_drones, int)

    assert isinstance(PARAMETERS["n_targets"], list)
    for n_targets in PARAMETERS["n_targets"]:
        assert isinstance(n_targets, int)

    assert isinstance(PARAMETERS["targets_speed"], list)
    for targets_speed in PARAMETERS["targets_speed"]:
        assert isinstance(targets_speed, float) or isinstance(targets_speed, int)

    assert isinstance(PARAMETERS["alpha"], list)
    for alpha in PARAMETERS["alpha"]:
        assert isinstance(alpha, float) or isinstance(alpha, int)

    assert isinstance(PARAMETERS["n_positions"], list)
    for n_positions in PARAMETERS["n_positions"]:
        assert isinstance(n_positions, int)

    assert isinstance(PARAMETERS["heights"], list)
    for heights in PARAMETERS["heights"]:
        assert isinstance(heights, float) or isinstance(heights, int)

    assert isinstance(PARAMETERS["base_station"], tuple)
    assert isinstance(PARAMETERS["base_station"][0], float) or isinstance(PARAMETERS["base_station"][0], int)
    assert isinstance(PARAMETERS["base_station"][1], float) or isinstance(PARAMETERS["base_station"][1], int)
    assert isinstance(PARAMETERS["base_station"][2], float) or isinstance(PARAMETERS["base_station"][2], int)
    assert len(PARAMETERS["base_station"]) == 3

    assert isinstance(PARAMETERS["beta"], float) or isinstance(PARAMETERS["beta"], int)
    assert isinstance(PARAMETERS["area_size"], float) or isinstance(PARAMETERS["area_size"], int)
    assert isinstance(PARAMETERS["observation_period"], int)
    assert isinstance(PARAMETERS["time_step_delta"], float) or isinstance(PARAMETERS["time_step_delta"], int)
    assert isinstance(PARAMETERS["comm_range"], float) or isinstance(PARAMETERS["comm_range"], int)
    assert isinstance(PARAMETERS["coverage_angle"], float) or isinstance(PARAMETERS["coverage_angle"], int)
    assert isinstance(PARAMETERS["n_instances"], int)
