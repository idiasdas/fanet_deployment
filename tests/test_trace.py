import os
this_dirctory = os.path.dirname(__file__)
from fanet.targets_trace import Trace

def test_trace_creation() -> None:
    """Creates a trace file, saves it and then loads it again. Verifies that the loaded trace is the same as the original one.
    """
    n_targets = 5
    observation_period = 3
    target_speed = 20
    area_size = 100
    sensors_trace = Trace(n_targets, observation_period, target_speed, area_size)
    sensors_trace.save_trace(this_dirctory + "/out/test_trace.txt")
    file_sensors_trace = Trace(load_file = this_dirctory + "/out/test_trace.txt")

    assert sensors_trace.n_targets == file_sensors_trace.n_targets
    assert sensors_trace.observation_period == file_sensors_trace.observation_period
    assert sensors_trace.target_speed == file_sensors_trace.target_speed
    assert sensors_trace.area_size == file_sensors_trace.area_size
    assert sensors_trace.time_step_delta == file_sensors_trace.time_step_delta
    assert sensors_trace.trace_set == file_sensors_trace.trace_set

def test_wall_bounce() -> None:
    """Tests if the function bounces the targets off the walls.
    """
    n_targets = 100
    observation_period = 2
    target_speed = 200
    area_size = 10
    sensors_trace = Trace(n_targets, observation_period, target_speed, area_size)
    assert sensors_trace.wall_bounce(0) == 0
    assert sensors_trace.wall_bounce(area_size) == area_size
    assert sensors_trace.wall_bounce(-1) == 1
    assert sensors_trace.wall_bounce(- area_size - 1) == area_size - 1
    assert sensors_trace.wall_bounce(- area_size * 2 - 1) == 1
    assert sensors_trace.wall_bounce(- area_size * 3 - 1) == area_size - 1
    assert sensors_trace.wall_bounce(- area_size * 4 - 1) == 1
    assert sensors_trace.wall_bounce(area_size + 1) == area_size - 1
    assert sensors_trace.wall_bounce(area_size * 2 + 1) == 1
    assert sensors_trace.wall_bounce(area_size * 3 + 1) == area_size - 1
    assert sensors_trace.wall_bounce(area_size * 4 + 1) == 1


def test_trace_within_bounds() -> None:
    """Tests if the targets remain inside the area.
    """
    n_targets = 100
    observation_period = 2
    target_speed = 200
    area_size = 10
    sensors_trace = Trace(n_targets, observation_period, target_speed, area_size)
    for target_trace in sensors_trace.trace_set:
        for position in target_trace:
            assert position[0] >= 0 and position[0] <= area_size
            assert position[1] >= 0 and position[1] <= area_size

def test_plot() -> None:
    """Tests if the function creates an eps file.
    """
    n_targets = 5
    observation_period = 3
    target_speed = 20
    area_size = 100
    sensors_trace = Trace(n_targets, observation_period, target_speed, area_size)
    sensors_trace.plot_trace(this_dirctory + "/out/test_trace_plot.eps")

    assert os.path.exists(this_dirctory + "/out/test_trace_plot.eps")
    os.remove(this_dirctory + "/out/test_trace_plot.eps")