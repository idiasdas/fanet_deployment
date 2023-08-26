import os
import sys
this_dirctory = os.path.dirname(__file__)
sys.path.append( this_dirctory + '/../src/')

from targets_trace import Trace

def test_trace_creation():
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