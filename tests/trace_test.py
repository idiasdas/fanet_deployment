import sys
this_dirctory = __file__.split('trace_test.py')[0]  # directory of trace_test.py
sys.path.append(this_dirctory + '../src/')          # Now can import src modules
from import_file import *
from targets_trace import Trace

print(" - Testing Trace class")
sensors_trace = Trace(5,3,20,100)
out_txt = "\t* {variable:20} = {test_value:<10}"
print(" - Created new instante:")
print(out_txt.format(variable = "n_targets", test_value = str(sensors_trace.n_targets)))
print(out_txt.format(variable = "observation_period", test_value = str(sensors_trace.observation_period)))
print(out_txt.format(variable = "target_speed", test_value = str(sensors_trace.target_speed)))
print(out_txt.format(variable = "area_size", test_value = str(sensors_trace.area_size)))
print(out_txt.format(variable = "time_step_delta", test_value = str(sensors_trace.time_step_delta)))

print(" - Plotting trace to " + this_dirctory + "out/test_trace_plot.eps")
sensors_trace.plot_trace(this_dirctory + "out/test_trace_plot.eps")

print(" - Saving trace to out/test_trace.txt")
sensors_trace.save_trace(this_dirctory + "out/test_trace.txt")

print(" - Loading trace from out/test_trace.txt")
file_sensors_trace = Trace(load_file = this_dirctory + "out/test_trace.txt")

test_result = True
print(" - Comparing traces")
print(out_txt.format(variable = "n_targets", test_value = str(sensors_trace.n_targets == file_sensors_trace.n_targets)))
test_result = test_result and sensors_trace.n_targets == file_sensors_trace.n_targets
print(out_txt.format(variable = "observation_period", test_value = str(sensors_trace.observation_period == file_sensors_trace.observation_period)))
test_result = test_result and sensors_trace.observation_period == file_sensors_trace.observation_period
print(out_txt.format(variable = "target_speed", test_value = str(sensors_trace.target_speed == file_sensors_trace.target_speed)))
test_result = test_result and sensors_trace.target_speed == file_sensors_trace.target_speed
print(out_txt.format(variable = "area_size", test_value = str(sensors_trace.area_size == file_sensors_trace.area_size)))
test_result = test_result and sensors_trace.area_size == file_sensors_trace.area_size
print(out_txt.format(variable = "time_step_delta", test_value = str(sensors_trace.time_step_delta == file_sensors_trace.time_step_delta)))
test_result = test_result and sensors_trace.time_step_delta == file_sensors_trace.time_step_delta
print(out_txt.format(variable = "trace_set", test_value = str(sensors_trace.trace_set == file_sensors_trace.trace_set)))
test_result = test_result and sensors_trace.trace_set == file_sensors_trace.trace_set

print(" TEST RESULT = "  + ("OK" if str(test_result) else "FAIL"))