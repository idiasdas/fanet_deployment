import os
import sys
this_dirctory = os.path.dirname(__file__)
sys.path.append( this_dirctory + '/../src/')  # Now can import modules in src
from targets_trace import Trace
from graph import Graph
import numpy as np

print(" - Testing Trace class")

n_targets = 5
observation_period = 3
target_speed = 20
area_size = 100

sensors_trace = Trace(n_targets, observation_period, target_speed, area_size)
out_txt = "\t* {variable:20} = {test_value:<10}"
print(" - Created new instante:")
print(out_txt.format(variable = "n_targets", test_value = str(sensors_trace.n_targets)))
print(out_txt.format(variable = "observation_period", test_value = str(sensors_trace.observation_period)))
print(out_txt.format(variable = "target_speed", test_value = str(sensors_trace.target_speed)))
print(out_txt.format(variable = "area_size", test_value = str(sensors_trace.area_size)))
print(out_txt.format(variable = "time_step_delta", test_value = str(sensors_trace.time_step_delta)))

print(" - Plotting trace to " + this_dirctory + "out/test_trace_plot.eps")
sensors_trace.plot_trace(this_dirctory + "/out/test_trace_plot.eps")

print(" - Saving trace to out/test_trace.txt")
sensors_trace.save_trace(this_dirctory + "/out/test_trace.txt")

print(" - Loading trace from out/test_trace.txt")
file_sensors_trace = Trace(load_file = this_dirctory + "/out/test_trace.txt")

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

print(" NOW TESTING CREATION OF FEASIBLE TRACES")

my_graph = Graph(size_A = 100, heights = [45], base_station = (0,0,0), n_positions_per_axis = 3, communication_range = 40, coverage_tan_angle = np.tan(np.pi/6))



n_traces = 5
count_failed_traces = 0
for n in range(n_traces):
    new_trace = Trace(n_targets, observation_period, target_speed, area_size)
    while not my_graph.verify_trace_feasiblity(new_trace.trace_set):
        count_failed_traces += 1
        new_trace = Trace(n_targets, observation_period, target_speed, area_size)
    print(" - Trace " + str(n) + " is feasible")
    new_trace.save_trace(this_dirctory + "/out/feasible_trace_" + str(n) + ".txt")
    new_trace.plot_trace(this_dirctory + "/out/feasible_trace_" + str(n) + ".eps")
print(" - Done testing creation of feasible traces")
print(" - " + str(count_failed_traces) + " traces were not feasible")



