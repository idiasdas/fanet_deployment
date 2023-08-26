import os
import sys
this_dirctory = os.path.dirname(__file__)
sys.path.append( this_dirctory + '/../src/')  # Now can import modules in src

import numpy as np
from targets_trace import Trace
from graph import Graph
from milp_model import MILPModel

CPXMIP_OPTIMAL = 101
CPXMIP_INFEASIBLE = 103

def model_shut_up(milp_model):
    milp_model.cplex_model.set_log_stream(None)
    milp_model.cplex_model.set_error_stream(None)
    milp_model.cplex_model.set_warning_stream(None)
    milp_model.cplex_model.set_results_stream(None)

def test_basic_0():
    """For this example, we have one sensor in the middle of the area (50,50,0) and only one deployement solution (50,50,10) for a single time step.
    
    This test verifies if the model is built properly, solved and if the solution is correct.
    
    The expected solution is: 142.82857. This is the distance cost for the drone to leave the base station, go to (50,50,10) and go back to the base station.
    """
    targets_trace = Trace(n_targets=1, observation_period=1)
    targets_trace.trace_set = [[(50,50)]]
    graph = Graph(100, [10], (0,0,0), 1, 100, np.tan(np.pi/6))
    milp_model = MILPModel(n_available_drones=1,
                        observation_period=1,
                        time_step_delta=1,
                        targets_trace=targets_trace,
                        input_graph=graph,
                        alpha = 0,
                        beta = 0)
    
    model_shut_up(milp_model)
    milp_model.build_model()
    milp_model.solve_model()
    out_file = this_dirctory + "/out/test_basic_0.sol"
    assert not os.path.exists(out_file)
    milp_model.cplex_save_solution(out_file)
    assert os.path.exists(out_file)
    os.remove(out_file)
    assert not os.path.exists(out_file)
    assert milp_model.cplex_model.solution.get_status() == CPXMIP_OPTIMAL
    assert round(milp_model.get_objective_value(),5) == 142.82857
    milp_model.cplex_finish()


def test_basic_1():
    """In this example, we have two time steps, where the sensor remains in (50,50,0). So the drone is expected to go to (50,50,10) for both time steps."""
    targets_trace = Trace(n_targets=1, observation_period=2)
    targets_trace.trace_set = [[(50,50),(50,50)]]
    graph = Graph(100, [10], (0,0,0), 1, 100, np.tan(np.pi/6))
    milp_model = MILPModel(n_available_drones=1,
                        observation_period=2,
                        time_step_delta=1,
                        targets_trace=targets_trace,
                        input_graph=graph,
                        alpha = 0,
                        beta = 0)
    
    model_shut_up(milp_model)
    milp_model.build_model()
    milp_model.solve_model()

    assert milp_model.cplex_model.solution.get_status() == CPXMIP_OPTIMAL
    assert round(milp_model.get_objective_value(),5) == 142.82857
    milp_model.cplex_finish()


def test_save_file():
    """This test verifies if the method save_solution_to_file is working properly.
    """
    targets_trace = Trace(n_targets=1, observation_period=1)
    targets_trace.trace_set = [[(50,50)]]
    graph = Graph(100, [10], (0,0,0), 1, 100, np.tan(np.pi/6))
    milp_model = MILPModel(n_available_drones=1,
                        observation_period=1,
                        time_step_delta=1,
                        targets_trace=targets_trace,
                        input_graph=graph,
                        alpha = 0,
                        beta = 0)
    
    model_shut_up(milp_model)
    milp_model.build_model()
    milp_model.solve_model()
    
    assert milp_model.cplex_model.solution.get_status() == CPXMIP_OPTIMAL
    milp_model.cplex_finish()
    

def test_movement_0():
    """In this example, we have two time steps, where the sensor moves from (25,50,0) to (75,50,0). So the drone is expected to go to move between time steps."""
    targets_trace = Trace(n_targets=1, observation_period=2)
    targets_trace.trace_set = [[(25,50),(75,50)]]
    graph = Graph(100, [10], (0,0,0), 1, 100, np.tan(np.pi/6))
    graph.deployment_positions = [(25,50,10),(75,50,10)]
    milp_model = MILPModel(n_available_drones=1,
                        observation_period=2,
                        time_step_delta=1,
                        targets_trace=targets_trace,
                        input_graph=graph,
                        alpha = 0,
                        beta = 0)

    model_shut_up(milp_model)

    milp_model.build_model()
    milp_model.solve_model()

    milp_model.cplex_model.solution.get_status() == CPXMIP_OPTIMAL
    round(milp_model.get_objective_value(),5) == 197.48087
    milp_model.cplex_finish()


def test_movement_1():
    """In this example, we have two time steps, where the sensor moves from (25,50,0) to (75,50,0). So the drone is expected to go to move between time steps. Unlike test_movement_0, we have only 60m of communication range. So this should be infeasible."""
    targets_trace = Trace(n_targets=1, observation_period=2)
    targets_trace.trace_set = [[(25,50),(75,50)]]
    graph = Graph(100, [10], (0,0,0), 1, 60, np.tan(np.pi/6))
    graph.deployment_positions = [(25,50,10),(75,50,10)]
    milp_model = MILPModel(n_available_drones=1,
                        observation_period=2,
                        time_step_delta=1,
                        targets_trace=targets_trace,
                        input_graph=graph,
                        alpha = 0,
                        beta = 0)

    model_shut_up(milp_model)

    milp_model.build_model()
    milp_model.solve_model()

    milp_model.cplex_model.solution.get_status() == CPXMIP_INFEASIBLE
    milp_model.cplex_finish()


def test_movement_2():
    """In this example, we have two time steps, where the sensor moves from (25,50,0) to (75,50,0). So the drone is expected to go to move between time steps. Unlike test_movement_0, we have only 60m of communication range but 2 drones available. So at the second time step two drones should be deployed for the flow to reach the sensor."""

    targets_trace = Trace(n_targets=1, observation_period=2)
    targets_trace.trace_set = [[(25,50),(75,50)]]
    graph = Graph(100, [10], (0,0,0), 1, 60, np.tan(np.pi/6))
    graph.deployment_positions = [(25,50,10),(75,50,10)]
    milp_model = MILPModel(n_available_drones=2,
                        observation_period=2,
                        time_step_delta=1,
                        targets_trace=targets_trace,
                        input_graph=graph,
                        alpha = 0,
                        beta = 0)

    model_shut_up(milp_model)

    milp_model.build_model()
    milp_model.solve_model()

    milp_model.cplex_model.solution.get_status() == CPXMIP_OPTIMAL
    round(milp_model.get_objective_value(),5) == 294.96174
    milp_model.cplex_finish()


def test_idle_drones_0():
    """"We repeat test_movement_0, but now we have 10 drones available. So we expect the same solution, but with 9 idle drones."""
    targets_trace = Trace(n_targets=1, observation_period=2)
    targets_trace.trace_set = [[(25,50),(75,50)]]
    graph = Graph(100, [10], (0,0,0), 1, 100, np.tan(np.pi/6))
    graph.deployment_positions = [(25,50,10),(75,50,10)]
    milp_model = MILPModel(n_available_drones=10,
                        observation_period=2,
                        time_step_delta=1,
                        targets_trace=targets_trace,
                        input_graph=graph,
                        alpha = 0,
                        beta = 0)

    model_shut_up(milp_model)
    milp_model.build_model()
    milp_model.solve_model()

    milp_model.cplex_model.solution.get_status() == CPXMIP_OPTIMAL
    round(milp_model.get_objective_value(),5) == 197.48087

    deployement = milp_model.get_drones_deployement()
    for t in range(2):
        for drone in range(10):
            assert len(deployement[t][drone]) == len(graph.base_station)
    milp_model.cplex_finish()


def test_uncoverable_sensor_0():
    """For this example, we have 4 sensors on the corners (0,0),(100,0),(0,100) and (100,100). There is only one deployement solution (50,50,10) for a single time step.
    
    This test verifies this scenario is infeasible.
    """
    targets_trace = Trace(n_targets = 4, observation_period=1)
    targets_trace.trace_set = [[(0,0),(100,0),(0,100),(100,100)]]
    graph = Graph(100, [10], (0,0,0), 1, 100, np.tan(np.pi/6))
    milp_model = MILPModel(n_available_drones=1,
                        observation_period=1,
                        time_step_delta=1,
                        targets_trace=targets_trace,
                        input_graph=graph,
                        alpha = 0,
                        beta = 0)
    
    model_shut_up(milp_model)
    milp_model.build_model()
    milp_model.solve_model()
    
    milp_model.cplex_model.solution.get_status() == CPXMIP_INFEASIBLE
    milp_model.cplex_finish()

def test_sensor_coverage_0():
    """For this example, we have 2 sensors for 2 time steps. The first one stays at (25,50) and the second one at (75,50). Even though both drone can communicate directly with the base station (comm_range = 100). Both must be deployed to cover the sensors.
    
    """
    targets_trace = Trace(n_targets=2, observation_period=2)
    targets_trace.trace_set = [[(25,50),(25,50)],[(75,50),(75,50)]]
    graph = Graph(100, [10], (0,0,0), 1, 100, np.tan(np.pi/6))
    graph.deployment_positions = [(25,50,10),(75,50,10)]
    milp_model = MILPModel(n_available_drones=2,
                        observation_period=2,
                        time_step_delta=1,
                        targets_trace=targets_trace,
                        input_graph=graph,
                        alpha = 0,
                        beta = 0)

    model_shut_up(milp_model)

    milp_model.build_model()
    milp_model.solve_model()

    milp_model.cplex_model.solution.get_status() == CPXMIP_OPTIMAL
    round(milp_model.get_objective_value(),5) == 294.96174
    milp_model.cplex_finish()

def test_sensor_coverage_1():
    """For this example, we have 2 sensors for 2 time steps. The first one stays at (25,50) and the second one at (75,50). But only one drone. So the solution is infeasible.
    """
    targets_trace = Trace(n_targets=2, observation_period=2)
    targets_trace.trace_set = [[(25,50),(25,50)],[(75,50),(75,50)]]
    graph = Graph(100, [10], (0,0,0), 1, 100, np.tan(np.pi/6))
    graph.deployment_positions = [(25,50,10),(75,50,10)]
    milp_model = MILPModel(n_available_drones=1,
                        observation_period=2,
                        time_step_delta=1,
                        targets_trace=targets_trace,
                        input_graph=graph,
                        alpha = 0,
                        beta = 0)

    model_shut_up(milp_model)

    milp_model.build_model()
    milp_model.solve_model()

    milp_model.cplex_model.solution.get_status() == CPXMIP_INFEASIBLE

    milp_model.cplex_finish()