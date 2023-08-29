from fanet.setup.cplex_constants import *
from fanet.milp_model import MilpModel
from fanet.graph import Graph
from fanet.targets_trace import TargetsTrace
import numpy as np
import os
this_dirctory = os.path.dirname(__file__)

def example_basic_0() -> list:
    """For this example, we have one sensor in the middle of the area (50,50,0) and only one deployement solution (50,50,10) for a single time step.

    Returns:
        list: [targets_trace, graph, milp_model]
    """
    targets_trace = TargetsTrace(n_targets=1, observation_period=1)
    targets_trace.trace_set = [[(50, 50)]]
    graph = Graph(100, [10], (0, 0, 0), 1, 100, np.tan(np.pi/6))
    milp_model = MilpModel(n_available_drones=1,
                           observation_period=1,
                           time_step_delta=1,
                           targets_trace=targets_trace,
                           input_graph=graph,
                           alpha=0,
                           beta=0.08095)
    return [targets_trace, graph, milp_model]

def example_basic_1() -> list:
    """In this example, we have two time steps, where the sensor remains in (50,50,0). So the drone is expected to go to (50,50,10) for both time steps.

    Returns:
        list: [targets_trace, graph, milp_model]
    """
    targets_trace = TargetsTrace(n_targets=1, observation_period=2)
    targets_trace.trace_set = [[(50, 50), (50, 50)]]
    graph = Graph(100, [10], (0, 0, 0), 1, 100, np.tan(np.pi/6))
    milp_model = MilpModel(n_available_drones=1,
                           observation_period=2,
                           time_step_delta=1,
                           targets_trace=targets_trace,
                           input_graph=graph,
                           alpha=0,
                           beta=0.08095)
    return [targets_trace, graph, milp_model]

def example_movement_0() -> list:
    """In this example, we have two time steps, where the sensor moves from (25,50,0) to (75,50,0). So the drone is expected to go to move between time steps."""
    targets_trace = TargetsTrace(n_targets=1, observation_period=2)
    targets_trace.trace_set = [[(25, 50), (75, 50)]]
    graph = Graph(100, [10], (0, 0, 0), 1, 100, np.tan(np.pi/6))
    graph.deployment_positions = [(25, 50, 10), (75, 50, 10)]
    milp_model = MilpModel(n_available_drones=1,
                           observation_period=2,
                           time_step_delta=1,
                           targets_trace=targets_trace,
                           input_graph=graph,
                           alpha=0,
                           beta=0.08095)
    return [targets_trace, graph, milp_model]

def example_movement_1() -> list:
    """In this example, we have two time steps, where the sensor moves from (25,50,0) to (75,50,0). But only one drone is available with 60m communication range. So the solution is infeasible."""
    targets_trace = TargetsTrace(n_targets=1, observation_period=2)
    targets_trace.trace_set = [[(25, 50), (75, 50)]]
    graph = Graph(100, [10], (0, 0, 0), 1, 60, np.tan(np.pi/6))
    graph.deployment_positions = [(25, 50, 10), (75, 50, 10)]
    milp_model = MilpModel(n_available_drones=1,
                           observation_period=2,
                           time_step_delta=1,
                           targets_trace=targets_trace,
                           input_graph=graph,
                           alpha=0,
                           beta=0.08095)
    return [targets_trace, graph, milp_model]

def example_movement_2() -> list:
    """In this example, we have two time steps, where the sensor moves from (25,50,0) to (75,50,0). So the drone is expected to go to move between time steps. Unlike test_movement_0, we have only 60m of communication range but 2 drones available."""
    targets_trace = TargetsTrace(n_targets=1, observation_period=2)
    targets_trace.trace_set = [[(25, 50), (75, 50)]]
    graph = Graph(100, [10], (0, 0, 0), 1, 60, np.tan(np.pi/6))
    graph.deployment_positions = [(25, 50, 10), (75, 50, 10)]
    milp_model = MilpModel(n_available_drones=2,
                           observation_period=2,
                           time_step_delta=1,
                           targets_trace=targets_trace,
                           input_graph=graph,
                           alpha=0,
                           beta=0.08095)
    return [targets_trace, graph, milp_model]

def example_sensor_coverage_0() -> list:
    """Two drones available. Two deployment positions: (25,50,10) and (75,50,10). Two time steps. Two sensors: (25,50) and (75,50). Both drones must be deployed to cover the sensors."""
    targets_trace = TargetsTrace(n_targets=2, observation_period=2)
    targets_trace.trace_set = [[(25, 50), (25, 50)], [(75, 50), (75, 50)]]
    graph = Graph(100, [10], (0, 0, 0), 1, 100, np.tan(np.pi/6))
    graph.deployment_positions = [(25, 50, 10), (75, 50, 10)]
    milp_model = MilpModel(n_available_drones=2,
                           observation_period=2,
                           time_step_delta=1,
                           targets_trace=targets_trace,
                           input_graph=graph,
                           alpha=0,
                           beta=0.08095)
    return [targets_trace, graph, milp_model]

def test_basic_0() -> None:
    """Verifies if the model properly solves a problem with 1 time step. Where the drone is forced to follows the trajectory: base_station -> (50,50,10) -> base_station.
    """
    targets_trace, graph, milp_model = example_basic_0()
    milp_model.model_shut_up()
    milp_model.build_model()
    milp_model.solve_model()
    assert milp_model.cplex_model.solution.get_status() == OPTIMAL_SOLUTION
    assert round(milp_model.get_objective_value(), 5) == 142.82857
    milp_model.cplex_finish()


def test_basic_1() -> None:
    """Tests if the model properly solves a problem with 2 time steps."""
    targets_trace, graph, milp_model = example_basic_1()
    milp_model.model_shut_up()
    milp_model.build_model()
    milp_model.solve_model()
    assert milp_model.cplex_model.solution.get_status() == OPTIMAL_SOLUTION
    assert round(milp_model.get_objective_value(), 5) == 142.82857
    milp_model.cplex_finish()


def test_save_file() -> None:
    """This test verifies if the method save_solution_to_file is working properly.
    """
    targets_trace, graph, milp_model = example_basic_0()
    milp_model.model_shut_up()
    milp_model.build_model()
    milp_model.solve_model()


def test_movement_0() -> None:
    """Tests if milp models properly solves example_movement_0 where the drones can move between time steps."""

    targets_trace, graph, milp_model = example_movement_0()
    milp_model.model_shut_up()
    milp_model.build_model()
    milp_model.solve_model()
    assert milp_model.cplex_model.solution.get_status() == OPTIMAL_SOLUTION
    assert round(milp_model.get_objective_value(), 5) == 197.48087
    milp_model.cplex_finish()


def test_movement_1() -> None:
    """The drone is expected to go to move between time steps. Unlike test_movement_0, we have only 60m of communication range. So this should be infeasible."""
    targets_trace, graph, milp_model = example_movement_1()
    milp_model.model_shut_up()
    milp_model.build_model()
    milp_model.solve_model()
    assert milp_model.cplex_model.solution.get_status() == INFEASIBLE_SOLUTION
    assert milp_model.get_objective_value() == -1
    milp_model.cplex_finish()


def test_movement_2() -> None:
    """Tests if milp_model properly solves example_movement_2."""
    targets_trace, graph, milp_model = example_movement_2()
    milp_model.model_shut_up()
    milp_model.build_model()
    milp_model.solve_model()
    assert milp_model.cplex_model.solution.get_status() == OPTIMAL_SOLUTION
    assert round(milp_model.get_objective_value(), 5) == 294.96174
    milp_model.cplex_finish()


def test_idle_drones_0() -> None:
    """"We repeat test_movement_0, but now we have 10 drones available. So we expect the same solution, but with 9 idle drones."""
    targets_trace, graph, milp_model = example_movement_0()
    milp_model.n_available_drones = 10

    milp_model.model_shut_up()
    milp_model.build_model()
    milp_model.solve_model()

    assert milp_model.cplex_model.solution.get_status() == OPTIMAL_SOLUTION
    assert round(milp_model.get_objective_value(), 5) == 197.48087

    deployement = milp_model.get_drones_deployement()
    drones_at_base_station = 0
    for t in range(2):
        for drone in range(10):
            if deployement[t][drone] == graph.base_station:
                drones_at_base_station += 1
        assert drones_at_base_station == 9
        drones_at_base_station = 0
    milp_model.cplex_finish()


def test_uncoverable_sensor_0() -> None:
    """For this example, we have 4 sensors on the corners (0,0),(100,0),(0,100) and (100,100). There is only one deployement solution (50,50,10) for a single time step.

    This test verifies this scenario is infeasible.
    """
    targets_trace = TargetsTrace(n_targets=4, observation_period=1)
    targets_trace.trace_set = [[(0, 0), (100, 0), (0, 100), (100, 100)]]
    graph = Graph(100, [10], (0, 0, 0), 1, 100, np.tan(np.pi/6))
    milp_model = MilpModel(n_available_drones=1,
                           observation_period=1,
                           time_step_delta=1,
                           targets_trace=targets_trace,
                           input_graph=graph,
                           alpha=0,
                           beta=0.08095)

    milp_model.model_shut_up()
    milp_model.build_model()
    milp_model.solve_model()

    assert milp_model.cplex_model.solution.get_status() == INFEASIBLE_SOLUTION
    assert milp_model.get_objective_value() == -1
    milp_model.cplex_finish()


def test_sensor_coverage_0() -> None:
    """For this example, we have 2 sensors for 2 time steps. The first one stays at (25,50) and the second one at (75,50). Even though both drone can communicate directly with the base station (comm_range = 100). Both must be deployed to cover the sensors.

    """
    targets_trace, graph, milp_model = example_sensor_coverage_0()
    milp_model.model_shut_up()
    milp_model.build_model()
    milp_model.solve_model()
    assert milp_model.cplex_model.solution.get_status() == OPTIMAL_SOLUTION
    assert round(milp_model.get_objective_value(), 5) == 294.96174
    milp_model.cplex_finish()


def test_sensor_coverage_1() -> None:
    """For this example, we have 2 sensors for 2 time steps. The first one stays at (25,50) and the second one at (75,50). But only one drone. So the solution is infeasible.
    """
    targets_trace, graph, milp_model = example_sensor_coverage_0()
    milp_model.n_available_drones = 1
    milp_model.model_shut_up()
    milp_model.build_model()
    milp_model.solve_model()
    assert milp_model.cplex_model.solution.get_status() == INFEASIBLE_SOLUTION
    assert milp_model.get_objective_value() == -1
    milp_model.cplex_finish()


def test_variables_types() -> None:
    """Tests if variables defined to cplex have the correct type.
    """
    targets_trace, graph, milp_model = example_sensor_coverage_0()
    milp_model.model_shut_up()
    milp_model.build_model()
    # Testing the variables z_t_p for all t \in T and p \in P \cup {base_station}
    for t in range(milp_model.observation_period):
        for p in graph.deployment_positions:
            assert milp_model.get_variable(milp_model.var_z_t_p(t, p))[
                "type"] == BINARY_VARIABLE

    # Testing the variables z_t_drone_p for all t \in T, drone \in n_available_drones and p \in P \cup {base_station}
    for t in range(milp_model.observation_period):
        for drone in range(milp_model.n_available_drones):
            assert milp_model.get_variable(milp_model.var_z_t_drone_p(
                t, drone, graph.base_station))["type"] == BINARY_VARIABLE
            for p in graph.deployment_positions:
                assert milp_model.get_variable(milp_model.var_z_t_drone_p(t, drone, p))[
                    "type"] == BINARY_VARIABLE

    # Testing the flow variables f_t_p_q for all t \in T, p,q \in P and p \neq q
    for t in range(milp_model.observation_period):
        for p in graph.deployment_positions + [graph.base_station]:
            for q in graph.get_positions_in_comm_range(p):
                assert milp_model.get_variable(milp_model.var_f_t_p_q(t, p, q))[
                    "type"] == CONTINUOUS_VARIABLE

    # Testing the flow variables f_t_p_q for all t \in T, sensor_position \in trace_set and delpoyment_position \in P that covers the sensor_position
    for t in range(milp_model.observation_period):
        for sensor_trace in targets_trace.trace_set:
            sensor_position = sensor_trace[t]
            for deployment_position in graph.get_target_coverage(sensor_position):
                assert milp_model.get_variable(milp_model.var_f_t_p_q(
                    t, deployment_position, sensor_position))["type"] == CONTINUOUS_VARIABLE

    # Testing the variables z_t_drone_p_q for all t \in T, drone \in n_available_drones, p,q \in P and p \neq q
    if milp_model.observation_period > 1:  # Otherwise there are no drone movements within the observation period
        for t in range(milp_model.observation_period):
            for drone in range(milp_model.n_available_drones):
                for p in graph.deployment_positions + [graph.base_station]:
                    for q in graph.deployment_positions + [graph.base_station]:
                        assert milp_model.get_variable(milp_model.var_z_t_drone_p_q(t, drone, p, q))[
                            "type"] == BINARY_VARIABLE

def test_solution_values_0():
    """Solves the example_basic_0 with alpha = 0.5 and verifies if the objective value and the distance and energy costs agree.
    """
    targets_traces, graph, milp_model = example_basic_0()
    milp_model.alpha = 0.5
    milp_model.model_shut_up()
    milp_model.build_model()
    milp_model.solve_model()
    assert milp_model.cplex_model.solution.get_status() == OPTIMAL_SOLUTION
    assert round(milp_model.get_solution_distance(), 5) == 142.82857
    milp_objective_value = round(milp_model.get_objective_value(),5)
    milp_solution_distance = milp_model.get_solution_distance()
    milp_solution_energy = milp_model.get_solution_energy()
    print(f"milp_objective_value: {milp_objective_value}")
    print(f"milp_solution_distance: {milp_solution_distance}")
    print(f"milp_solution_energy: {milp_solution_energy}")
    assert milp_objective_value == round(0.5*milp_solution_distance + 0.5*0.08095*milp_solution_energy,5)
    milp_model.cplex_finish()

def test_solution_values_1():
    """Solves the example_movement_0 with alpha = 0.5 and verifies if the objective value and the distance and energy costs agree.
    """
    targets_traces, graph, milp_model = example_movement_0()
    milp_model.alpha = 0.5
    milp_model.model_shut_up()
    milp_model.build_model()
    milp_model.solve_model()
    assert milp_model.cplex_model.solution.get_status() == OPTIMAL_SOLUTION
    milp_objective_value = round(milp_model.get_objective_value(),5)
    milp_solution_distance = milp_model.get_solution_distance()
    milp_solution_energy = milp_model.get_solution_energy()
    print(f"milp_objective_value: {milp_objective_value}")
    print(f"milp_solution_distance: {milp_solution_distance}")
    print(f"milp_solution_energy: {milp_solution_energy}")
    assert milp_objective_value == round(0.5*milp_solution_distance + 0.5*0.08095*milp_solution_energy,5)
    milp_model.cplex_finish()

def test_solution_values_2():
    """Solves the example_movement_2 with alpha = 0.5 and verifies if the objective value and the distance and energy costs agree.
    """
    targets_traces, graph, milp_model = example_movement_2()
    milp_model.alpha = 0.5
    milp_model.model_shut_up()
    milp_model.build_model()
    milp_model.solve_model()
    assert milp_model.cplex_model.solution.get_status() == OPTIMAL_SOLUTION
    milp_objective_value = round(milp_model.get_objective_value(),5)
    milp_solution_distance = milp_model.get_solution_distance()
    milp_solution_energy = milp_model.get_solution_energy()
    print(f"milp_objective_value: {milp_objective_value}")
    print(f"milp_solution_distance: {milp_solution_distance}")
    print(f"milp_solution_energy: {milp_solution_energy}")
    assert milp_objective_value == round(0.5*milp_solution_distance + 0.5*0.08095*milp_solution_energy,5)
    milp_model.cplex_finish()
