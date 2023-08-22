import os
import sys
import unittest
this_dirctory = os.path.dirname(__file__)
sys.path.append( this_dirctory + '/../src/')  # Now can import modules in src
import numpy as np
from targets_trace import Trace
from graph import Graph
from milp_model import MILPModel

class TestMilpModel(unittest.TestCase):
    """This class tests the MILPModel class. To execute the tests, run the following command:
        python -m unittest test_milp.py 
    """
    #cplex constants

    CPXMIP_OPTIMAL = 101
    CPXMIP_INFEASIBLE = 103

    def model_shut_up(self, milp_model):
        milp_model.cplex_model.set_log_stream(None)
        milp_model.cplex_model.set_error_stream(None)
        milp_model.cplex_model.set_warning_stream(None)
        milp_model.cplex_model.set_results_stream(None)

    def print_model(self, milp_model):
        print("---------------------------------")
        for var in milp_model.variables:
            print(var)
        print("---------------------------------")
        for constr in milp_model.constraints:
            print(constr)
        print("---------------------------------")
        print(milp_model.get_objective_function())
        print("---------------------------------")
        print(milp_model.targets_trace.trace_set)

    def test_basic_0(self):
        """For this example, we have one sensor in the middle of the area (50,50,0) and only one deployement solution (50,50,10) for a single time step.
        
        This test verifies if the model is built properly, solved and if the solution is correct.
        
        The expected solution is: 142.82857. This is the distance cost for the drone to leave the base station, go to (50,50,10) and go back to the base station.
        """
        targets_trace = Trace(n_targets=1, observation_period=1)
        targets_trace.trace_set = [[(50,50)]]
        graph = Graph(100, [10], (0,0,0), 1, 100, np.tan(np.pi/6))
        self.milp_model = MILPModel(n_available_drones=1,
                            observation_period=1,
                            time_step_delta=1,
                            targets_trace=targets_trace,
                            input_graph=graph,
                            alpha = 0,
                            beta = 0)
        
        self.model_shut_up(self.milp_model)
        self.milp_model.build_model()
        self.milp_model.cplex_model.solve()
        
        self.assertEqual(self.milp_model.cplex_model.solution.get_status(), self.CPXMIP_OPTIMAL)
        self.assertEqual(round(self.milp_model.cplex_model.solution.get_objective_value(),5), 142.82857)

    def test_basic_1(self):
        """In this example, we have two time steps, where the sensor remains in (50,50,0). So the drone is expected to go to (50,50,10) for both time steps."""
        targets_trace = Trace(n_targets=1, observation_period=2)
        targets_trace.trace_set = [[(50,50),(50,50)]]
        graph = Graph(100, [10], (0,0,0), 1, 100, np.tan(np.pi/6))
        self.milp_model = MILPModel(n_available_drones=1,
                            observation_period=2,
                            time_step_delta=1,
                            targets_trace=targets_trace,
                            input_graph=graph,
                            alpha = 0,
                            beta = 0)
        
        self.model_shut_up(self.milp_model)
        self.milp_model.build_model()
        self.milp_model.cplex_model.solve()

        self.assertEqual(self.milp_model.cplex_model.solution.get_status(), self.CPXMIP_OPTIMAL)
        self.assertEqual(round(self.milp_model.cplex_model.solution.get_objective_value(),5), 142.82857)

    def test_movement_0(self):
        """In this example, we have two time steps, where the sensor moves from (25,50,0) to (75,50,0). So the drone is expected to go to move between time steps."""
        targets_trace = Trace(n_targets=1, observation_period=2)
        targets_trace.trace_set = [[(25,50),(75,50)]]
        graph = Graph(100, [10], (0,0,0), 1, 100, np.tan(np.pi/6))
        graph.deployment_positions = [(25,50,10),(75,50,10)]
        self.milp_model = MILPModel(n_available_drones=1,
                            observation_period=2,
                            time_step_delta=1,
                            targets_trace=targets_trace,
                            input_graph=graph,
                            alpha = 0,
                            beta = 0)

        self.model_shut_up(self.milp_model)

        self.milp_model.build_model()
        self.milp_model.cplex_model.solve()

        self.assertEqual(self.milp_model.cplex_model.solution.get_status(), self.CPXMIP_OPTIMAL)
        self.assertEqual(round(self.milp_model.cplex_model.solution.get_objective_value(),5), 197.48087)

    def test_movement_1(self):
        """In this example, we have two time steps, where the sensor moves from (25,50,0) to (75,50,0). So the drone is expected to go to move between time steps. Unlike test_movement_0, we have only 60m of communication range. So this should be infeasible."""
        targets_trace = Trace(n_targets=1, observation_period=2)
        targets_trace.trace_set = [[(25,50),(75,50)]]
        graph = Graph(100, [10], (0,0,0), 1, 60, np.tan(np.pi/6))
        graph.deployment_positions = [(25,50,10),(75,50,10)]
        self.milp_model = MILPModel(n_available_drones=1,
                            observation_period=2,
                            time_step_delta=1,
                            targets_trace=targets_trace,
                            input_graph=graph,
                            alpha = 0,
                            beta = 0)

        self.model_shut_up(self.milp_model)

        self.milp_model.build_model()
        self.milp_model.cplex_model.solve()

        self.assertEqual(self.milp_model.cplex_model.solution.get_status(), self.CPXMIP_INFEASIBLE)

    def test_movement_2(self):
        """In this example, we have two time steps, where the sensor moves from (25,50,0) to (75,50,0). So the drone is expected to go to move between time steps. Unlike test_movement_0, we have only 60m of communication range but 2 drones available. So at the second time step two drones should be deployed for the flow to reach the sensor."""

        targets_trace = Trace(n_targets=1, observation_period=2)
        targets_trace.trace_set = [[(25,50),(75,50)]]
        graph = Graph(100, [10], (0,0,0), 1, 60, np.tan(np.pi/6))
        graph.deployment_positions = [(25,50,10),(75,50,10)]
        self.milp_model = MILPModel(n_available_drones=2,
                            observation_period=2,
                            time_step_delta=1,
                            targets_trace=targets_trace,
                            input_graph=graph,
                            alpha = 0,
                            beta = 0)

        self.model_shut_up(self.milp_model)

        self.milp_model.build_model()
        self.milp_model.cplex_model.solve()

        self.assertEqual(self.milp_model.cplex_model.solution.get_status(), self.CPXMIP_OPTIMAL)
        self.assertEqual(round(self.milp_model.cplex_model.solution.get_objective_value(),5), 294.96174)


        # deployement = self.milp_model.get_drones_deployement()

        # for t in range(self.milp_model.observation_period):
        #     for drone, position in deployement[t]:
        #         print(f"\t {t} \t {drone} \t {position}")


    def test_idle_drones_0(self):
        """"We repeat test_movement_0, but now we have 10 drones available. So we expect the same solution, but with 9 idle drones."""
        targets_trace = Trace(n_targets=1, observation_period=2)
        targets_trace.trace_set = [[(25,50),(75,50)]]
        graph = Graph(100, [10], (0,0,0), 1, 100, np.tan(np.pi/6))
        graph.deployment_positions = [(25,50,10),(75,50,10)]
        self.milp_model = MILPModel(n_available_drones=10,
                            observation_period=2,
                            time_step_delta=1,
                            targets_trace=targets_trace,
                            input_graph=graph,
                            alpha = 0,
                            beta = 0)

        self.model_shut_up(self.milp_model)
        self.milp_model.build_model()
        self.milp_model.cplex_model.solve()

        self.assertEqual(self.milp_model.cplex_model.solution.get_status(), self.CPXMIP_OPTIMAL)
        self.assertEqual(round(self.milp_model.cplex_model.solution.get_objective_value(),5), 197.48087)

    def test_uncoverable_sensor_0(self):
        """For this example, we have 4 sensors on the corners (0,0),(100,0),(0,100) and (100,100). There is only one deployement solution (50,50,10) for a single time step.
        
        This test verifies this scenario is infeasible.
        """
        targets_trace = Trace(n_targets = 4, observation_period=1)
        targets_trace.trace_set = [[(0,0),(100,0),(0,100),(100,100)]]
        graph = Graph(100, [10], (0,0,0), 1, 100, np.tan(np.pi/6))
        self.milp_model = MILPModel(n_available_drones=1,
                            observation_period=1,
                            time_step_delta=1,
                            targets_trace=targets_trace,
                            input_graph=graph,
                            alpha = 0,
                            beta = 0)
        
        self.model_shut_up(self.milp_model)
        self.milp_model.build_model()
        self.milp_model.cplex_model.solve()
        
        self.assertEqual(self.milp_model.cplex_model.solution.get_status(), self.CPXMIP_INFEASIBLE)

    def test_sensor_coverage_0(self):
        """For this example, we have 2 sensors for 2 time steps. The first one stays at (25,50) and the second one at (75,50). Even though both drone can communicate directly with the base station (comm_range = 100). Both must be deployed to cover the sensors.
        
        """
        targets_trace = Trace(n_targets=2, observation_period=2)
        targets_trace.trace_set = [[(25,50),(25,50)],[(75,50),(75,50)]]
        graph = Graph(100, [10], (0,0,0), 1, 100, np.tan(np.pi/6))
        graph.deployment_positions = [(25,50,10),(75,50,10)]
        self.milp_model = MILPModel(n_available_drones=2,
                            observation_period=2,
                            time_step_delta=1,
                            targets_trace=targets_trace,
                            input_graph=graph,
                            alpha = 0,
                            beta = 0)

        self.model_shut_up(self.milp_model)

        self.milp_model.build_model()
        self.milp_model.cplex_model.solve()

        self.assertEqual(self.milp_model.cplex_model.solution.get_status(), self.CPXMIP_OPTIMAL)
        self.assertEqual(round(self.milp_model.cplex_model.solution.get_objective_value(),5), 294.96174)

    def test_sensor_coverage_1(self):
        """For this example, we have 2 sensors for 2 time steps. The first one stays at (25,50) and the second one at (75,50). But only one drone. So the solution is infeasible.
        """
        targets_trace = Trace(n_targets=2, observation_period=2)
        targets_trace.trace_set = [[(25,50),(25,50)],[(75,50),(75,50)]]
        graph = Graph(100, [10], (0,0,0), 1, 100, np.tan(np.pi/6))
        graph.deployment_positions = [(25,50,10),(75,50,10)]
        self.milp_model = MILPModel(n_available_drones=1,
                            observation_period=2,
                            time_step_delta=1,
                            targets_trace=targets_trace,
                            input_graph=graph,
                            alpha = 0,
                            beta = 0)

        self.model_shut_up(self.milp_model)

        self.milp_model.build_model()
        self.milp_model.cplex_model.solve()

        self.assertEqual(self.milp_model.cplex_model.solution.get_status(), self.CPXMIP_INFEASIBLE)

if __name__ == '__main__':
    unittest.main()