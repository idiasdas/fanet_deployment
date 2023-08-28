import os
from fanet.targets_trace import TargetsTrace
from fanet.graph import Graph
from fanet.milp_model import MilpModel
from fanet.config import PARAMETERS, FILES_DIR, BASE_DIR, TESTS_OUTPUT_DIR

def run_milp_model(n_targets: int, n_drones: int, target_speed: float, instance: int, graph: Graph, alpha: float) -> float:
    """Runs the milp model for the given parameters and saves the solution in the experiment directory.
    If the solution already exists for an instance, it skips that instance."""
    solution_file = FILES_DIR+PARAMETERS["experiment_name"]+f"/milp_solution_p_{graph.n_positions_per_axis}_d_{n_drones}_nt_{n_targets}_t_{PARAMETERS['observation_period']}_v_{target_speed}_alpha_{alpha}_i_{instance}.txt"
    if os.path.isfile(solution_file):
        return 0
    trace_file = FILES_DIR+f"traces/trace_nt_{n_targets}_t_{PARAMETERS['observation_period']}_v_{target_speed}_i_{n}.txt"
    trace = TargetsTrace(load_file=trace_file)
    model = MilpModel(n_available_drones=2*n_drones,
                        observation_period=PARAMETERS["observation_period"],
                        time_step_delta=PARAMETERS["time_step_delta"],
                        targets_trace=trace,
                        input_graph=graph,
                        alpha=alpha,
                        beta = PARAMETERS["beta"])
    model.build_model()
    model.solve_model()
    solution = model.get_objective_value()
    model.save_solution(solution_file)
    model.cplex_finish()
    return solution

if __name__ == "__main__":
    """This script creates the experiment directory and runs the milp model for each parameter combination.
    It saves the solutions in the experiment directory.
    If the solution already exists for an instance, it skips that instance.
    """
    if os.path.isdir(FILES_DIR + PARAMETERS["experiment_name"]) == False:
        try:
            os.mkdir(FILES_DIR + PARAMETERS["experiment_name"])
        except OSError:
            print ("Creation of the directory %s failed" % (FILES_DIR + PARAMETERS["experiment_name"]))
            exit(1)

    file_parameters = open(FILES_DIR + PARAMETERS["experiment_name"] + "/parameters.txt", "w")
    file_parameters.write(str(PARAMETERS))
    file_parameters.close()

    for n_positions in PARAMETERS["n_positions"]:
        graph = Graph(size_A = PARAMETERS["area_size"],
                        heights = PARAMETERS["heights"],
                        base_station = PARAMETERS["base_station"],
                        n_positions_per_axis = n_positions,
                        communication_range = PARAMETERS["comm_range"],
                        coverage_angle = PARAMETERS["coverage_angle"])
        for n_targets in PARAMETERS["n_targets"]:
            for target_speed in PARAMETERS["targets_speed"]:
                for n_drones in PARAMETERS["n_drones"]:
                    for alpha in PARAMETERS["alpha"]:
                        for n in range(PARAMETERS["n_instances"]):
                            run_milp_model(n_targets, n_drones, target_speed, n, graph, alpha)
