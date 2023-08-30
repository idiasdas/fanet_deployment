from typing import Optional
from fanet.graph import Graph
from fanet.targets_trace import TargetsTrace
from fanet.energy_model import energy
from fanet.linear_expression import LinearExpression
from fanet.setup.cplex_constants import *
from fanet.setup.config import PARAMETERS
import cplex

class MilpModel:
    def __init__(self, n_available_drones: int, observation_period: int, time_step_delta: float, targets_trace: TargetsTrace, input_graph: Graph, alpha: float, beta: float, model_name: Optional[str] = "MILP_Model") -> None:
        """Builds the linear program to obtain the optimal deployment of drones to cover all targets at all time steps.

        Args:
            n_available_drones: Number of drones available.
            observation_period: Amount of time steps.
            time_step_delta: Amount of seconds between time steps.
            targets_trace: The trajectories of the targets.
            input_graph: The topology of the problem with the set of deployment positions and targets coordinates at each time step.
            alpha: Weight of objective function metrics.
            beta: Objective function normalization parameter.
            model_name: Name of the cplex model. Defaults to "MILP_Model".
        """

        self.n_available_drones = n_available_drones
        self.observation_period = observation_period
        self.time_step_delta = time_step_delta
        self.targets_trace = targets_trace
        self.input_graph = input_graph
        self.alpha = alpha
        self.beta = beta

        self.cplex_model = cplex.Cplex()
        self.cplex_model.set_problem_name(model_name)
        self.variables = []
        self.constraints = []

    def define_variable(self, var_name: str, var_lb: float, var_up: float, var_type: str) -> None:
        """Defines a variable and saves it in self.variables list. This function does not add the variables to the cplex model. Using these lists to add variables and constraints in batches is faster than adding them one by one to cplex.

        Args:
            var_name: Name of the variable.
            var_lb: Lower bound of the variable.
            var_up: Upper bound of the variable.
            var_type: Type of the variable. Use the constants defined in cplex_constants.py.
        """
        self.variables.append({"name": var_name, "lb": var_lb, "ub": var_up, "type": var_type})

    def get_variable(self, var_name: str) -> dict:
        """Returns the variable with the given name from the list self.variables.

        Args:
            var_name: Name of the variable.

        Returns:
            The variable with the given name or None if the variable is not found.
        """
        matching_vars = [var for var in self.variables if var["name"] == var_name]
        return matching_vars[0] if matching_vars else None

    def var_z_t_p(self, time_step: int, position: tuple) -> str:
        """Returns the name of the variable z_t_p. This is a binary variable for p \in P and t \in T that says if a drone is deployed at position p at time step t. For the base station this variable is an integer since the base station can have many drones in it simultaneously. This corresnponds to the variable z^t_p in the papers.

        Args:
            time_step: Time step.
            position: Position coordinates.
        Returns:
            The name of the variable z_t_p.
        """
        return f"z_t_{time_step}_p_{position}".replace(" ", "")

    def var_z_t_drone_p(self, time_step: int, drone: int, position: tuple) -> str:
        """Returns the name of the variable z_t_drone_p. This is a binary variable for p \in P and t \in T and drone \in n_available_drones that says if drone is deployed at position p at time step t. This corresnponds to the variable z^t_{pu} in the papers.

        Args:
            time_step: Time step.
            drone: Drone index.
            position: Position coordinates.

        Returns:
            The name of the variable z_t_drone_p.
        """
        return f"z_t_{time_step}_drone_{drone}_p_{position}".replace(" ", "")

    def var_f_t_p_q(self, time_step: int, position_p: tuple, position_q: tuple) -> str:
        """Returns the name of the variable f_t_p_q. This is a continuous variable for p, q \in P and t \in T that says how much flow is sent from position p to position q at time step t. This corresnponds to the variable f^t_{pq} in the papers.

        Args:
            time_step: Time step.
            position_p: Position coordinates.
            position_q: Position coordinates.

        Returns:
            The name of the variable f_t_p_q.
        """
        return f"f_t_{time_step}_p_{position_p}_q_{position_q}".replace(" ", "")

    def var_z_t_drone_p_q(self, time_step: int, drone:int, position_p: tuple, position_q: tuple) -> str:
        """Returns the name of the variable z_t_drone_p_q. This is a binary variable for p, q \in P and t \in T and drone \in n_available_drones that says if drone is deployed at position p at time step t and moves to position q at time step t+1. This corresnponds to the variable z^t_{upq} in the papers.

        Args:
            time_step (int): The given time step
            drone (int): The drone id
            position_p (tuple): The coordinates of position p at time step t
            position_q (tuple): The coordinates of position q at time step t+1

        Returns:
            str: The name of the variable z_t_drone_p_q.
        """
        return f"z_t_{time_step}_drone_{drone}_p_{position_p}_q_{position_q}".replace(" ", "")

    def define_all_variables(self) -> None:
        """Defines all the variables of the linear program. Uses the function define_variable to save the information of the variables in the corresponding lists. Later the variables must be added to the cplex model."""
        # Defining the variables z_t_p for all t \in T and p \in P \cup {base_station}
        for t in range(self.observation_period):
            self.define_variable(self.var_z_t_p(t, self.input_graph.base_station), 0, self.n_available_drones, INTEGER_VARIABLE)
            for p in self.input_graph.deployment_positions:
                self.define_variable(self.var_z_t_p(t, p), 0, 1, BINARY_VARIABLE)

        # Defining the variables z_t_drone_p for all t \in T, drone \in n_available_drones and p \in P \cup {base_station}
        for t in range(self.observation_period):
            for drone in range(self.n_available_drones):
                self.define_variable(self.var_z_t_drone_p(t, drone, self.input_graph.base_station), 0, 1, BINARY_VARIABLE)
                for p in self.input_graph.deployment_positions:
                    self.define_variable(self.var_z_t_drone_p(t, drone, p), 0, 1, BINARY_VARIABLE)

        # Defining the flow variables f_t_p_q for all t \in T, p, q \in P and p \neq q
        for t in range(self.observation_period):
            for p in self.input_graph.deployment_positions + [self.input_graph.base_station]:
                for q in self.input_graph.get_positions_in_comm_range(p):
                    self.define_variable(self.var_f_t_p_q(t, p, q), 0, len(self.targets_trace.trace_set), CONTINUOUS_VARIABLE)

        # Defining the flow variables f_t_p_q for all t \in T, sensor_position \in trace_set and delpoyment_position \in P that covers the sensor_position
        for t in range(self.observation_period):
            for sensor_trace in self.targets_trace.trace_set:
                sensor_position = sensor_trace[t]
                for deployment_position in self.input_graph.get_target_coverage(sensor_position):
                    self.define_variable(self.var_f_t_p_q(t, deployment_position, sensor_position), 0, len(self.targets_trace.trace_set), CONTINUOUS_VARIABLE)

        # Defining the variables z_t_drone_p_q for all t \in T, drone \in n_available_drones, p, q \in P and p \neq q
        if self.observation_period > 1: # Otherwise there are no drone movements within the observation period
            for t in range(self.observation_period):
                for drone in range(self.n_available_drones):
                    for p in self.input_graph.deployment_positions + [self.input_graph.base_station]:
                        for q in self.input_graph.deployment_positions + [self.input_graph.base_station]:
                            self.define_variable(self.var_z_t_drone_p_q(t, drone, p, q), 0, 1, BINARY_VARIABLE)

    def define_constraint(self, constr_name: str, constr_linear_expr: list, constr_sense:int, constr_rhs:float) -> None:
        """Defines a constraint and saves its information in the corresponding lists.

        This function does not add the constraints to the CPLEX model. Using these lists for batch additions is faster than adding constraints individually.

        Args:
            constr_name: Name of the constraint.
            constr_linear_expr: Linear expression of the constraint. A list with two lists: [[variable_names], [coefficients]].
            constr_sense: Sense of the constraint. Use the constants GREATER_EQUAL, EQUAL or LESS_EQUAL defined in this class.
            constr_rhs: Right hand side of the constraint.
        """
        self.constraints.append({"name": constr_name, "linear_expr": constr_linear_expr, "sense": constr_sense, "rhs": constr_rhs})

    def define_flow_constraints(self) -> None:
        """Defines the flow constraints for all time steps."""
        for t in range(self.observation_period):
            for p in self.input_graph.deployment_positions:
                constr = LinearExpression()
                constr_name = f"flow_conservation_t_{t}_p_{p}"

                if p in self.input_graph.get_positions_in_comm_range(self.input_graph.base_station):
                    # (-) Flow that enters p from base station
                    constr.add_term(-1, self.var_f_t_p_q(t, self.input_graph.base_station, p))
                for q in self.input_graph.get_positions_in_comm_range(p):
                    # (+) Flow that leaves p to q
                    constr.add_term(1, self.var_f_t_p_q(t, p, q))
                    # (-) Flow that enters p from q
                    constr.add_term(-1, self.var_f_t_p_q(t, q, p))

                for sensor in self.input_graph.get_position_coverage(p, self.targets_trace.get_targets_positions_at_time(t)):
                    # (+) Flow that leaves p to sensor
                    constr.add_term(1, self.var_f_t_p_q(t, p, sensor))

                # For any position p, the flow that enters p must be equal to the flow that leaves p at any time step
                self.define_constraint(constr_name, constr.get_expression(), EQUAL, 0)

            for sensor in self.targets_trace.get_targets_positions_at_time(t):
                constr = LinearExpression()
                constr_name = f"flow_conservation_t_{t}_sensor_{sensor}"
                for p in self.input_graph.get_target_coverage(sensor):
                    # (+) Flow that leaves p to sensor
                    constr.add_term(1, self.var_f_t_p_q(t, p, sensor))

                # At any time step, a sensor must receive at least one flow
                self.define_constraint(constr_name, constr.get_expression(), GREATER_EQUAL, 1)

    def define_drone_flow_constraints(self) -> None:
        """This constraint ensures a flow only exists if a drone is deployed at the source position."""
        for t in range(self.observation_period):
            for p in self.input_graph.deployment_positions:
                if p in self.input_graph.get_positions_in_comm_range(self.input_graph.base_station):
                    # f^t_{bp} - |S|z^t_p <= 0
                    constr = LinearExpression()
                    constr_name = f"drone_flow_constr_{t}_base_p_{p}"
                    # (+) Flow that enters p from base station
                    constr.add_term(1, self.var_f_t_p_q(t, self.input_graph.base_station, p))
                    # (-) Number of sensors * z^t_p
                    constr.add_term(-1*len(self.targets_trace.trace_set), self.var_z_t_p(t, p))
                    # Has to be less or equal to 0
                    self.define_constraint(constr_name, constr.get_expression(), LESS_EQUAL, 0)

                for q in self.input_graph.get_positions_in_comm_range(p):
                    # f^t_{pq} - |S|z^t_p <= 0
                    constr = LinearExpression()
                    constr_name = f"drone_flow_constr_{t}_p_{p}_q_{q}"
                    # (+) Flow that leaves p to q
                    constr.add_term(1, self.var_f_t_p_q(t, p, q))
                    # (-) Number of sensors * z^t_p
                    constr.add_term(-1*len(self.targets_trace.trace_set), self.var_z_t_p(t, p))
                    # Has to be less or equal to 0
                    self.define_constraint(constr_name, constr.get_expression(), LESS_EQUAL, 0)

                for sensor in self.input_graph.get_position_coverage(p, self.targets_trace.get_targets_positions_at_time(t)):
                    constr = LinearExpression()
                    constr_name = f"drone_flow_constr_{t}_p_{p}_sensor_{sensor}"
                    # (+) Flow that leaves p to sensor
                    constr.add_term(1, self.var_f_t_p_q(t, p, sensor))
                    # (-) Number of sensors * z^t_p
                    constr.add_term(-1*len(self.targets_trace.trace_set), self.var_z_t_p(t, p))
                    # Has to be less or equal to 0
                    self.define_constraint(constr_name, constr.get_expression(), LESS_EQUAL, 0)

    def define_drone_integrity_constraints(self) -> None:
        """Defines the constraints to ensure a drone is always placed somewhere in P \cup {base_station} at any time step and that a drone can only be in one position at a time."""

        for t in range(self.observation_period):
            for drone in range(self.n_available_drones):
                constr = LinearExpression()
                constr_name = f"drone_position_constr_t_{t}_drone_{drone}"
                for p in self.input_graph.deployment_positions:
                    constr.add_term(1, self.var_z_t_drone_p(t, drone, p))
                constr.add_term(1, self.var_z_t_drone_p(t, drone, self.input_graph.base_station))
                self.define_constraint(constr_name, constr.get_expression(), EQUAL, 1)

    def define_position_use_constraints(self) -> None:
        """Defines the constraints to ensure that if there is a drone in p at time t, then z^t_p = 1. And at most one drone can be placed in a position p at a given time step."""
        for t in range(self.observation_period):
            for p in self.input_graph.deployment_positions:
                constr = LinearExpression()
                constr_name = f"position_use_constr_t_{t}_p_{p}"
                for drone in range(self.n_available_drones):
                    constr.add_term(1, self.var_z_t_drone_p(t, drone, p))
                constr.add_term(-1, self.var_z_t_p(t, p))
                self.define_constraint(constr_name, constr.get_expression(), EQUAL, 0)

    def define_drone_movement_constraints(self) -> None:
        """Defines the constraints that ensure the definition of the variables z^t_{upq}."""
        if self.observation_period <= 1: # In this case there are no movements within the observation period so there is no need to define these constraints
            return
        for t in range(1, self.observation_period):
            for drone in range(self.n_available_drones):
                for p in self.input_graph.deployment_positions + [self.input_graph.base_station]:
                    for q in self.input_graph.deployment_positions + [self.input_graph.base_station]:
                        # z^t_{upq} - z^{t-1}_up <= 0
                        constr = LinearExpression()
                        constr_name = f"drone_mov_constr1_t_{t}_drone_{drone}_p_{p}_q_{q}"
                        constr.add_term(1, self.var_z_t_drone_p_q(t, drone, p, q))
                        constr.add_term(-1, self.var_z_t_drone_p(t-1, drone, p))
                        self.define_constraint(constr_name, constr.get_expression(), LESS_EQUAL, 0)

                        # z^t_{upq} - z^t_uq <= 0
                        constr.clear_expression()
                        constr_name = f"drone_mov_constr1_t_{t}_drone_{drone}_p_{p}_q_{q}"
                        constr.add_term(1, self.var_z_t_drone_p_q(t, drone, p, q))
                        constr.add_term(-1, self.var_z_t_drone_p(t, drone, q))
                        self.define_constraint(constr_name, constr.get_expression(), LESS_EQUAL, 0)

                        # z^t_{upq} - z^t_q - z^{t-1}_p >= -1
                        constr.clear_expression()
                        constr_name = f"drone_mov_constr1_t_{t}_drone_{drone}_p_{p}_q_{q}"
                        constr.add_term(1, self.var_z_t_drone_p_q(t, drone, p, q))
                        constr.add_term(-1, self.var_z_t_drone_p(t, drone, q))
                        constr.add_term(-1, self.var_z_t_drone_p(t-1, drone, p))
                        self.define_constraint(constr_name, constr.get_expression(), GREATER_EQUAL, -1)

    def define_all_constraints(self) -> None:
        """Defines all the constraints of the linear program. Uses the function define_constraint to save the information of the constraints in the corresponding lists. Later the constraints must be added to the cplex model."""
        self.define_flow_constraints()
        self.define_drone_flow_constraints()
        self.define_drone_integrity_constraints()
        self.define_position_use_constraints()
        self.define_drone_movement_constraints()

    def get_objective_function(self) -> list:
        """ Returns the linear expression of the objective function.

        Returns:
            list: The linear expression of the objective function. This is a list of tuples with the form (variable_name (str), coefficient (float)).
        """
        obj_func = LinearExpression()

        for p in self.input_graph.deployment_positions:
            # Deployement cost (t = 0)
            energy_consumed = energy(self.input_graph.get_distance(p, self.input_graph.base_station), self.time_step_delta, False)
            distance_cost = (1 - self.alpha) * self.input_graph.get_distance(p, self.input_graph.base_station)
            energy_cost = self.alpha * self.beta * energy_consumed
            obj_func.add_term(distance_cost + energy_cost, self.var_z_t_p(0, p))
            # Return to base cost (t = T - 1)
            energy_consumed = energy(self.input_graph.get_distance(p, self.input_graph.base_station), self.time_step_delta, False)
            distance_cost = (1 - self.alpha) * self.input_graph.get_distance(p, self.input_graph.base_station)
            energy_cost = self.alpha * self.beta * energy_consumed
            obj_func.add_term(distance_cost + energy_cost, self.var_z_t_p(self.observation_period - 1, p))

        if self.observation_period == 1: # In this case the deployement and return costs use the same variable
            obj_func.merge_duplicates()  # So we merge them into one term
            return obj_func.get_tuple_expression()
        # Movement cost
        for p in self.input_graph.deployment_positions + [self.input_graph.base_station]:
            for q in self.input_graph.deployment_positions + [self.input_graph.base_station]:
                hover = False if (p == self.input_graph.base_station or q == self.input_graph.base_station) else True
                energy_consumed = energy(self.input_graph.get_distance(p, q), self.time_step_delta, hover)
                distance_cost = (1 - self.alpha) * self.input_graph.get_distance(p, q)
                energy_cost = self.alpha * self.beta * energy_consumed
                for t in range(1,self.observation_period):
                    for drone in range(self.n_available_drones):
                        obj_func.add_term(distance_cost + energy_cost, self.var_z_t_drone_p_q(t, drone, p, q))
        return obj_func.get_tuple_expression() # For some reason cplex API doesn't use the notation of linear expressions for constraints and objective function. Instead it uses a list of tuples with the form (variable_name (str), coefficient (float)).

    def set_variables_to_cplex(self) -> None:
        """Adds the variables to the cplex model."""
        var_names = [var["name"] for var in self.variables]
        var_lower_bounds = [var["lb"] for var in self.variables]
        var_upper_bounds = [var["ub"] for var in self.variables]
        var_types = [var["type"] for var in self.variables]
        self.cplex_model.variables.add(names = var_names, lb = var_lower_bounds, ub = var_upper_bounds, types = var_types)

    def set_constraints_to_cplex(self) -> None:
        """Adds the constraints to the cplex model."""
        constr_names = [constr["name"] for constr in self.constraints]
        constr_linear_expr = [constr["linear_expr"] for constr in self.constraints]
        constr_sense = [constr["sense"] for constr in self.constraints]
        constr_rhs = [constr["rhs"] for constr in self.constraints]

        self.cplex_model.linear_constraints.add(lin_expr = constr_linear_expr, senses = constr_sense, rhs = constr_rhs, names = constr_names)

    def set_objective_function_to_cplex(self, objective_function: list, maximize: Optional[bool] = True) -> None:
        """Sets the objective function to the cplex model.

        Args:
            objective_function (list): Linear expression of the objective function. A ¡
            maximize (bool, optional): If True, the objective function is maximized. If False, the objective function is minimized. Defaults to True.
        """
        self.cplex_model.objective.set_linear(objective_function)
        self.cplex_model.objective.set_sense(self.cplex_model.objective.sense.maximize if maximize else self.cplex_model.objective.sense.minimize)

    def build_model(self) -> None:
        """Builds the linear program by defining all the variables, constraints and objective function and adding them to the cplex model."""
        self.define_all_variables()
        self.define_all_constraints()

        self.set_variables_to_cplex()
        self.set_constraints_to_cplex()
        self.set_objective_function_to_cplex(self.get_objective_function(), maximize=False)

    def solve_model(self) -> None:
        """Solves the linear program and saves the time required to reach the solution."""
        self.start_time = self.cplex_model.get_time()
        self.cplex_model.solve()
        self.finish_time = self.cplex_model.get_time()
        self.solution_time = self.finish_time - self.start_time

    def get_drones_deployement(self) -> list:
        """Returns the deployment of drones at each time step.

        Returns:
            list: List of list of tuples. For each time step, for each drone, the position where the drone is deployed.
        """
        drones_deployement = []
        for t in range(self.observation_period):
            deployement_at_t = []
            for drone in range(self.n_available_drones):
                for p in self.input_graph.deployment_positions + [self.input_graph.base_station]:
                    if self.cplex_model.solution.get_values(self.var_z_t_drone_p(t, drone, p)) >= 0.9:
                        deployement_at_t.append(p)
            drones_deployement.append(deployement_at_t)
        return drones_deployement

    def cplex_finish(self) -> None:
        """Closes the cplex model."""
        self.cplex_model.end()

    def get_objective_value(self) -> float:
        """Returns the value of the objective function. In case of infeasible solution, returns -1."""
        if self.cplex_model.solution.get_status() in [OPTIMAL_SOLUTION, OPTIMAL_TOL_SOLUTION, ABORTED_FEASIBLE, TIME_LIMIT_FEASIBLE, MEMORY_LIMIT_FEASIBLE]:
            return self.cplex_model.solution.get_objective_value()
        return -1

    def cplex_save_solution(self, file_name: str) -> None:
        """Saves the solution of the linear program to a file using cplex method. Constains too much information.

        Args:
            file_name (str): Name of the file to save the solution.
        """
        self.cplex_model.solution.write(file_name)

    def save_solution(self, file_name: str) -> None:
        """Saves the solution of the linear program to a file.
        It saves the objective function value, time to reach the solution, and the deployment of drones at each time step.

        Args:
            file_name (str): Name of the file to save the solution.
        """
        with open(file_name, "w") as file:
            file.write(f"{'Solution status:':<30} {self.get_solution_status()}\n")
            file.write(f"{'Objective function value:':<30} {self.get_objective_value()}\n")
            file.write(f"{'Total Distance:':<30} {self.get_solution_distance()}\n")
            file.write(f"{'Total Energy:':<30} {self.get_solution_energy()}\n")
            file.write(f"{'Time to reach the solution:':<30} {self.solution_time}\n")
            if self.cplex_model.solution.get_status() in [OPTIMAL_SOLUTION, OPTIMAL_TOL_SOLUTION, ABORTED_FEASIBLE, TIME_LIMIT_FEASIBLE, MEMORY_LIMIT_FEASIBLE]:
                file.write(f"{'Drones deployment:':<30}\n")
                file.write("-------------------------------------------\n")
                drones_deployement = self.get_drones_deployement()
                file.write(f"{'time_step:':<15} {'drone:':<11} {'position:':}\n")
                for time_step in range(len(drones_deployement)):
                    for drone in range(len(drones_deployement[time_step])):
                        file.write(f"{time_step:<15} {drone:<11} {drones_deployement[time_step][drone]}\n")

    def model_shut_up(self) -> None:
        """Disables cplex output stream."""
        self.cplex_model.set_log_stream(None)
        self.cplex_model.set_error_stream(None)
        self.cplex_model.set_warning_stream(None)
        self.cplex_model.set_results_stream(None)

    def get_solution_status(self) -> int:
        """Returns the status of the solution. Use the constants defined in cplex_constants.py."""
        return self.cplex_model.solution.get_status()

    def get_solution_status_string(self) -> str:
        """Returns the status of the solution as a string."""
        return self.cplex_model.solution.get_status_string()

    def set_time_limit(self, time_limit: float) -> None:
        """Sets the time limit parameter of cplex.

        Args:
            time_limit: Limit in seconds before stopping the execution of cplex.
        """
        if time_limit > 0:
            self.cplex_model.parameters.timelimit.set(time_limit)

    def set_memory_limit(self, memory_limit: float) -> None:
        """Sets the memory limit parameter of cplex.

        Args:
            memory_limit: Limit in MB of the working memory before stopping the execution of cplex.
        """
        if memory_limit > 0:
            self.cplex_model.parameters.workmem.set(memory_limit)

    def get_solution_distance(self) -> float:
        """Returns the distance traveled by the drones in the solution. If the solution was not reached, returns -1."""
        if not self.cplex_model.solution.get_status() in [OPTIMAL_SOLUTION, OPTIMAL_TOL_SOLUTION, ABORTED_FEASIBLE, TIME_LIMIT_FEASIBLE, MEMORY_LIMIT_FEASIBLE]:
            return -1

        total_distance = 0
        drones_deployement = self.get_drones_deployement()
        for drone in range(self.n_available_drones):
            total_distance += self.input_graph.get_distance(self.input_graph.base_station, drones_deployement[0][drone]) # depoyement cost
            total_distance += self.input_graph.get_distance(drones_deployement[self.observation_period-1][drone], self.input_graph.base_station) # return to base cost
            for t in range(1, self.observation_period):
                total_distance += self.input_graph.get_distance(drones_deployement[t-1][drone], drones_deployement[t][drone]) # movement cost

        return total_distance

    def get_solution_energy(self) -> float:
        """Returns the energy consumed by the drones in the solution. If the solution was not reached, returns -1."""
        if not self.cplex_model.solution.get_status() in [OPTIMAL_SOLUTION, OPTIMAL_TOL_SOLUTION, ABORTED_FEASIBLE, TIME_LIMIT_FEASIBLE, MEMORY_LIMIT_FEASIBLE]:
            return -1

        total_energy = 0
        drones_deployement = self.get_drones_deployement()
        for drone in range(self.n_available_drones):
            total_energy += energy(self.input_graph.get_distance(self.input_graph.base_station, drones_deployement[0][drone]), self.time_step_delta, False) # depoyement cost
            total_energy += energy(self.input_graph.get_distance(drones_deployement[self.observation_period-1][drone], self.input_graph.base_station), self.time_step_delta, False) # return to base cost
            for t in range(1, self.observation_period):
                hover = False if (drones_deployement[t-1][drone] == self.input_graph.base_station or drones_deployement[t][drone] == self.input_graph.base_station) else True
                total_energy += energy(self.input_graph.get_distance(drones_deployement[t-1][drone], drones_deployement[t][drone]), self.time_step_delta, hover=hover) # movement cost

        return total_energy
