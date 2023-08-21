from typing import Optional
from graph import Graph
from targets_trace import Trace
try:
    import cplex
except ImportError:
    print("Error while importing cplex. Please refer to the README.md file for instructions on how to install cplex.")
    exit(1)

class MILPModel:
    # Constants for variables types
    BINARY_VARIABLE = "B"
    INTEGER_VARIABLE = "I"
    CONTINUOUS_VARIABLE = "C"

    # Constants for constraints sense
    GREATER_EQUAL = "G"
    EQUAL = "E"
    LESS_EQUAL = "L"

    def __init__(self, n_available_drones: int, observation_period: int, time_step_delta: float, targets_trace: Trace, input_graph: Graph, model_name: Optional[str] = "MILP_Model"):
        """Builds the linear program to obtain the optimal deployment of drones to cover all targets at all time steps.

        Args:
            n_available_drones: Number of drones available.
            observation_period: Amount of time steps.
            time_step_delta: Amount of seconds between time steps.
            targets_trace: The trajectories of the targets.
            input_graph: The topology of the problem with the set of deployment positions and targets coordinates at each time step.
            model_name: Name of the cplex model. Defaults to "MILP_Model".
        """        

        self.n_available_drones = n_available_drones
        self.observation_period = observation_period
        self.time_step_delta = time_step_delta
        self.targets_trace = targets_trace
        self.input_graph = input_graph

        self.cplex_model = cplex.Cplex()
        self.cplex_model.set_problem_name(model_name)

        self.variables_names = []
        self.variables_lower_bounds = []
        self.variables_upper_bounds = []
        self.variables_types = []

        self.constraints_names = []
        self.constraints_linear_expr = []
        self.constraints_sense = []
        self.constraints_right_hand_side = []

    def define_variable(self, var_name: str, var_lb: float, var_up: float, var_type: str):
        """Defines a variable and saves its information in the corresponding lists. This function does not add the variables to the cplex model. I'm using these lists is because addign variables and constraints in batches is faster than adding them one by one for some reason. 
        
        Args:
            var_name: Name of the variable.
            var_lb: Lower bound of the variable.
            var_up: Upper bound of the variable.
            var_type: Type of the variable. Use the constants CONTINUOUS_VARIABLE, INTEGER_VARIABLE or BINARY_VARIABLE.
        """            
        self.variables_names.append(var_name)
        self.variables_lower_bounds.append(var_lb)
        self.variables_upper_bounds.append(var_up)
        self.variables_types.append(var_type)

    def var_z_t_p(self, time_step: int, position: tuple) -> str:
        """Returns the name of the variable z_t_p. This is a binary variable for p \in P and t \in T that says if a drone is deployed at position p at time step t. For the base station this variable is an integer since the base station can have many drones in it simultaneously. This corresnponds to the variable z^t_p in the papers.
        
        Args:
            time_step: Time step.
            position: Position coordinates.
        Returns:
            The name of the variable z_t_p.
        """
        return "z_t_"+str(time_step)+"_p_" + str(position)

    def var_z_t_drone_p(self, time_step: int, drone: int, position: tuple) -> str:
        """Returns the name of the variable z_t_drone_p. This is a binary variable for p \in P and t \in T and drone \in n_available_drones that says if drone is deployed at position p at time step t. This corresnponds to the variable z^t_{pu} in the papers.
        
        Args:
            time_step: Time step.
            drone: Drone index.
            position: Position coordinates.

        Returns:
            The name of the variable z_t_drone_p.
        """
        return "z_t_"+str(time_step)+"_drone_"+str(drone)+"_p_"+str(position)

    def var_f_t_p_q(self, time_step: int, position_p: tuple, position_q: tuple) -> str:
        """Returns the name of the variable f_t_p_q. This is a continuous variable for p,q \in P and t \in T that says how much flow is sent from position p to position q at time step t. This corresnponds to the variable f^t_{pq} in the papers.

        Args:
            time_step: Time step.
            position_p: Position coordinates.
            position_q: Position coordinates.

        Returns:
            The name of the variable f_t_p_q.
        """
        return "f_t_"+str(time_step)+"_p_"+str(position_p)+"_q_"+str(position_q)

    def define_all_variables(self):
        """Defines all the variables of the linear program. Uses the function define_variable to save the information of the variables in the corresponding lists. Later the variables must be added to the cplex model."""
        # Defining the variables z_t_p for all t \in T and p \in P \cup {base_station}
        for t in range(self.observation_period):
            self.define_variable(self.var_z_t_p(t,self.input_graph.base_station),0,self.n_available_drones,self.INTEGER_VARIABLE)
            for p in self.input_graph.deployment_positions:
                self.define_variable(self.var_z_t_p(t,p),0,1,self.BINARY_VARIABLE)
        
        # Defining the variables z_t_drone_p for all t \in T, drone \in n_available_drones and p \in P \cup {base_station}
        for t in range(self.observation_period):
            for drone in range(self.n_available_drones):
                self.define_variable(self.var_z_t_drone_p(t,drone,self.input_graph.base_station),0,1,self.BINARY_VARIABLE)
                for p in self.input_graph.deployment_positions:
                    self.define_variable(self.var_z_t_drone_p(t,drone,p),0,1,self.BINARY_VARIABLE)

        # Defining the flow variables f_t_base_p for all t \in T and p \in P
        for t in range(self.observation_period):
            for p in self.input_graph.get_positions_in_comm_range(self.input_graph.base_station):
                self.define_variable(self.var_f_t_p_q(t,self.input_graph.base_station,p),0,len(self.targets_trace.trace_set),self.CONTINUOUS_VARIABLE)

        # Defining the flow variables f_t_p_q for all t \in T, p,q \in P and p \neq q
        for t in range(self.observation_period):
            for p in self.input_graph.deployment_positions:
                for q in self.input_graph.get_positions_in_comm_range(p):
                    self.define_variable(self.var_f_t_p_q(t,p,q),0,len(self.targets_trace.trace_set),self.CONTINUOUS_VARIABLE)

        # Defining the flow variables f_t_p_q for all t \in T, sensor_position \in trace_set and delpoyment_position \in P that covers the sensor_position
        for t in range(self.observation_period):
                for sensor_trace in self.targets_trace.trace_set:
                    for sensor_position in sensor_trace:
                        for deployment_position in self.input_graph.get_target_coverage(sensor_position):
                            self.define_variable(self.var_f_t_p_q(t,sensor_position,deployment_position),0,len(self.targets_trace.trace_set),self.CONTINUOUS_VARIABLE)

    def define_constraint(self, constr_name: str, constr_linear_expr: list, constr_sense:int , constr_rhs:float):
        """Defines a constraint and saves its information in the corresponding lists. This function does not add the constraints to the cplex model. I'm using these lists is because addign variables and constraints in batches is faster than adding them one by one for some reason.

        Args:
            constr_name: Name of the constraint.
            constr_linear_expr: Linear expression of the constraint.
            constr_sense: Sense of the constraint. Use cplex_model.objective.sense.minimize or cplex_model.objective.sense.maximize.
            constr_rhs: Right hand side of the constraint.
        """            
        self.constraints_names.append(constr_name)
        self.constraints_linear_expr.append(constr_linear_expr)
        self.constraints_sense.append(constr_sense)
        self.constraints_right_hand_side.append(constr_rhs)

    def define_flow_constraints(self):
        """Defines the flow constraints for all time steps."""
        for t in range(self.observation_period):
            for p in self.input_graph.deployment_positions:
                constr_linear_expr = []
                constr_name = "flow_conservation_t_"+str(t)+"_p_"+str(p)

                if p in self.input_graph.get_positions_in_comm_range(self.input_graph.base_station):
                    constr_linear_expr.append([self.var_f_t_p_q(t,self.input_graph.base_station,p), -1]) # (-) Flow that enters p from base station
                for q in self.input_graph.get_positions_in_comm_range(p):
                    constr_linear_expr.append([self.var_f_t_p_q(t,p,q),1])  # (+) Flow that leaves p to q
                    constr_linear_expr.append([self.var_f_t_p_q(t,q,p),-1]) # (-) Flow that enters p from q

                for sensor in self.input_graph.get_position_coverage(p,self.targets_trace.get_targets_positions_at_time(t)):
                    constr_linear_expr.append([self.var_f_t_p_q(t,p,sensor),1]) # (+) Flow that leaves p to sensor

                # For any position p, the flow that enters p must be equal to the flow that leaves p at any time step
                self.define_constraint(constr_name,constr_linear_expr,self.EQUAL,0) 

            for sensor in self.targets_trace.get_targets_positions_at_time(t):
                constr_linear_expr = []
                constr_name = "flow_conservation_t_"+str(t)+"_sensor_"+str(sensor)
                for p in self.input_graph.get_target_coverage(sensor):
                    constr_linear_expr.append([self.var_f_t_p_q(t,p,sensor),1]) # (+) Flow that leaves p to sensor
                
                # At any time step, a sensor must receive at least one flow
                self.define_constraint(constr_name,constr_linear_expr,self.GREATER_EQUAL,1)
    
    def define_drone_flow_constraints(self):
        """This constraint ensures a flow only exists if a drone is deployed at the source position."""
        for t in range(self.observation_period):
            for p in self.input_graph.deployment_positions:
                if p in self.input_graph.get_positions_in_comm_range(self.input_graph.base_station):
                    constr_linear_expr = []
                    constr_name = "drone_flow_constr_"+str(t)+"_p_"+str(p)
                    constr_linear_expr.append([self.var_f_t_p_q(t,self.input_graph.base_station,p), 1]) # (+) Flow that enters p from base station
                    constr_linear_expr.append([self.var_z_t_p(t,p), -1*len(self.targets_trace.trace_set)]) # (-) Number of sensors * z^t_p

                    self.define_constraint(constr_name,constr_linear_expr,self.LESS_EQUAL,0) # Has to be less or equal to 0

                for q in self.input_graph.get_positions_in_comm_range(p):
                    constr_linear_expr = []
                    constr_name = "drone_flow_constr_"+str(t)+"_p_"+str(p)+"_q_"+str(q)
                    constr_linear_expr.append([self.var_f_t_p_q(t,p,q), 1])
                    constr_linear_expr.append([self.var_z_t_p(t,p), -1*len(self.targets_trace.trace_set)])
                    self.define_constraint(constr_name,constr_linear_expr,self.LESS_EQUAL,0)

                for sensor in self.input_graph.get_position_coverage(p,self.targets_trace.get_targets_positions_at_time(t)):
                    constr_linear_expr = []
                    constr_name = "drone_flow_constr_"+str(t)+"_p_"+str(p)+"_sensor_"+str(sensor)
                    constr_linear_expr.append([self.var_f_t_p_q(t,p,sensor), 1])
                    constr_linear_expr.append([self.var_z_t_p(t,p), -1*len(self.targets_trace.trace_set)])
                    self.define_constraint(constr_name,constr_linear_expr,self.LESS_EQUAL,0)

    def set_variables_to_cplex(self):
        """Adds the variables to the cplex model."""
        self.cplex_model.variables.add(names = self.variables_names, lb = self.variables_lower_bounds, ub = self.variables_upper_bounds, types = self.variables_types)

    def set_constraints_to_cplex(self):
        """Adds the constraints to the cplex model."""
        self.cplex_model.linear_constraints.add(lin_expr = self.constraints_linear_expr, senses = self.constraints_sense, rhs = self.constraints_right_hand_side, names = self.constraints_names)

    def set_objective_function(self, objective_function: list, maximize: Optional[bool] = True):
        """Sets the objective function of the cplex model.

        Args:
            objective_function (list): List of tuples describing the objective function. Each tuple has the form (variable_name (str), coefficient (float)).
            maximize (bool, optional): If True, the objective function is maximized. If False, the objective function is minimized. Defaults to True.
        """            
        self.cplex_model.objective.set_linear(objective_function)
        self.cplex_model.objective.set_sense(self.cplex_model.objective.sense.maximize if maximize else self.cplex_model.objective.sense.minimize)
    