from import_file import *

class MILPModel:
    def __init__(self, n_available_drones, observation_period, time_step_delta, communication_range, coverage_range, input_graph, model_name = "MILP_Model"):
        """Builds the linear program to obtain the optimal deployment of drones to cover all targets at all time steps.

        Args:
            n_available_drones (int): Number of drones available.
            observation_period (int): Amount of time steps.
            time_step_delta (float): Amount of seconds between time steps.
            communication_range (float): The range of drones communication.
            coverage_range (float): The maximmum distance for communication between a drone and a target.
            input_graph (Graph): The topology of the problem with the set of deployment positions and targets coordinates at each time step.
            model_name (str, optional): Name of the cplex model. Defaults to "MILP_Model".
        """        

        self.n_available_drones = n_available_drones
        self.observation_period = observation_period
        self.time_step_delta = time_step_delta
        self.communication_range = communication_range
        self.coverage_range = coverage_range
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

        def define_variable(self, var_name, var_lb, var_up, var_type):
            """Defines a variable. And saves its information in the corresponding lists. This function does not add the variables to the cplex model.The reason I'm using these lists is because addign variables and constraints in batches is faster than adding them one by one for some reason.

            Args:
                var_name (str): Name of the variable.
                var_lb (float): Lower bound of the variable.
                var_up (float): Upper bound of the variable.
                var_type (str): Type of the variable.
            """            
            self.variables_names.append(var_name)
            self.variables_lower_bounds.append(var_lb)
            self.variables_upper_bounds.append(var_up)
            self.variables_types.append(var_type)

