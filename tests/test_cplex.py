import os
import sys
import unittest
sys.path.append(os.path.dirname(__file__) + '/../src/')  # Now can import modules in src

from linear_expression import LinearExpression
try:
    import cplex
except ImportError:
    print("Error importing required modules. Refer to README.md for more information.")
    exit(1)

class Test_cplex_module(unittest.TestCase):
    def test_cplex(self):
        """This test verify if the cplex module is working properly.
            It creates a small linear program and solves it.
            ----------------------------------------
            The linear program is:
            Maximize x_1 + x_2
            Subject to:
                -x_1 + 2x_2 <= 7
                0.2x_1 + x_2 <= 4
                2x_1 + x_2 <= 8
                x_1, x_2 >= 0
            And the solution is:
                x_1 = 2.22
                x_2 = 3.55
            ----------------------------------------
            You can see this as a small example of how to use the cplex module (or not).
            The relevant documentation is:
                * CPLEX CLASS
                    - https://www.ibm.com/docs/en/icos/20.1.0?topic=classes-cplexcplex#problem_type
                * TYPES OF PROBLEMS
                    - https://www.ibm.com/docs/en/icos/20.1.0?topic=SSSA5P_20.1.0/ilog.odms.cplex.help/refpythoncplex/html/cplex._internal.ProblemType-class.html
                * VARIABLES CLASS
                    - https://www.ibm.com/docs/en/icos/20.1.0?topic=SSSA5P_20.1.0/ilog.odms.cplex.help/refpythoncplex/html/cplex._internal._subinterfaces.VariablesInterface-class.html
                * TYPES OF VARIABLES
                    - https://www.ibm.com/docs/en/icos/20.1.0?topic=SSSA5P_20.1.0/ilog.odms.cplex.help/refpythoncplex/html/cplex._internal._subinterfaces.VarTypes-class.html
                * OBJECTIVE FUNCTION
                    - https://www.ibm.com/docs/en/icos/12.10.0?topic=SSSA5P_12.10.0/ilog.odms.cplex.help/refpythoncplex/html/cplex._internal._subinterfaces.ObjectiveInterface-class.html
        """
        # CREATES THE CPLEX MODEL
        cplex_model = cplex.Cplex()
        cplex_model.set_problem_name("TestModel")
        cplex_model.set_problem_type(cplex.Cplex.problem_type.MILP)

        cplex_model.set_log_stream(None)
        cplex_model.set_error_stream(None)
        cplex_model.set_warning_stream(None)
        cplex_model.set_results_stream(None)

        # DEFINES THE VARIABLES
        cplex_model.variables.add(names = ["x1", "x2"], lb = [0,0], ub = None, types = [cplex_model.variables.type.continuous]*2)

        # LISTS TO STORE THE CONSTRAINTS (apparently adding variables and constraints to the model is faster if you do it in batches)
        constraints_linear_expr = []
        constraints_sense = []
        constraints_right_hand_side = []
        constraints_names = []

        # -x_1 + 2x_2 <= 7
        constr = LinearExpression()
        constr.add_term(-1, "x1")
        constr.add_term(2, "x2")
        constraints_linear_expr.append(constr.get_expression())
        constraints_sense.append("L")
        constraints_right_hand_side.append(7)
        constraints_names.append("constr_1")

        # 0.2x_1 x_2 <= 4
        constr.clear_expression()
        constr.add_term(0.2, "x1")
        constr.add_term(1, "x2")
        constraints_linear_expr.append(constr.get_expression())
        constraints_sense.append("L")
        constraints_right_hand_side.append(4)
        constraints_names.append("constr_2")

        # 2x_1 + x_2 <= 8
        constr.clear_expression()
        constr.add_term(2, "x1")
        constr.add_term(1, "x2")
        constraints_linear_expr.append(constr.get_expression())
        constraints_sense.append("L")
        constraints_right_hand_side.append(8)
        constraints_names.append("constr_3")

        # ADDS THE CONSTRAINTS TO THE MODEL
        cplex_model.linear_constraints.add(lin_expr = constraints_linear_expr, senses = constraints_sense, rhs = constraints_right_hand_side, names = constraints_names)

        # OBJECTIVE FUNCTION
        cplex_model.objective.set_linear([("x1",1),("x2",1)])
        cplex_model.objective.set_sense(cplex_model.objective.sense.maximize)

        #SOLVE
        cplex_model.solve()
        self.assertEqual(cplex_model.solution.get_status(), 101)

        self.assertEqual(round(cplex_model.solution.get_objective_value(),5), 5.77778)
        self.assertEqual(round(cplex_model.solution.get_values("x1"),5), 2.22222)
        self.assertEqual(round(cplex_model.solution.get_values("x2"),5), 3.55556)

        # FINISHES CPLEX
        cplex_model.end()