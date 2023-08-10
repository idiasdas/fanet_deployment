import os
import sys
sys.path.append(os.path.dirname(__file__) + '/../src/')  # Now can import modules in src

from import_file import *

# CPLEX CLASS
# https://www.ibm.com/docs/en/icos/20.1.0?topic=classes-cplexcplex#problem_type
cplex_model = cplex.Cplex()
cplex_model.set_problem_name("TestModel")
# TYPES OF PROBLEMS
# https://www.ibm.com/docs/en/icos/20.1.0?topic=SSSA5P_20.1.0/ilog.odms.cplex.help/refpythoncplex/html/cplex._internal.ProblemType-class.html
cplex_model.set_problem_type(cplex.Cplex.problem_type.MILP)
print("The problem type is: " + cplex_model.problem_type[cplex_model.get_problem_type()])
# VARIABLES CLASS
# https://www.ibm.com/docs/en/icos/20.1.0?topic=SSSA5P_20.1.0/ilog.odms.cplex.help/refpythoncplex/html/cplex._internal._subinterfaces.VariablesInterface-class.html
# TYPES OF VARIABLES
# https://www.ibm.com/docs/en/icos/20.1.0?topic=SSSA5P_20.1.0/ilog.odms.cplex.help/refpythoncplex/html/cplex._internal._subinterfaces.VarTypes-class.html
cplex_model.variables.add(names = ["x1", "x2"], lb = [0,0], ub = None, types = [cplex_model.variables.type.continuous]*2)
print("The variables are: " + str(cplex_model.variables.get_names()))
constraints_linear_expr = []
constraints_sense = []
constraints_right_hand_side = []
constraints_names = []

# -x_1 + 2x_2 <= 7
constraints_linear_expr.append(cplex.SparsePair(ind = ["x1", "x2"], val = [-1, 2]))
constraints_sense.append("L")
constraints_right_hand_side.append(7)
constraints_names.append("constr_1")

# 0.2x_1 x_2 <= 4
constraints_linear_expr.append(cplex.SparsePair(ind = ["x1", "x2"], val = [0.2, 1]))
constraints_sense.append("L")
constraints_right_hand_side.append(4)
constraints_names.append("constr_2")

# 2x_1 + x_2 <= 8
constraints_linear_expr.append(cplex.SparsePair(ind = ["x1", "x2"], val = [2, 1]))
constraints_sense.append("L")
constraints_right_hand_side.append(8)
constraints_names.append("constr_3")

cplex_model.linear_constraints.add(lin_expr = constraints_linear_expr, senses = constraints_sense, rhs = constraints_right_hand_side, names = constraints_names)
print("The constraints are: " + str(cplex_model.linear_constraints.get_names()))

# OBJECTIVE FUNCTION
# https://www.ibm.com/docs/en/icos/12.10.0?topic=SSSA5P_12.10.0/ilog.odms.cplex.help/refpythoncplex/html/cplex._internal._subinterfaces.ObjectiveInterface-class.html

cplex_model.objective.set_linear([("x1",1),("x2",1)])
cplex_model.objective.set_sense(cplex_model.objective.sense.maximize)

#SOLVE
cplex_model.solve()
print(cplex_model.solution.get_values())
# FINISHES CPLEX
cplex_model.end()