# IF FOR SOME REASON THEY CHANGE THE DEFINITION OF THE CONSTANTS IN THE CPLEX MODULE, WE CAN CHANGE THEM HERE
# https://www.ibm.com/docs/en/icos/20.1.0?topic=micclcarm-solution-status-codes-by-number-in-cplex-callable-library-c-api
# Constants for variables type
BINARY_VARIABLE = "B"
INTEGER_VARIABLE = "I"
CONTINUOUS_VARIABLE = "C"
# Constants for constraints sense"
GREATER_EQUAL = "G"
EQUAL = "E"
LESS_EQUAL = "L"
# Constants for CPLEX solution status
OPTIMAL_SOLUTION = 101
OPTIMAL_TOL_SOLUTION = 102
INFEASIBLE_SOLUTION = 103
TIME_LIMIT_FEASIBLE = 107
TIME_LIMIT_INFEASIBLE = 108
MEMORY_LIMIT_FEASIBLE = 111
MEMORY_LIMIT_INFEASIBLE = 112
ABORTED_FEASIBLE = 113