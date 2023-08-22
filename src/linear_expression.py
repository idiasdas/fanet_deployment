class LinearExpression:
    def __init__(self):
        """Describes a linear expression as a list [[variables],[coefficients]].]"""
        self.variables = []
        self.coefficients = []
    
    def add_term(self, coefficient: float, variable: str):
        """Adds a term to the expression.

        Args:
            coefficient: Coefficient of the term
            variable: Variable of the term
        """
        self.variables.append(variable)
        self.coefficients.append(coefficient)

    def get_expression(self) -> list:
        """Returns the expression.

        Returns:
            List of terms of the expression
        """
        return [self.variables, self.coefficients]
    
    def get_tuple_expression(self) -> list:
        """Returns the expression as a list of tuples. For some reason (bad API) cplex API doesn't use the notation of linear expressions for constraints and objective function. Instead it uses a list of tuples with the form (variable_name (str), coefficient (float)).
    

        Returns:
            List of tuples (variable, coefficient) of the expression
        """
        return list(zip(self.variables, self.coefficients))
    
    def clear_expression(self):
        """Clears the expression."""
        self.variables = []
        self.coefficients = []

    def merge_duplicates(self):
        """When there are duplicates of variables, it merges them into one term. [["x1","x2","x1"],[1,2,3]] -> [["x1","x2"],[4,2]]"""
        variables = []
        coefficients = []
        for i in range(len(self.variables)):
            if self.variables[i] in variables:
                coefficients[variables.index(self.variables[i])] += self.coefficients[i]
            else:
                variables.append(self.variables[i])
                coefficients.append(self.coefficients[i])
        self.variables = variables
        self.coefficients = coefficients
