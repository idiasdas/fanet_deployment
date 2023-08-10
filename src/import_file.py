
"""All packages imported in the project."""
try:
    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib import collections  as mc
except:
    raise Exception('Please install requirements.txt. Use one of the following options:\n\t $ pip install -r requirements.txt \n\t $ conda env create -f environment.yml')
    
try:
    import cplex
except:
    raise Exception('Please install cplex and add it to PYTHONPATH. See https://www.ibm.com/docs/en/icos/20.1.0?topic=cplex-setting-up-python-api')