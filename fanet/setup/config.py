import os
from fanet.setup.parameters import DEFAULT_PARAMETERS, TEST_PARAMETERS, TEST_TIME_LIMIT, EXPERIMENT_PARAMETERS

# Parameters used in the project.
PARAMETERS = EXPERIMENT_PARAMETERS
# Path to the root directory of the project
BASE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(__file__))) + "/"
# Path where tests results are stored
TESTS_OUTPUT_DIR = os.path.join(BASE_DIR, "tests", "out") + "/"
# Path where files are stored such as traces, solutions and figures.
FILES_DIR = os.path.join(BASE_DIR, "files") + "/"
