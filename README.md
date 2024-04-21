# Optimizing drone trajectories to form flying networks

This project is a remake of the works presented on:

* I. Dias da Silva, C. Caillouet, D. Coudert. "Optimizing FANET deployment for mobile sensor tracking in disaster management scenario". ICT-DM, Dec 2021.  [(IEEE-9664204)](https://ieeexplore.ieee.org/abstract/document/9664204)

* I. Dias Da Silva and C. Caillouet, "Optimizing the trajectory of drones: trade-off between distance and energy", in 2nd International Workshop on Internet of Autonomous Unmanned Vehicles (IAUV) in conjunction with IEEE SECON 2020, Como, Italy, Jun. 2020.  [(IEEE-9149781)](https://ieeexplore.ieee.org/abstract/document/9149781)

Below, I've included the instructions to reproduce our experiments. If you have any problems or questions about this project (papers, code or whatever), do not hesitate to contact me.

## REQUIREMENTS

For this project, you will need the following:
- **Python 3.7**: This project requires Python 3.7 specifically because of CPLEX.
    - If you don't have Python 3.7 installed, please [download and install it](https://www.python.org/downloads/release/python-3716/) first.
      
>[!NOTE]
>I'm not sure about newer versions of CPLEX, but I used version 1210, and the API doesn't work with newer Python versions.

- **CPLEX**: Ensure that CPLEX is properly installed and accessible via Python:
    - [Download and install it](https://www.ibm.com/docs/en/icos/20.1.0?topic=cplex-installing).
    - [Set the environment variable PYTHONPATH](https://www.ibm.com/docs/en/icos/20.1.0?topic=cplex-setting-up-python-api).
    - Below, you can see how to verify if CPLEX is properly installed using the makefile.

## SETUP

**Clone the Repository**:
```bash
git clone https://github.com/idiasdas/fanet_deployment
cd fanet_deployment
```

**Setup Environment**:
You can choose between a virtual environment and a conda environment. Run the following command to create a virtual environment named `fanet` and install the required packages using pip:
```bash
make setup-venv
```
Run the following command to create a conda environment named `fanet` with the required packages:
```bash
make setup-conda
```
When working on this project, ensure you use one of these two virtual environments. Use the following command to activate `venv fanet`:

```bash
. fanet/bin/activate
```

The following command activates the conda environment `fanet`:

```bash
conda activate fanet
```

**Verify CPLEX Installation**:
Ensure that CPLEX is properly installed and accessible in the virtual environment:
```bash
make check-cplex
```

## TESTS
This project uses `PyTest`. All tests can be found in `fanet_deployment/tests/`. If you wish to run these tests, use:

```bash
make test
```

If you want a coverage report, use:

```bash
make test-cov
```

## CONFIGURATION

The directory `/fanet_deployment/fanet/setup/` contains the main configuration files, which are:
- `config.py`: Defines TESTS_OUTPUT_DIR and FILES_DIR, which determine where the test outputs and the model results are saved. It also defines PARAMETERS, a dictionary with all relevant information the models use.
- `parameters.py`: By default, PARAMETERS = DEFAULT_PARAMETERS, which are defined here. Use this file to define your own parameters dictionaries and then set PARAMETERS accordingly in `config.py`.
- `cplex_constants`: This file contains constants used in the CPLEX module to determine variable types, constraints senses and objective status. I have defined them here so that any changes to these values can be quickly applied to the project, although these are very unlikely.

The parameters used are the following:
| PARAMETER | DESCRIPTION | TYPE |
| --------- | ----------- | ---- |
| n_drones | Number of available drones | List of integers |
| n_targets | Number of targets | List of integers |
| targets_speed | Speed of the targets in m/s | List of floats |
| observation_period | Number of time steps | Integer |
| time_step_delta | Time between each time step in seconds | Float |
| alpha | Weight between distance and energy | List of floats |
| beta | Normalization factor | Float |
| area_size | Size of the square area in meters | Float |
| n_positions | Number of axis splits to form the grid | List of integers |
| heights | Possible heights for deployment positions| List of floats |
| base_station | Base station position | Tuple of floats (x,y,h) |
| comm_range | Communication range in meters | Float |
| coverage_angle | Coverage angle in radians | Float |
| n_instances | Number of instances to generate for each parameter combination | Integer |
| cplex_workmem_limit | CPLEX maximum memory in MB | Integer |
| cplex_time_limit | CPLEX maximum time in seconds | Integer |
| experiment_name | Name of the experiment (becomes a folder in FILES_DIR with the results) | String |

>[!WARNING]
>**If you make any changes to these files, rerun the tests** to ensure they are valid. Remember that the tests only verify the dictionary referenced by PARAMETERS in `config.py`.

## CREATE THE SENSORS TRACES

You will need to create the target traces (sequences of sensor positions) that are the input to the models. To do so, run the following command:

```bash
make generate-traces
```

This command verifies if the traces exist, and if they don't, creates them. These traces are saved to `fanet_deployment/files/traces/`. If you want to delete the current traces, use the following:

```bash
make clean-traces
```
Now that you are in an activated `fanet` environment (venv or conda) and `make check-cplex` has validated the accessibility to the CPLEX module, you can execute any code in this project.
## SOLVE MILP MODEL

To build and solve the Mixed-Integer Linear Program defined in [IEEE-9149781](https://ieeexplore.ieee.org/abstract/document/9149781), use:

```bash
make solve-milp
```

This command will solve the MILP model for each trace and each combination of parameters described by PARAMETERS. The results are all saved to `FILES_DIR + PARAMETERS["experiment_name"]`. Whenever the solution file already exists for an instance, we skip it.
Therefore, if you change the parameters and wish to solve the same instances again, clear the results directory or change the experiment_name parameter. Remember that big instances of the problem require much time and memory. We are talking about days and tens of GB of memory for huge instances. The default parameters limit both to 3 hours and 10 GB, respectively. When CPLEX reaches these limits, we save the best solution found so far and the [solution status](https://www.ibm.com/docs/en/icos/20.1.0?topic=micclcarm-solution-status-codes-by-number-in-cplex-callable-library-c-api) accordingly. Adjust the parameters according to what is feasible for you. 

Once again, if you have any problems, please don't hesitate to contact me.


