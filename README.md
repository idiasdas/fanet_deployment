# Optimizing drones trajectory to form a flying network

This project is a remake of the works presented on:

* I. Dias da Silva, C. Caillouet, D. Coudert. "Optimizing FANET deployment for mobile sensor tracking in disaster management scenario". ICT-DM, Dec 2021.  [(IEEE-9664204)](https://ieeexplore.ieee.org/abstract/document/9664204)

* I. Dias Da Silva and C. Caillouet, "Optimizing the trajectory of drones: trade-off between distance and energy", in 2nd International Workshop on Internet of Autonomous Unmanned Vehicles (IAUV) in conjunction with IEEE SECON 2020, Como, Italy, Jun. 2020.  [(IEEE-9149781)](https://ieeexplore.ieee.org/abstract/document/9149781)

Below you will find the instructions to replicate our experiments. If you run into any problems or have any questions about this projet (papers, code or whatever), do not exitate to contact me directly on [igor.dias-da-silva@inria.fr](mailto:igor.dias-da-silva.inria.fr).

## REQUIREMENTS

For this project you will need:
- **Python 3.7**: This project requires Python 3.7 specifically because of cplex.
    - If you don't have python 3.7 installed, please [download and install it](https://www.python.org/downloads/release/python-3716/) first.
    - I'm not sure about newer versions of cplex, but I used version 1210 and the API doesn't work with newer versions of python.
- **CPLEX**: Ensure that CPLEX is properly installed and accessible via Python:
    - [Download and install it](https://www.ibm.com/docs/en/icos/20.1.0?topic=cplex-installing).
    - [Set the environment variable PYTHONPATH](https://www.ibm.com/docs/en/icos/20.1.0?topic=cplex-setting-up-python-api).
    - Below you can find how to verify if cplex is properly installed using the makefile.

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
Whenever you are working on this project, ensure that you are using one these two virtual environments. Use the following command to activate `venv fanet`:

```bash
. fanet/bin/activate
```

The followig command activates the conda environment `fanet`:

```bash
conda activate fanet
```

**Verify CPLEX Installation**:
Ensure that CPLEX is properly installed and accessible in the virtual environment:
```bash
make check-cplex
```

## TESTS
This project uses `PyTest` and all tests can be found in `fanet_deployment/tests/`. If you wish to run these tests, use:

```bash
make test
```

If you want a coverage report, use:

```bash
make test-cov
```

## CONFIGURATION

The directory `/fanet_deployment/fanet/setup/` constains the main configuration files which are:
- `config.py`: Defines TESTS_OUTPUT_DIR and FILES_DIR which determine where the tests outputs and the models results are saved. It also defines PARAMETERS, a dictionary with all relevant information used by the models.
- `parameters.py`: By default, PARAMETERS = DEFAULT_PARAMETERS which are defined here. Use this file to define your own parameters dictionaries and then set PARAMETERS accordingly in `config.py`.
- `cplex_constants`: This files constains constants used in cplex module to determine variables types, constraints senses and objective status. I have defined them here so that any changes to these values can be quickly applied to the project. Although these are very unlikely.

The parameters used are the following:
| PARAMETER | DESCRIPTION | TYPE |
| --------- | ----------- | ---- |
| `n_drones`| number of available drones | list of integers |
| `n_targets`| number of targets | list of integers |
| `targets_speed`| speed of the targets in m/s | list of floats |
| `observation_period`| number of time steps | integer |
| `time_step_delta`| time between each time step in seconds | float |
| `alpha`| weight between distance and energy | list of floats |
| `beta`| normalization factor | float |
| `area_size`| size of the square area in meters | float |
| `n_positions`| number of axis splits to form the grid | list of integers |
| `heights`| possible heights for deployment positions| list of floats |
| `base_station`| base station position | tuple of floats (x,y,h) |
| `comm_range`| communication range in meters | float |
| `coverage_angle`| coverage angle in radians | float |
| `n_instances`| number of instances to generate for each parameter combination | integer |
| `cplex_workmem_limit`| cplex maximum memory in MB | integer |
| `cplex_time_limit`| cplex maximum time in seconds | integer |
| `experiment_name`| name of the experiment (becomes a folder in FILES_DIR with the results) | string |

**If you make any changes to these files, run the tests again** to ensure they are valid. Keep in mind the tests only verify the dictionary referenced by PARAMETERS in `config.py`.

## CREATE THE SENSORS TRACES

You will need to create the targets traces (sequence of sensors positions) that are the input to the models. To do so, run the following command:

```bash
make generate-traces
```

This command verifies if the traces already exist and if they don't, creates them. These traces are saved to `fanet_deployment/files/traces/`. If you want to delete the current traces use:

```bash
make clean-traces
```
Now that you are in an activate `fanet` virtual environment and `make check-cplex` has validated the accessiblity to the cplex model, you can execute any code in this project. The following section explains how to replicate our experiments results.

## SOLVE MILP MODEL

Once the targets traces have been created we can apply our models to them. To build and solve the Mixed-Integer Linear Program defined in [IEEE-9149781](https://ieeexplore.ieee.org/abstract/document/9149781), use:

```bash
make solve-milp
```

By default, it will use DEFAULT_PARAMETERS, which save the results in `fanet_deployment/files/default/`.


