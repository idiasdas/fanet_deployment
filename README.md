# Optimizing drones trajectory to form a flying network

This project is a remake of the works presented on:

* I. Dias da Silva, C. Caillouet, D. Coudert. "Optimizing FANET deployment for mobile sensor tracking in disaster management scenario". ICT-DM, Dec 2021.  [(IEEE-9664204)](https://ieeexplore.ieee.org/abstract/document/9664204)

* I. Dias Da Silva and C. Caillouet, "Optimizing the trajectory of drones: trade-off between distance and energy", in 2nd International Workshop on Internet of Autonomous Unmanned Vehicles (IAUV) in conjunction with IEEE SECON 2020, Como, Italy, Jun. 2020.  [(IEEE-9149781)](https://ieeexplore.ieee.org/abstract/document/9149781)

---

## Requirements

For this project you will need:
- **Python 3.7**: This project requires Python 3.7 specifically because of cplex. 
    - If you don't have python 3.7 installed, please [download and install it](https://www.python.org/downloads/release/python-3716/) first.
    - I'm not sure about newer versions of cplex, but I used version 1210 and the API doesn't work with newer versions of python.
- **CPLEX**: Ensure that CPLEX is properly installed and accessible via Python:
    - [Download and install it](https://www.ibm.com/docs/en/icos/20.1.0?topic=cplex-installing).
    - [Set the environment variable PYTHONPATH](https://www.ibm.com/docs/en/icos/20.1.0?topic=cplex-setting-up-python-api).
    - Below you can find how to verify if cplex is properly installed using the makefile.
## Setup

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

 **Verify CPLEX Installation**:
Ensure that CPLEX is properly installed and accessible in the virtual environment:
```bash
make check-cplex
```

## Create the sensors traces

You will need to create the targets traces (sequence of sensors positions) that are the input to the models. To do so, run the following command:

```bash
make generate-traces
```

This command verifies if the traces already exist and if they don't, creates them. These traces are saved to `fanet_deployment/files/traces/`. If you want to delete the current traces use:

```bash
make clean-traces
```

