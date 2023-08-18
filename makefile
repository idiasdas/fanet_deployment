# Choose either setup-requirements or setup-environment to install the required modules for the project.
.PHONY: setup-venv
setup-venv:
	python3.7 -m venv fanet
	. fanet/bin/activate && pip install -r requirements.txt

.PHONY: setup-conda
setup-conda:
	conda env create -f environment.yml

# Target to run the generate_traces.py script
.PHONY: generate-traces
generate-traces:
	python src/generate_traces.py

# Target to delete all .txt files (traces) from files/traces
.PHONY: clean-traces
clean-traces:
	rm -f files/traces/*.txt

# Verifies if cplex is installed and accessible
.PHONY: check-cplex
check-cplex:
	@python -c "import cplex" 2>/dev/null && echo "cplex is installed and accessible." || echo "cplex is not accessible. Please refer to the README for more information."
