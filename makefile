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
	python fanet/generate_traces.py

# Target to delete all .txt files (traces) from files/traces
.PHONY: clean-traces
clean-traces:
	rm -f files/traces/*.txt

# Verifies if cplex is installed and accessible
.PHONY: check-cplex
check-cplex:
	@python -c "import cplex" 2>/dev/null && echo "cplex is installed and accessible." || echo "cplex is not accessible. Please refer to the README for more information."

# Target to run all tests with PyTest
.PHONY: test
test:
	pytest -v tests/ -W ignore::DeprecationWarning

# Target to run all tests with PyTest and coverage report
.PHONY: test-cov
test-cov:
	pytest -v --cov-report term-missing --cov=fanet tests/ -W ignore::DeprecationWarning

# Updates the environment.yml and requirements.txt files
.PHONY: update-deps
update-deps:
	# Update environment.yml
	conda env export --name fanet | grep -v "prefix:" > environment.yml

	# Create requirements.txt from conda list
	conda list --name fanet --export | grep -v "^#" | awk -F' ' '{print $$1"=="$$2}' > requirements.txt
