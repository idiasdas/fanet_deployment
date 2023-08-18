# Choose either setup-requirements or setup-environment to install the required modules for the project.
.PHONY: setup-venv
setup-venv:
	python3.7 -m venv fanet
	. fanet/bin/activate && pip install -r requirements.txt

.PHONY: setup-conda
setup-conda:
	conda env create -f environment.yml

