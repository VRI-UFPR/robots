SHELL := /bin/bash

cpu-rasp:
	make -C cpu_rasp

doc:
	cd docs; python3 architecture.py

clean:
	make -C docs clean
	make -C cpu_rasp clean