SHELL=/bin/bash
TOP_DIR=$(shell TOP_DIR=Unable_To_Find_Top_Dir; \
				CUR_DIR=$$(pwd); \
				while [ "$$CUR_DIR" != "/" ]; \
				do { \
					if [ -a $$CUR_DIR/.git ]; then \
					TOP_DIR=$$CUR_DIR; \
					fi; \
				    CUR_DIR=$$(dirname $$CUR_DIR); } \
				done;\
				echo $$TOP_DIR)

$(info TOP_DIR = $(TOP_DIR))

include $(TOP_DIR)/modules-path.mk

all: trick-build

trick-build: modules_build
	cd exe/HIL/sdt_sample_code/; echo "I'm in exe/HIL/sdt_sample_code/"; \
	trick-CP
modules_build:
	$(TOP_DIR)/loop_build.sh 1 $(SIM_MODULES_PATH)
run:
	cd exe/HIL/sdt_sample_code/; echo "I'm in exe/HIL/sdt_sample_code/"; \
	./S_main_Linux_7_x86_64.exe RUN_test/input.cpp

clean:
	$(TOP_DIR)/loop_build.sh 0 $(SIM_MODULES_PATH)
	cd exe/HIL/sdt_sample_code/; echo "I'm in exe/HIL/sdt_sample_code/"; \
	make clean