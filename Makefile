
SHELL=/bin/bash
TOP_DIR=$(shell TOP_DIR=Unable_To_Find_Top_Dir; \
				CUR_DIR=$$(pwd); \
				while [ "$$CUR_DIR" != "/" ]; \
				do { \
					if [ -a $$CUR_DIR/BOBI ]; then \
					TOP_DIR=$$CUR_DIR; \
					fi; \
				    CUR_DIR=$$(dirname $$CUR_DIR); } \
				done;\
				echo $$TOP_DIR)

$(info TOP_DIR = $(TOP_DIR))

GIT_HOOK := .git/hooks/applied
$(GIT_HOOK): scripts/install-git-hooks
	@$<
	@echo

project ?= all

include $(TOP_DIR)/modules-path.mk
include $(TOP_DIR)/sim_exe-path.mk

all: trick_build $(GIT_HOOK)
.DEFAULT_GOAL := all
trick_build: modules_build
	$(TOP_DIR)/loop_build.sh $(TOP_DIR) trick_build $(SIM_EXE_TRICK_PATH)
modules_build:
	$(TOP_DIR)/loop_build.sh $(TOP_DIR) module_build $(SIM_MODULES_PATH)

run-sample_code:
	./run_sim.sh $(TOP_DIR) sample_code

run-egse_dm:
	./run_sim.sh $(TOP_DIR) egse_dm

clean:
	$(TOP_DIR)/loop_build.sh $(TOP_DIR) module_clean $(SIM_MODULES_PATH)
	$(TOP_DIR)/loop_build.sh $(TOP_DIR) trick_clean $(SIM_EXE_TRICK_PATH)
	