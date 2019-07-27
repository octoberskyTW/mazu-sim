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
include $(TOP_DIR)/sim_exe-path.mk

all: trick-build

trick-build: modules_build
	$(TOP_DIR)/loop_build.sh $(TOP_DIR) trick-build $(SIM_EXE_TRICK_PATH)
modules_build:
	$(TOP_DIR)/loop_build.sh $(TOP_DIR) module-build $(SIM_MODULES_PATH)

run-sample_code:
	ln -sf $(TOP_DIR)/conf/dcmbus/dcm_channel.cfg $(TOP_DIR)/sim_exe/sample_code/dcm_channel.cfg
	ln -sf $(TOP_DIR)/conf/dcmbus/dcm_ring.cfg $(TOP_DIR)/sim_exe/sample_code/dcm_ring.cfg
	ln -sf $(TOP_DIR)/conf/dcmbus/dcm_bind.cfg $(TOP_DIR)/sim_exe/sample_code/dcm_bind.cfg
	ln -sf $(TOP_DIR)/conf/dcmbus/dcm_client_ring.cfg $(TOP_DIR)/sim_exe/sample_client/dcm_client_ring.cfg
	ln -sf $(TOP_DIR)/conf/dcmbus/dcm_client_channel.cfg $(TOP_DIR)/sim_exe/sample_client/dcm_client_channel.cfg
	ln -sf $(TOP_DIR)/conf/dcmbus/dcm_client_bind.cfg $(TOP_DIR)/sim_exe/sample_client/dcm_client_bind.cfg
	./run_sim.sh $(TOP_DIR)

clean:
	rm -f $(TOP_DIR)/sim_exe/sample_code/*.cfg
	rm -f $(TOP_DIR)/sim_exe/sample_client/*.cfg
	$(TOP_DIR)/loop_build.sh $(TOP_DIR) module-clean $(SIM_MODULES_PATH)
	$(TOP_DIR)/loop_build.sh $(TOP_DIR) trick-clean $(SIM_EXE_TRICK_PATH)
	