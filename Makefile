
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

sample_code_cfg:
	ln -sf $(TOP_DIR)/conf/dcmbus/dcm_channel.cfg $(TOP_DIR)/sim_exe/sample_code/dcm_channel.cfg
	ln -sf $(TOP_DIR)/conf/dcmbus/dcm_ring.cfg $(TOP_DIR)/sim_exe/sample_code/dcm_ring.cfg
	ln -sf $(TOP_DIR)/conf/dcmbus/dcm_bind.cfg $(TOP_DIR)/sim_exe/sample_code/dcm_bind.cfg
	ln -sf $(TOP_DIR)/conf/dcmbus/dcm_client_ring.cfg $(TOP_DIR)/sim_exe/sample_client/dcm_client_ring.cfg
	ln -sf $(TOP_DIR)/conf/dcmbus/dcm_client_channel.cfg $(TOP_DIR)/sim_exe/sample_client/dcm_client_channel.cfg
	ln -sf $(TOP_DIR)/conf/dcmbus/dcm_client_bind.cfg $(TOP_DIR)/sim_exe/sample_client/dcm_client_bind.cfg

run-sample_code: sample_code_cfg
	./run_sim.sh $(TOP_DIR) sample_code

egse_dm_cfg: 
	ln -sf $(TOP_DIR)/conf/dcmbus/dcm_channel.cfg $(TOP_DIR)/sim_exe/egse_dm/dcm_channel.cfg
	ln -sf $(TOP_DIR)/conf/dcmbus/dcm_ring.cfg $(TOP_DIR)/sim_exe/egse_dm/dcm_ring.cfg
	ln -sf $(TOP_DIR)/conf/dcmbus/dcm_bind.cfg $(TOP_DIR)/sim_exe/egse_dm/dcm_bind.cfg
	ln -sf $(TOP_DIR)/conf/dcmbus/dcm_client_ring.cfg $(TOP_DIR)/sim_exe/fsw_gnc/dcm_client_ring.cfg
	ln -sf $(TOP_DIR)/conf/dcmbus/dcm_client_channel.cfg $(TOP_DIR)/sim_exe/fsw_gnc/dcm_client_channel.cfg
	ln -sf $(TOP_DIR)/conf/dcmbus/dcm_client_bind.cfg $(TOP_DIR)/sim_exe/fsw_gnc/dcm_client_bind.cfg

run-egse_dm: egse_dm_cfg
	./run_sim.sh $(TOP_DIR) egse_dm

clean:
	rm -f $(TOP_DIR)/sim_exe/sample_code/*.cfg
	rm -f $(TOP_DIR)/sim_exe/sample_client/*.cfg
	rm -f $(TOP_DIR)/sim_exe/egse_dm/*.cfg
	rm -f $(TOP_DIR)/sim_exe/fsw_gnc/*.cfg
	$(TOP_DIR)/loop_build.sh $(TOP_DIR) module_clean $(SIM_MODULES_PATH)
	$(TOP_DIR)/loop_build.sh $(TOP_DIR) trick_clean $(SIM_EXE_TRICK_PATH)
	