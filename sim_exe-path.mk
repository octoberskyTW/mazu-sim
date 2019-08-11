
ifeq ($(project), $(filter $(project), sample_code all))
SIM_EXE_TRICK_PATH+=$(TOP_DIR)/sim_exe/sample_code/
SIM_EXE_TRICK_PATH+=$(TOP_DIR)/sim_exe/sample_client/
endif

ifeq ($(project), $(filter $(project), egse_dm all))
SIM_EXE_TRICK_PATH+=$(TOP_DIR)/sim_exe/egse_dm/
SIM_EXE_TRICK_PATH+=$(TOP_DIR)/sim_exe/fsw_gnc/
endif
$(info SIM_EXE_TRICK_PATH = $(SIM_EXE_TRICK_PATH))