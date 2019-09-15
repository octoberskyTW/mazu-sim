
ifeq ($(project), $(filter $(project), sample_master all))
SIM_EXE_TRICK_PATH+=$(TOP_DIR)/sim_exe/sample_master/
SIM_EXE_TRICK_PATH+=$(TOP_DIR)/sim_exe/sample_slave/
endif

ifeq ($(project), $(filter $(project), skyline_dm all))
SIM_EXE_TRICK_PATH+=$(TOP_DIR)/sim_exe/skyline_dm/
SIM_EXE_TRICK_PATH+=$(TOP_DIR)/sim_exe/skyline_gnc/
endif
$(info SIM_EXE_TRICK_PATH = $(SIM_EXE_TRICK_PATH))