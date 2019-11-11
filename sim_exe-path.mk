
ifeq ($(project), $(filter $(project), sample all))
SIM_EXE_TRICK_PATH+=$(TOP_DIR)/sim_exe/sample_master/
SIM_EXE_TRICK_PATH+=$(TOP_DIR)/sim_exe/sample_slave/
endif

ifeq ($(project), $(filter $(project), skyline all))
SIM_EXE_TRICK_PATH+=$(TOP_DIR)/sim_exe/skyline_dm/
SIM_EXE_TRICK_PATH+=$(TOP_DIR)/sim_exe/skyline_gnc/
endif

ifeq ($(project), $(filter $(project), sheipa all))
SIM_EXE_TRICK_PATH+=$(TOP_DIR)/sim_exe/sheipa_dm/
SIM_EXE_TRICK_PATH+=$(TOP_DIR)/sim_exe/sheipa_gnc/
endif
$(info SIM_EXE_TRICK_PATH = $(SIM_EXE_TRICK_PATH))