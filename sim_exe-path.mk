ifeq ($(project), $(filter $(project), sheipa all))
SIM_EXE_TRICK_PATH+=$(TOP_DIR)/sim_exe/sheipa_dm/
SIM_EXE_TRICK_PATH+=$(TOP_DIR)/sim_exe/sheipa_gnc/
endif
$(info SIM_EXE_TRICK_PATH = $(SIM_EXE_TRICK_PATH))