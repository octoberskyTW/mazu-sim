include modules-path_var.mk
SIM_MODULES_PATH+=$(module_dcm_bus)
SIM_MODULES_PATH+=$(module_hello_world)
SIM_MODULES_PATH+=$(module_ringbuffer)
$(info SIM_MODULES_PATH = $(SIM_MODULES_PATH))
