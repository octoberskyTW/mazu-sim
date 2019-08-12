include modules-path_var.mk
SIM_MODULES_PATH+=$(module_dcmbus)
SIM_MODULES_PATH+=$(module_hello_world)
SIM_MODULES_PATH+=$(module_ringbuffer)
SIM_MODULES_PATH+=$(module_config_util)
SIM_MODULES_PATH+=$(module_misc_utils)
SIM_MODULES_PATH+=$(module_math_tools)
SIM_MODULES_PATH+=$(module_dynamic_model)
$(info SIM_MODULES_PATH = $(SIM_MODULES_PATH))
