include modules-path_var.mk
SIM_MODULES_PATH+=$(module_dcmbus)
SIM_MODULES_PATH+=$(module_hello_world)
SIM_MODULES_PATH+=$(module_ringbuffer)
SIM_MODULES_PATH+=$(module_config_util)
SIM_MODULES_PATH+=$(module_misc_utils)
SIM_MODULES_PATH+=$(module_math_utils)
SIM_MODULES_PATH+=$(module_dynamic_model)
SIM_MODULES_PATH+=$(module_cadac)
SIM_MODULES_PATH+=$(module_sensor_dm)
SIM_MODULES_PATH+=$(module_actuator_dm)
SIM_MODULES_PATH+=$(module_gnc)
SIM_MODULES_PATH+=$(module_navigation)
$(info SIM_MODULES_PATH = $(SIM_MODULES_PATH))
