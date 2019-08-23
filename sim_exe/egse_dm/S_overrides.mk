SHELL=/bin/bash
TOP_HOME = $(shell TOP_HOME=Unable_To_Find_Top_Dir; \
				CUR_DIR=$$(pwd); \
				while [ "$$CUR_DIR" != "/" ]; \
				do { \
					if [ -a $$CUR_DIR/BOBI ]; then \
					TOP_HOME=$$CUR_DIR; \
					fi; \
				    CUR_DIR=$$(dirname $$CUR_DIR); } \
				done;\
				echo $$TOP_HOME)

$(info TOP_HOME = $(TOP_HOME))

INCLUDES = -I${TOP_HOME}/modules/dcmbus \
           -I${TOP_HOME}/modules/ringbuffer \
           -I${TOP_HOME}/modules/config_util \
           -I${TOP_HOME}/modules/misc_utils \
           -I${TOP_HOME}/modules/dynamic_model \
           -I${TOP_HOME}/modules/sensor_dm \
           -I${TOP_HOME}/modules/actuator_dm \
           -I${TOP_HOME}/modules/cadac \
           -I${TOP_HOME}/modules/math_utils

TRICK_USER_LINK_LIBS += -L${TOP_HOME}/modules/dcmbus -ldcmbus \
						-L${TOP_HOME}/modules/ringbuffer -lringbuffer \
						-L${TOP_HOME}/modules/config_util -lconfig_util \
						-L${TOP_HOME}/modules/misc_utils -lmisc_utils \
						-L${TOP_HOME}/modules/dynamic_model -ldynamic_model \
						-L${TOP_HOME}/modules/sensor_dm -lsensor_dm \
						-L${TOP_HOME}/modules/actuator_dm -lactuator_dm \
						-L${TOP_HOME}/modules/cadac -lcadac \
						-L${TOP_HOME}/modules/math_utils -lmath_utils
TRICK_USER_LINK_LIBS += -larmadillo 

TRICK_EXCLUDE += ${TOP_HOME}/modules
#TRICK_ICG_EXCLUDE += ${TOP_HOME}/modules
TRICK_CXXFLAGS += ${INCLUDES}
TRICK_CFLAGS += ${INCLUDES}
#TRICK_CFLAGS += -Wall -Wmissing-prototypes -Wextra -Wshadow
MAKEFLAGS += -j16
