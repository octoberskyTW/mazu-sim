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
           -I${TOP_HOME}/modules/actuator_dm/include \
           -I${TOP_HOME}/modules/cadac \
           -I${TOP_HOME}/modules/math_utils \
		   -I${TOP_HOME}/modules/gnc \
           -I${TOP_HOME}/models/skyline_input \
           -I${TOP_HOME}/models/dm_fsw_interface

TRICK_USER_LINK_LIBS += -L${TOP_HOME}/modules/dcmbus -ldcmbus \
						-L${TOP_HOME}/modules/ringbuffer -lringbuffer \
						-L${TOP_HOME}/modules/config_util -lconfig_util \
						-L${TOP_HOME}/modules/misc_utils -lmisc_utils \
						-L${TOP_HOME}/modules/dynamic_model -ldynamic_model \
						-L${TOP_HOME}/modules/sensor_dm -lsensor_dm \
						-L${TOP_HOME}/modules/actuator_dm/build -lactuator_dm \
						-L${TOP_HOME}/modules/cadac -lcadac \
						-L${TOP_HOME}/modules/math_utils -lmath_utils \
						-L${TOP_HOME}/modules/gnc -lgnc

TRICK_USER_LINK_LIBS += -larmadillo 

## During a simulation build, Trick generates several rounds of files to
## support data recording, checkpointing, and Python access:

## Trick generates S_source.hh from the S_define
## ICG recursively builds a tree of all header files included from S_source.hh and
## generates an io_*.cpp and py_*.i file for each.
## SWIG converts all py_*.i to py_*.cpp files
## Trick compiles all io_*.cpp and py_*.cpp files

# TRICK_EXCLUDE += ${TOP_HOME}/modules
## ICG will let variable could record by Trick
## It is possible to instruct ICG to skip entire directories
## using the environment variable TRICK_ICG_EXCLUDE.
## Set this variable to a colon separated list of directories
## which you wish ICG to bypass.
## This is useful when there is code which you do not wish Trick to have any knowledge of 
## (i.e. you don’t need any of the parameters recorded or input processable).
# TRICK_ICG_EXCLUDE += ${TOP_HOME}/modules
# TRICK_SWIG_EXCLUDE += ${TOP_HOME}/modules
TRICK_CXXFLAGS += ${INCLUDES} -std=c++11
TRICK_CFLAGS += ${INCLUDES} -std=c99
#TRICK_CFLAGS += -Wall -Wmissing-prototypes -Wextra -Wshadow
MAKEFLAGS += -j16
