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

INCLUDES = -I${TOP_HOME}/modules/hello_world \
           -I${TOP_HOME}/modules/dcmbus \
           -I${TOP_HOME}/modules/ringbuffer \
           -I${TOP_HOME}/modules/config_util \
           -I${TOP_HOME}/modules/misc_utils
TRICK_USER_LINK_LIBS = -L${TOP_HOME}/modules/hello_world -lhello
TRICK_USER_LINK_LIBS += -L${TOP_HOME}/modules/dcmbus -ldcmbus
TRICK_USER_LINK_LIBS += -L${TOP_HOME}/modules/ringbuffer -lringbuffer
TRICK_USER_LINK_LIBS += -L${TOP_HOME}/modules/config_util -lconfig_util
TRICK_USER_LINK_LIBS += -L${TOP_HOME}/modules/misc_utils -lmisc_utils
TRICK_CXXFLAGS += ${INCLUDES}
TRICK_CFLAGS += ${INCLUDES}
#TRICK_CFLAGS += -Wall -Wmissing-prototypes -Wextra -Wshadow
MAKEFLAGS += -j16
