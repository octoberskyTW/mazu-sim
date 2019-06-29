
MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
TOP_HOME = $(shell TOP_HOME=Unable_To_Find_Top_Dir; \
				CUR_DIR=$$(pwd); \
				while [ "$$CUR_DIR" != "/" ]; \
				do { \
					if [ -a $$CUR_DIR/.git ]; then \
					TOP_HOME=$$CUR_DIR; \
					fi; \
				    CUR_DIR=$$(dirname $$CUR_DIR); } \
				done;\
				echo $$TOP_HOME)

$(info MKFILE_PATH = $(MKFILE_PATH))
$(info TOP_HOME = $(TOP_HOME))

INCLUDES = ${TOP_HOME}/modules/hello_world
TRICK_USER_LINK_LIBS = -L${TOP_HOME}/modules/hello_world/ -lhello
TRICK_LDFLAGS =
TRICK_CFLAGS += ${INCLUDES} -g -D_GNU_SOURCE
TRICK_CFLAGS += -Wall -Wmissing-prototypes -Wextra -Wshadow
MAKEFLAGS += -j16
