
MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
CANNON_HOME = $(patsubst %/exe/HIL/sdt_sample_code/S_overrides.mk, %, $(MKFILE_PATH))

$(info MKFILE_PATH = $(MKFILE_PATH))
$(info CANNON_HOME = $(CANNON_HOME))

INCLUDES = ${CANNON_HOME}/module/hello_world
TRICK_USER_LINK_LIBS=-L${CANNON_HOME}/module/hello_world/ -lhello
TRICK_LDFLAGS=
TRICK_CFLAGS += ${INCLUDES} -g -D_GNU_SOURCE
TRICK_CFLAGS += -Wall -Wmissing-prototypes -Wextra -Wshadow
MAKEFLAGS += -j16
