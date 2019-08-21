SHELL=/bin/bash
TOP_DIR=$(shell TOP_DIR=Unable_To_Find_Top_Dir; \
				CUR_DIR=$$(pwd); \
				while [ "$$CUR_DIR" != "/" ]; \
				do { \
					if [ -a $$CUR_DIR/BOBI ]; then \
					TOP_DIR=$$CUR_DIR; \
					fi; \
				    CUR_DIR=$$(dirname $$CUR_DIR); } \
				done;\
				echo $$TOP_DIR)
include $(TOP_DIR)/modules-path_var.mk
#TARGET=libhello.so
TARGET=libdynamic_model.a
# FLAGS
CXXFLAGS = -Wall -g -MD -save-temps -fPIC --std=c++17 -Wextra -Wshadow
CXXFLAGS += -I. -larmadillo
CXXFLAGS += -I$(module_math_utils) \
            -I$(module_cadac)
LDFLAGS =

# Source code
CPP_SOURCES += $(TOP_DIR)/modules/dynamic_model/component.cpp \
               $(TOP_DIR)/modules/dynamic_model/signal_process.cpp \
               $(TOP_DIR)/modules/dynamic_model/vehicle_var.cpp \
               $(TOP_DIR)/modules/dynamic_model/vehicle.cpp \
               $(TOP_DIR)/modules/dynamic_model/time_util.cpp \
               $(TOP_DIR)/modules/dynamic_model/time_management.cpp \
               $(TOP_DIR)/modules/dynamic_model/propulsion.cpp \
               $(TOP_DIR)/modules/dynamic_model/tvc.cpp \
               $(TOP_DIR)/modules/dynamic_model/rocket_flight_dm.cpp \
               $(TOP_DIR)/modules/dynamic_model/rcs.cpp \
               $(TOP_DIR)/modules/dynamic_model/earth_environment.cpp \
               $(TOP_DIR)/modules/dynamic_model/aerodynamics.cpp
OBJECTS += $(patsubst %.cpp, %.o, $(CPP_SOURCES))

all: $(TARGET) $(CPP_SOURCES)

%.o: %.cpp %.h
	$(CXX) -c $(CXXFLAGS) -o $@ $<

$(TARGET): $(OBJECTS)
#	$(CC) $(LDFLAGS) -o $@ $<
	$(AR) rcs $@ $<

-include $(OBJECTS:.o=.d)

.PHONY : clean
clean:
	find . -name "*.so" -type f -delete
	find . -name "*.a" -type f -delete
	find . -name "*.o" -type f -delete
	find . -name "*.ii" -type f -delete
	find . -name "*.d" -type f -delete
	find . -name "*.s" -type f -delete
	rm -rf build