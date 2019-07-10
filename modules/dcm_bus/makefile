SHELL=/bin/bash
TOP_DIR=$(shell TOP_DIR=Unable_To_Find_Top_Dir; \
				CUR_DIR=$$(pwd); \
				while [ "$$CUR_DIR" != "/" ]; \
				do { \
					if [ -a $$CUR_DIR/.git ]; then \
					TOP_DIR=$$CUR_DIR; \
					fi; \
				    CUR_DIR=$$(dirname $$CUR_DIR); } \
				done;\
				echo $$TOP_DIR)
TARGET=libdcm_bus.a
include $(TOP_DIR)/modules-path_var.mk
# FLAGS
CXXFLAGS = -Wall -g -MD -save-temps
CXXFLAGS += -I.
CXXFLAGS += -I$(module_ringbuffer)
LDFLAGS =
# Source code
CPP_SOURCES += $(TOP_DIR)/modules/dcm_bus/dcm_bus.cpp

OBJECTS += $(patsubst %.cpp, %.o, $(CPP_SOURCES))

all: $(TARGET) $(CPP_SOURCES)

%.o: %.cpp %.hh
	$(CXX) -c $(CXXFLAGS) -o $@ $<

$(TARGET): $(OBJECTS)
	$(AR) rcs $@ $^

-include $(OBJECTS:.o=.d)

test: $(TARGET) $(CPP_SOURCES)
	mkdir -p build
	cd build; cmake -S $(TOP_DIR)/modules/dcm_bus/tests/ ;\
	make
	./build/runTests

#	g++ -Wall -g tests/tests_ringbuffer.cpp  $< -I. -o $@
#	./test
.PHONY : clean
clean:
	find . -name "*.so" -type f -delete
	find . -name "*.a" -type f -delete
	find . -name "*.o" -type f -delete
	find . -name "*.i" -type f -delete
	find . -name "*.ii" -type f -delete
	find . -name "*.d" -type f -delete
	find . -name "*.s" -type f -delete
	rm -rf build