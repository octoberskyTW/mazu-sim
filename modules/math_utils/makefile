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
#TARGET=libhello.so
TARGET=libmath_utils.a
# FLAGS
CXXFLAGS = -Wall -g -MD -save-temps -fPIC --std=c++17 -Wextra -Wshadow
CXXFLAGS += -I. -larmadillo
CFLAGS = -g -MD -save-temps -fPIC -std=c99 -pedantic -W -D _GNU_SOURCE
CFLAGS += -I.
LDFLAGS =

# Source code
CPP_SOURCES += $(TOP_DIR)/modules/math_utils/matrix_tool.cpp
CPP_SOURCES += $(TOP_DIR)/modules/math_utils/integrate.cpp
CPP_SOURCES += $(TOP_DIR)/modules/math_utils/stochastic.cpp \
               $(TOP_DIR)/modules/math_utils/nrutil.cpp
C_SOURCES =
OBJECTS += $(patsubst %.cpp, %.o, $(CPP_SOURCES))
OBJECTS += $(patsubst %.c, %.o, $(C_SOURCES))

all: $(TARGET) $(CPP_SOURCES) $(C_SOURCES)

%.o: %.cpp %.hh
	$(CXX) -c $(CXXFLAGS) -o $@ $<
%.o: %.c %.h
	$(CC) -c $(CFLAGS) -o $@ $<

$(TARGET): $(OBJECTS)
#	$(CC) $(LDFLAGS) -o $@ $<
	$(AR) rcs $@ $^

-include $(OBJECTS:.o=.d)

.PHONY : clean cmake

cmake:
	mkdir -p build
	cd build; echo $(PWD); \
	cmake ..
clean:
	find . -name "*.so" -type f -delete
	find . -name "*.a" -type f -delete
	find . -name "*.o" -type f -delete
	find . -name "*.i" -type f -delete
	find . -name "*.ii" -type f -delete
	find . -name "*.d" -type f -delete
	find . -name "*.s" -type f -delete
	rm -rf build
