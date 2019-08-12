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
TARGET=libaero_dynamic.a
# FLAGS
CXXFLAGS = -Wall -g -MD -save-temps -fPIC
CXXFLAGS += -I.
LDFLAGS =

# Source code
CPP_SOURCES += $(TOP_DIR)/modules/aero_dynamic/Tvc.cpp
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
	find . -name "*.i" -type f -delete
	find . -name "*.d" -type f -delete
	find . -name "*.s" -type f -delete
	rm -rf build