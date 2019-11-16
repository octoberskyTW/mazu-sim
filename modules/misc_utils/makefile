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
TARGET=libmisc_utils.a
# FLAGS
CFLAGS = -Wall -g -MD -save-temps -fPIC
CFLAGS += -I.
LDFLAGS =

# Source code
C_SOURCES += $(TOP_DIR)/modules/misc_utils/misc_utils.c
OBJECTS += $(patsubst %.c, %.o, $(C_SOURCES))

all: $(TARGET) $(C_SOURCES)

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
	find . -name "*.d" -type f -delete
	find . -name "*.s" -type f -delete
	rm -rf build
