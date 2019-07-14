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
TARGET=libdcmbus.a
include $(TOP_DIR)/modules-path_var.mk
# FLAGS
CFLAGS = -Wall -g -MD -save-temps -fPIC -std=c99 -pedantic -W -Werror
CFLAGS += -I.
CFLAGS += -I$(module_ringbuffer)
LDFLAGS =
# Source code
C_SOURCES += $(TOP_DIR)/modules/dcmbus/dcmbus.c \
             $(TOP_DIR)/modules/dcmbus/ethernet.c


OBJECTS += $(patsubst %.c, %.o, $(C_SOURCES))

all: $(TARGET) $(C_SOURCES)

%.o: %.c %.h
	$(CC) -c $(CFLAGS) -o $@ $<

$(TARGET): $(OBJECTS)
	$(AR) rcs $@ $^

-include $(OBJECTS:.o=.d)

test: $(TARGET) $(C_SOURCES)
	mkdir -p build
	cd build; cmake -S $(TOP_DIR)/modules/dcmbus/tests/ ;\
	make
	./build/runTests

#	g++ -Wall -g tests/tests_ringbuffer.c  $< -I. -o $@
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