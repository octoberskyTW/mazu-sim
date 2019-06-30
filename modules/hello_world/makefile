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
TARGET=libhello.so

# FLAGS
CFLAGS = -Wall -g -MD -save-temps -fPIC
CFLAGS += -I.
LDFLAGS = -shared

# Source code
C_SOURCES += $(TOP_DIR)/modules/hello_world/hello.c
OBJECTS += $(patsubst %.c, %.o, $(C_SOURCES))

all: $(TARGET) $(C_SOURCES)

%.o: %.c %.h
	$(CC) -c $(CFLAGS) -o $@ $<

$(TARGET): $(OBJECTS)
	$(CC) $(LDFLAGS) -o $@ $<

-include $(OBJECTS:.o=.d)

.PHONY : clean
clean:
	find . -name "*.so" -type f -delete
	find . -name "*.o" -type f -delete
	find . -name "*.i" -type f -delete
	find . -name "*.d" -type f -delete
	find . -name "*.s" -type f -delete