cmake_minimum_required(VERSION 3.10)

# set the project name
project(math_utils VERSION 1.0)

# execute_process(COMMAND bash
#                 TOP_DIR=$(shell TOP_DIR=Unable_To_Find_Top_Dir; \
#                 CUR_DIR=$$(pwd); \
#                 while [ "$$CUR_DIR" != "/" ]; \
#                 do { \
#                     if [ -a $$CUR_DIR/BOBI ]; then \
#                     TOP_DIR=$$CUR_DIR; \
#                     fi; \
#                     CUR_DIR=$$(dirname $$CUR_DIR); } \
#                 done;\
#                 echo $$TOP_DIR)
#                 OUTPUT_VARIABLE TOP_DIR
#                 )


set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CPP_SOURCES matrix_tool.cpp integrate.cpp stochastic.cpp nrutil.cpp)
add_library(math_utils STATIC ${CPP_SOURCES})


message(STATUS "${CMAKE_BINARY_DIR}")
set_target_properties(math_utils PROPERTIES
    OUTPUT_NAME "math_utils"
    PREFIX      "lib"
    )
# specify the C standard
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED True)
set(${CMAKE_CXX_FLAGS} -Wall -MD -save-temps -fPIC --std=c++17 -Wextra -Wshadow)
set(${CMAKE_CXX_FLAGS} -I. -larmadillo)
# # add the executable



