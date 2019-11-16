cmake_minimum_required(VERSION 3.10)

# set the project name
project(misc_utils VERSION 1.0)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

add_library(misc_utils STATIC misc_utils.c)
set_target_properties(misc_utils PROPERTIES
    OUTPUT_NAME "misc_utils"
    PREFIX      "lib"
    )
# specify the C standard
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED True)
set(${CMAKE_C_FLAGS} -Wall -MD -save-temps -fPIC)
set(${CMAKE_C_FLAGS} -I.)

# # add the executable


