# Global Setup
cmake_minimum_required(VERSION 3.0)

project(runTests VERSION 1.0.0)


######### Libary Build #########
# Declare your module
add_library(libringbuffer STATIC
            ${CMAKE_SOURCE_DIR}/../ringbuffer.c
)
# -I/foo/bar
target_include_directories(libringbuffer
    ## public header for external module
    PUBLIC
        #$<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/..>
    # Internal used
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# Declare your flags: Don’t use TARGET_INCLUDE_DIRECTORIES() with a path outside your module
target_compile_options(libringbuffer PRIVATE -Wall -g -MD -save-temps -fPIC)
target_compile_features(libringbuffer PRIVATE cxx_std_17)

######### Executable Build #########
# Declare your module
add_executable(runTests 
    gtest.cpp
    ringbuffer_tests.cpp
)


find_package(GTest REQUIRED)

# Compile options
target_compile_options(runTests PRIVATE -Wall -g -MD -save-temps)
target_compile_features(runTests PRIVATE cxx_std_17)

# Declare your flags: Don’t use TARGET_INCLUDE_DIRECTORIES() with a path outside your module
# -I/foo/bar
target_include_directories(runTests
    ## public header for external module
    PUBLIC
        #$<INSTALL_INTERFACE:include> 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
    # Internal used
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# Declare your dependencies, -l/path/to/lib
target_link_libraries(runTests
    ## Public (interface) dependencies
    PUBLIC
    ## Private (implementation) dependencies
    PRIVATE
        GTest::GTest
        GTest::Main
        pthread
        libringbuffer
)

