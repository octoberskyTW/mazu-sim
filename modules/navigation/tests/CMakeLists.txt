# Global Setup
cmake_minimum_required(VERSION 3.0)

project(runTests VERSION 1.0.0)


######### Libary Build #########
# Declare your module
add_library(libnavigation STATIC
            ${CMAKE_SOURCE_DIR}/../ins.cpp
            ${CMAKE_SOURCE_DIR}/../sensor_calibration.cpp
)

add_library(libmath_utils STATIC
            ${CMAKE_SOURCE_DIR}/../../math_utils/matrix_tool.cpp
            ${CMAKE_SOURCE_DIR}/../../math_utils/integrate.cpp
            ${CMAKE_SOURCE_DIR}/../../math_utils/stochastic.cpp
            ${CMAKE_SOURCE_DIR}/../../math_utils/nrutil.cpp
)

add_library(libmisc_utils STATIC
            ${CMAKE_SOURCE_DIR}/../../misc_utils/misc_utils.c
)


# -I/foo/bar
target_include_directories(libmath_utils
    ## public header for external module
    PUBLIC
        #$<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/../../math_utils>
    # Internal used
    PRIVATE
        
)

# -I/foo/bar
target_include_directories(libmisc_utils
    ## public header for external module
    PUBLIC
        #$<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/../../misc_utils>
    # Internal used
    PRIVATE
        #${CMAKE_CURRENT_SOURCE_DIR}
)
# -I/foo/bar
target_include_directories(libnavigation
    ## public header for external module
    PUBLIC
        #$<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/..>
    # Internal used
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# Declare your flags: Don’t use TARGET_INCLUDE_DIRECTORIES() with a path outside your module
target_compile_options(libnavigation PRIVATE -Wall -g -MD -save-temps -fPIC)
target_compile_features(libnavigation PRIVATE cxx_std_17)

target_compile_options(libmath_utils PRIVATE -Wall -g -MD -save-temps -fPIC)
target_compile_features(libmath_utils PRIVATE cxx_std_17)

target_compile_options(libmisc_utils PRIVATE -Wall -g -MD -save-temps -fPIC)
target_compile_features(libmisc_utils PRIVATE cxx_std_17)

target_link_libraries(libmath_utils
    ## Public (interface) dependencies
    PUBLIC
        -larmadillo
    ## Private (implementation) dependencies
    PRIVATE

)

target_link_libraries(libnavigation
    ## Public (interface) dependencies
    PUBLIC
        libmath_utils
        libmisc_utils
    ## Private (implementation) dependencies
    PRIVATE

)


######### Executable Build #########
# Declare your module
add_executable(runTests 
    gtest.cpp
    navigation_tests.cpp
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
        libnavigation
)

