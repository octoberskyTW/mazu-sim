cmake_minimum_required(VERSION 3.5)

project(libactuator_dm VERSION 1.0.0 LANGUAGES CXX)

# Declare your module
add_library(actuator_dm STATIC
            src/rcs.cpp
            src/tvc.cpp
)

# Declare your flags: Don’t use TARGET_INCLUDE_DIRECTORIES() with a path outside your module
target_compile_options(actuator_dm PRIVATE -Wall -g -MD -save-temps -fPIC -Wextra -Wshadow -Wpedantic)
target_compile_features(actuator_dm PRIVATE cxx_std_17)
target_include_directories(actuator_dm
    ## public header for external module
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../math_utils>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../cadac>
)

# Declare your dependencies (in your other module)
target_link_libraries(actuator_dm INTERFACE arma::arma)
