cmake_minimum_required(VERSION 3.5)

project(libcadac VERSION 1.0.0 LANGUAGES CXX)

# Declare your module
add_library(cadac STATIC
            src/datadeck.cpp
            src/cadac_util.cpp
            src/schmitt_trigger.cpp
            src/env/wind.cpp
            src/env/wind_constant.cpp
            src/env/wind_no.cpp
            src/env/wind_tabular.cpp
            src/env/atmosphere76.cpp
            src/env/atmosphere_nasa2002.cpp
            src/env/atmosphere_weatherdeck.cpp
            src/component.cpp
            src/signal_process.cpp
            src/vehicle_var.cpp
            src/vehicle.cpp
            src/time_management.cpp
            src/time_util.cpp
)

# Declare your flags: Don’t use TARGET_INCLUDE_DIRECTORIES() with a path outside your module
target_compile_options(cadac PRIVATE -Wall -g -MD -save-temps -fPIC -Wextra -Wshadow -Wpedantic)
target_compile_features(cadac PRIVATE cxx_std_17)
target_include_directories(cadac
    ## public header for external module
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../math_utils>
)

# Declare your dependencies (in your other module)
target_link_libraries(cadac INTERFACE arma::arma)
