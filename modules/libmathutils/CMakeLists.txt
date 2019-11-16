# Global Setup
cmake_minimum_required(VERSION 3.0)

project(libmathutils VERSION 1.0.0 LANGUAGES CXX)


# Declare your module
add_library(mathutils STATIC
            src/matrix_tool.cpp
            src/integrate.cpp
            src/stochastic.cpp
            src/nrutil.cpp
)



# Declare your flags: Don’t use TARGET_INCLUDE_DIRECTORIES() with a path outside your module
target_compile_options(mathutils PRIVATE -Wall -g -MD -save-temps -fPIC -Wextra -Wshadow -Wpedantic)
target_compile_features(mathutils PRIVATE cxx_std_17)
target_include_directories(mathutils
    ## public header for external module
    PUBLIC
        #$<INSTALL_INTERFACE:include>    
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    # Internal used
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Declare your dependencies (in your other module)

target_link_libraries(mathutils
    ## Public (interface) dependencies
    PUBLIC
    ## Private (implementation) dependencies
    PRIVATE
        #Boost::regex
)

# Header-only libraries
target_link_libraries(mathutils INTERFACE arma::arma)
## INTERFACE:Nothing to build so it must beINTERFACE
#add_library(mylib INTERFACE)
#target_include_directories(mylib INTERFACE include)
#target_link_libraries(mylib INTERFACE Boost::Boost)