cmake_minimum_required(VERSION 2.6)
 
# Locate GTest
find_package(GTest REQUIRED)
#SET(BASEPATH "${CMAKE_SOURCE_DIR}")
include_directories("${GTEST_INCLUDE_DIRS}" "${CMAKE_SOURCE_DIR}/..")

# Link runTests with what we want to test and the GTest and pthread library
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "../build")
add_executable(runTests gtest.cpp)
target_link_libraries(runTests ${GTEST_LIBRARIES} pthread)