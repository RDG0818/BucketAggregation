cmake_minimum_required(VERSION 3.16)
project(BucketHeap VERSION 1.0) # PROJECT_NAME = BucketHeap, PROJECT_VERSION = 1.0

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)

add_subdirectory(googletest)

file(GLOB SRC_FILES "src/environments.cpp")
add_library(core_lib ${SRC_FILES})

add_executable(main src/main.cpp)
target_link_libraries(main PRIVATE core_lib) # Links main executable with files in core_lib

target_include_directories(core_lib PUBLIC include) # Adds include/ to the include path

# Tests
enable_testing()
include(GoogleTest)

add_executable(tests tests/*.cpp)
target_link_libraries(tests PRIVATE core_lib gtest_main)
gtest_discover_tests(tests)

