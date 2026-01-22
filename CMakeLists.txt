cmake_minimum_required(VERSION 3.16)
project(BucketHeap VERSION 1.0) # PROJECT_NAME = BucketHeap, PROJECT_VERSION = 1.0

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)

add_subdirectory(googletest)

set(SRC_FILES 
src/grid_environment.cpp 
src/pancake_environment.cpp 
src/sliding_tile_environment.cpp 
src/binary_heap.cpp
src/bucket_queue.cpp)
add_library(core_lib ${SRC_FILES})

add_executable(main src/main.cpp)
target_link_libraries(main PRIVATE core_lib) # Links main executable with files in core_lib

target_include_directories(core_lib PUBLIC include) # Adds include/ to the include path

# Tests
enable_testing()
include(GoogleTest)

add_executable(tests 
tests/test_grid_environment.cpp 
tests/test_pancake_environment.cpp 
tests/test_sliding_tile_environment.cpp
tests/test_binary_heap.cpp
tests/test_bucket_queue.cpp
tests/test_a_star.cpp
tests/test_awa.cpp
)
target_link_libraries(tests PRIVATE core_lib gtest_main)
gtest_discover_tests(tests)

