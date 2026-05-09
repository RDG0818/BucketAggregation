cmake_minimum_required(VERSION 3.16)
project(BucketHeap VERSION 1.0) # PROJECT_NAME = BucketHeap, PROJECT_VERSION = 1.0

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(MSVC)
    add_compile_options(/O2 /Oi /Ot)
else()
    add_compile_options(-O3 -march=native -flto -g -fno-omit-frame-pointer)
endif()

add_subdirectory(external/cxxopts)


set(SRC_FILES 
src/grid_environment.cpp 
src/pancake_environment.cpp 
src/sliding_tile_environment.cpp 
src/msa_environment.cpp
)
add_library(core_lib ${SRC_FILES})

add_executable(main src/main.cpp)
target_link_libraries(main PRIVATE core_lib cxxopts::cxxopts) # Links main executable with files in core_lib

target_include_directories(core_lib PUBLIC include external/parallel_hashmap) # Adds include/ to the include path
