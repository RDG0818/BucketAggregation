# minimum required CMAKE version
CMAKE_MINIMUM_REQUIRED(VERSION 3.13 FATAL_ERROR)

project(HeuristicSearch)

set(CMAKE_CXX_STANDARD 23)

add_executable(${PROJECT_NAME})

include(CheckCXXCompilerFlag)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    check_cxx_compiler_flag("-mssse3" SUPPORTS_SSSE3)
endif()

if(SUPPORTS_SSSE3)
    target_compile_options(${PROJECT_NAME} PUBLIC "-mssse3")
else()
    message(WARNING "SSSE3 is not supported by the compiler.")
endif()

find_package(TBB REQUIRED)

add_definitions(-DCMAKE_EXPORT_COMPILE_COMMANDS=ON)

add_subdirectory(src)

target_include_directories(${PROJECT_NAME} PUBLIC src)
target_link_libraries(${PROJECT_NAME} PRIVATE TBB::tbb)