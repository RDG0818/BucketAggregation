target_sources(${PROJECT_NAME}
PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}/scope_exit.hpp
${CMAKE_CURRENT_SOURCE_DIR}/detector.hpp
${CMAKE_CURRENT_SOURCE_DIR}/ordered_set.hpp
${CMAKE_CURRENT_SOURCE_DIR}/geometric_ordered_set.hpp
)

add_subdirectory(allocation)
add_subdirectory(parallel_hashmap)