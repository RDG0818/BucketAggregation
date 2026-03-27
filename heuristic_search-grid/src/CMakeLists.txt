target_sources(${PROJECT_NAME}
PRIVATE
${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
)

add_subdirectory(search)
add_subdirectory(domains)
add_subdirectory(priority_queues)
add_subdirectory(utils)