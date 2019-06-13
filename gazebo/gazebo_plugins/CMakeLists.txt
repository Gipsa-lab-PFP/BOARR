cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(gazebo_world_creation_plugin SHARED gazebo_world_creation_plugin.cc)
target_link_libraries(gazebo_world_creation_plugin
  ${GAZEBO_LIBRARIES}
)
