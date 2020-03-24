cmake_minimum_required(VERSION 2.8.3)
project(avoidance_benchmark)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  sensor_msgs
  std_msgs
  cv_bridge
  gazebo_msgs
  geometry_msgs
  mavros_msgs
  trajectory_msgs
  nav_msgs
  tf
  std_srvs
  gazebo_dev
  camera_info_manager
)

find_package( OpenCV REQUIRED )
#find_package( gazebo REQUIRED)

set( CMAKE_CXX_STANDARD 11) #needed in the benchmark manager and in the wind plugin
set( CMAKE_EXPORT_COMPILE_COMMANDS ON ) #needed for Vim YouCompleteMe plugin
set( CMAKE_CXX_FLAGS_RELEASE -O3 -DNDEBUG )

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(benchmark_manager 
		src/benchmark_manager/test_node.cpp
		src/benchmark_manager/test.cpp
		src/benchmark_manager/util.cpp
		src/benchmark_manager/image_creation.cpp)
add_dependencies(benchmark_manager ${${PROJECT_NAME}_EXPORTED_TARGETS}${catkin_EXPORTED_TARGETS})
target_link_libraries(benchmark_manager ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_executable(dummy_avoidance_algorithm 
		src/dummy_avoidance_algorithm/node.cpp
		src/dummy_avoidance_algorithm/dummyAvoidance.cpp
		src/dummy_avoidance_algorithm/util.cpp)
add_dependencies(dummy_avoidance_algorithm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dummy_avoidance_algorithm ${catkin_LIBRARIES} )

add_executable(noisy_velodyne 
		src/noisy_velodyne/node.cpp
		src/noisy_velodyne/noisy_velodyne.cpp)
add_dependencies(noisy_velodyne ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(noisy_velodyne ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_executable(noisy_realsense 
		src/noisy_realsense/node.cpp
		src/noisy_realsense/noisy_realsense.cpp)
add_dependencies(noisy_realsense ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(noisy_realsense ${catkin_LIBRARIES} ${OpenCV_LIBS} )

# ============================================== #
# ============== REALSENSE PLUGIN ============== #
# ============================================== #

add_library(RealSensePlugin SHARED gazebo/gazebo_plugins/RealSensePlugin.cpp gazebo/gazebo_plugins/gazebo_ros_realsense.cpp)
target_link_libraries(RealSensePlugin ${catkin_LIBRARIES})

# ============================================== #
# =========== WORLD CREATION PLUGIN ============ #
# ============================================== #

add_library(gazebo_world_creation_plugin SHARED gazebo/gazebo_plugins/gazebo_world_creation_plugin.cc)
target_link_libraries(gazebo_world_creation_plugin
  ${catkin_LIBRARIES}
)


# ============================================== #
# ================= WIND PLUGIN ================ #
# ============================================== #
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

#add_library(boarr_gazebo_wind_plugin SHARED gazebo/gazebo_plugins/gazebo_wind_plugin.cc)
#target_link_libraries(boarr_gazebo_wind_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
#add_dependencies(boarr_gazebo_wind_plugin ${catkin_EXPORTED_TARGETS})
