cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(gazebo_world_creation_plugin SHARED gazebo_world_creation_plugin.cc)
target_link_libraries(gazebo_world_creation_plugin
  ${GAZEBO_LIBRARIES}
)



# wind plugin ( from rotors )

 
set(targets_to_install "")

add_library(roast_gazebo_wind_plugin SHARED gazebo_wind_plugin.cc)
target_link_libraries(roast_gazebo_wind_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
list(APPEND targets_to_install roast_gazebo_wind_plugin)


set(BIN_DESTINATION ${CMAKE_BINARY_DIR})
set(LIB_DESTINATION ${CMAKE_BINARY_DIR})

install(
  TARGETS
    ${targets_to_install}
  DESTINATION ${BIN_DESTINATION}
  LIBRARY DESTINATION ${LIB_DESTINATION}
)


/localdata/tezenast/Documents/ROS/rotors/devel/.private/mav_msgs/include;
/localdata/tezenast/Documents/ROS/rotors/devel/.private/rotors_comm/include;

/localdata/tezenast/Documents/ROS/rotors/src/mav_comm/mav_msgs/include;
/localdata/tezenast/Documents/ROS/rotors/src/rotors_simulator/rotors_control/include;
/localdata/tezenast/Documents/ROS/rotors/src/rotors_simulator/rotors_gazebo_plugins/include;
/localdata/tezenast/Documents/ROS/lib_workspace/src/vision_opencv/cv_bridge/include;

/opt/ros/kinetic/include;
/opt/ros/kinetic/share/xmlrpcpp/cmake/../../../include/xmlrpcpp;

/usr/include;

/usr/local/include/eigen3

