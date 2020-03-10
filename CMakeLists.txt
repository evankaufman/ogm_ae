cmake_minimum_required(VERSION 2.8.3)
project(ogm_ae)

## Compile as C++11, Supported in ROS Kinetic & Newer
add_compile_options(-std=c++11)

## Packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  cmake_modules
  message_filters
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
  tf2_sensor_msgs
  sensor_msgs
  laser_geometry
  gazebo_msgs
)
find_package(trajectory_msgs)
find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)
find_package(gazebo REQUIRED)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()



# Directories
include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${OpenGL_INCLUDE_DIRS}
  ${GLUT_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

# Gazebo Flags
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


# Custom Messages
add_message_files(
  FILES
  UpdatedMapCells.msg
  PolyLeastSquaresTraj.msg
  ModelStateStamped.msg
  DesiredState.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  trajectory_msgs
  gazebo_msgs
)

# Libraries
catkin_package(
  CATKIN_DEPENDS message_runtime
  INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS} include
  LIBRARIES ${PCL_LIBRARIES} ogm_ae
)

add_library(robot src/Robot.cpp)
add_dependencies(robot ${ogm_ae_EXPORTED_TARGETS})

add_library(mapping src/Mapping.cpp)
add_dependencies(mapping robot)

add_library(exploration src/Exploration.cpp)
add_dependencies(exploration robot mapping)

# Mapping
add_executable(probabilistic_occupancy_grid_mapping src/probabilistic_occupancy_grid_mapping_node.cpp)
target_link_libraries(probabilistic_occupancy_grid_mapping robot mapping ${catkin_LIBRARIES})
add_dependencies(probabilistic_occupancy_grid_mapping ogm_ae_generate_messages_cpp)

# Exploration
add_executable(entropy_based_autonomous_exploration src/entropy_based_autonomous_exploration_node.cpp)
target_link_libraries(entropy_based_autonomous_exploration robot mapping exploration ${catkin_LIBRARIES})
add_dependencies(entropy_based_autonomous_exploration ogm_ae_generate_messages_cpp)

# Visualization
add_executable(generate_3D_rviz_msgs src/generate_3D_rviz_msgs_node.cpp)
add_dependencies(generate_3D_rviz_msgs ogm_ae_generate_messages_cpp)
target_link_libraries(generate_3D_rviz_msgs robot mapping ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_executable(robot_tf_publisher src/rot_tf_node.cpp)# moving viewpoint: 3rd-person follower of Rviz frame
target_link_libraries(robot_tf_publisher ${catkin_LIBRARIES})

# Robot Commands
add_executable(paths_to_cmd src/paths_to_cmd_node.cpp) 
target_link_libraries(paths_to_cmd robot mapping exploration ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(entropy_based_autonomous_exploration ogm_ae_generate_messages_cpp)

add_executable(wandering_robot src/wandering_robot_node.cpp) 
target_link_libraries(wandering_robot ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(wandering_robot ogm_ae_generate_messages_cpp)

add_executable(gazebo_communications src/gazebo_communication_node.cpp) 
target_link_libraries(gazebo_communications ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(gazebo_communications ogm_ae_generate_messages_cpp)

# Message Conversions
add_executable(sensor_conversion_node src/sensor_conversion.cpp)
target_link_libraries(sensor_conversion_node ${catkin_LIBRARIES})
add_executable(laser2pcl src/laser2pcl.cpp)
target_link_libraries(laser2pcl ${catkin_LIBRARIES})
add_executable(pcl_limit_range src/pcl_limit_range.cpp)
target_link_libraries(pcl_limit_range ${catkin_LIBRARIES})

## Install project namespaced headers (aux_fun.h for other package usage)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)


