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
)
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

# Custom Messages
add_message_files(
  FILES
  UpdatedMapCells.msg
  PolyLeastSquaresTraj.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

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

# Libraries
catkin_package(
  CATKIN_DEPENDS message_runtime
  INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES pogm
  LIBRARIES ebae
  LIBRARIES voxel
)

# Shared Library Among Package Nodes
add_library(voxel src/voxel.cpp)
add_dependencies(voxel ${ogm_ae_EXPORTED_TARGETS})

# Mapping
add_library(pogm src/ProbabilisticOccupancyGridMapping.cpp)
add_executable(probabilistic_occupancy_grid_mapping src/probabilistic_occupancy_grid_mapping_node.cpp)
target_link_libraries(probabilistic_occupancy_grid_mapping pogm voxel ${catkin_LIBRARIES})

# Exploration
add_library(ebae src/EntropyBasedAutonomousExploration.cpp)
add_executable(entropy_based_autonomous_exploration src/entropy_based_autonomous_exploration_node.cpp)
target_link_libraries(entropy_based_autonomous_exploration ebae voxel ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# Map Reduction
add_executable(generate_exploration_map src/generate_exploration_map_node.cpp)
target_link_libraries(generate_exploration_map voxel ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# Visualization
add_executable(generate_3D_rviz_msgs src/generate_3D_rviz_msgs_node.cpp)
target_link_libraries(generate_3D_rviz_msgs voxel ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_executable(robot_tf_publisher src/rot_tf_node.cpp)# fix Rviz frame for rotated view
target_link_libraries(robot_tf_publisher ${catkin_LIBRARIES})

# Robot Commands
add_executable(paths_to_cmd src/paths_to_cmd_node.cpp) 
target_link_libraries(paths_to_cmd voxel ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}) 

# Message Conversions
add_executable(sensor_conversion_node src/sensor_conversion.cpp)
target_link_libraries(sensor_conversion_node ${catkin_LIBRARIES})
add_executable(laser2pcl src/laser2pcl.cpp)
target_link_libraries(laser2pcl ${catkin_LIBRARIES})
add_executable(pcl_limit_range src/pcl_limit_range.cpp)
target_link_libraries(pcl_limit_range ${catkin_LIBRARIES})

