cmake_minimum_required(VERSION 3.0.2)
project(semantic_occupancy_mapping_3d)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

#SET OCTOMAP_OMP to disable/enable OPENMP (experimental for octomap, default disabled)
SET(OCTOMAP_OMP FALSE CACHE BOOL "Enable/disable OpenMP parallelization")
IF(DEFINED ENV{OCTOMAP_OMP})
  SET(OCTOMAP_OMP $ENV{OCTOMAP_OMP})
ENDIF(DEFINED ENV{OCTOMAP_OMP})
IF(OCTOMAP_OMP)
  FIND_PACKAGE( OpenMP REQUIRED)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
ENDIF(OCTOMAP_OMP)


list(APPEND CMAKE_MODULE_PATH ${CMAKE_PREFIX_PATH})
# Verify
message(STATUS "CMAKE_MODULE_PATH=${CMAKE_MODULE_PATH}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  visualization_msgs
  message_generation
  std_msgs
  tf
  kdtree
  cmake_modules
  rospy
  pcl_ros
  sensor_msgs
  tf
  tf_conversions
  eigen_conversions
  nav_msgs
  rviz_visual_tools
  pcl_conversions
  std_srvs
  octomap_msgs
  minkindr
  minkindr_conversions
)

find_package(Eigen REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED QUIET COMPONENTS common sample_consensus io segmentation filters)
find_package(octomap REQUIRED)


add_service_files(
  DIRECTORY srv
  FILES
  GetPath.srv
  SelectPose.srv
  ExtractView.srv
  GetDroneState.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  visualization_msgs
  sensor_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include ${OCTOMAP_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS}
  LIBRARIES ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime roscpp geometry_msgs visualization_msgs tf kdtree  mav_msgs rospy std_msgs pcl_ros genmsg
  DEPENDS octomap PCL
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${rviz_visual_tools_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

set(LINK_LIBS
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


add_executable(sem_node  src/sem_node.cpp
                                src/sem_core.cpp
                                src/rrt_tree.cpp
                                src/semantic_mapper.cpp
                                src/octomap_generator/octomap_generator.cpp
                                src/semantic_octree/semantics_bayesian.cpp
                                src/semantic_octree/semantics_max.cpp
                                src/utilities/time_profiler.cpp
                            )

add_dependencies(sem_node ${catkin_EXPORTED_TARGETS}  )
#${PROJECT_NAME}_generate_messages_cpp
target_link_libraries(sem_node
   ${LINK_LIBS}
   ${Boost_DATE_TIME_LIBRARY}
   ${OCTOMAP_LIBRARIES} 
   ${catkin_LIBRARIES} 
   ${PCL_LIBRARIES}
)



add_executable(semantic_mapper_node src/semantic_mapper_node.cpp)
add_dependencies(semantic_mapper_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(semantic_mapper_node ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})


