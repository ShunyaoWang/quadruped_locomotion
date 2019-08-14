cmake_minimum_required(VERSION 2.8.3)
project(quadruped_odom)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  kdl_parser
  tf
  geometry_msgs
  std_msgs
  sensor_msgs
  qp_solver
  quadruped_model
  free_gait_msgs
  sim_assiants
#  free_gait_ros
  free_gait_core
#  orocos_kdl
)
find_package(Eigen3 REQUIRED)
#find_package(Sophus REQUIRED)
find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

catkin_package(
  INCLUDE_DIRS
  include
  ${EIGEN3_INCLUDE_DIR}
  ${Sophus_INCLUDE_DIRS}
  LIBRARIES quadruped_odom
  CATKIN_DEPENDS
  roscpp
  kdl_parser
#  qp_solver
#  quadruped_model
#  sim_assiants
#  free_gait_core
#  orocos_kdl
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
#  ${RBDL_INCLUDE_DIR}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/legodom.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Sophus_LIBRARIES})

add_executable(legodom
  src/legodom_node.cpp
)
# add_dependencies(legodom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(legodom
  ${PROJECT_NAME} ${catkin_LIBRARIES} ${Sophus_LIBRARIES}
)

add_executable(legodom_test
  src/legodom_test.cpp
)
add_dependencies(legodom_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(legodom_test
  ${PROJECT_NAME} ${catkin_LIBRARIES} ${Sophus_LIBRARIES}
)


 install(TARGETS ${PROJECT_NAME} #${PROJECT_NAME}_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
 )
