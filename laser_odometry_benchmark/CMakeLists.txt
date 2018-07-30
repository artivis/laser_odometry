cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)
project(laser_odometry_benchmark)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++11 support.")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  laser_odometry_core
  nav_msgs
  pluginlib
  roscpp
  sensor_msgs
  tf2_eigen
  tf2_ros
  topic_tools
  message_generation
  gazebo_msgs
  geometry_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  RMSE.msg
  RMSEs.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES benchmark_node
  CATKIN_DEPENDS eigen_conversions laser_odometry_core nav_msgs pluginlib
                 roscpp sensor_msgs tf2_eigen tf2_ros topic_tools message_runtime
)

###########
## Build ##
###########

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

add_executable(benchmark_node src/benchmark_node.cpp)
target_link_libraries(benchmark_node ${catkin_LIBRARIES})

add_executable(ground_truth_node src/ground_truth_node.cpp)
target_link_libraries(ground_truth_node ${catkin_LIBRARIES})

#Require C++11
if(CMAKE_VERSION VERSION_LESS "3.1")
  set_target_properties(benchmark_node PROPERTIES COMPILE_FLAGS "-std=c++11")
else()
  set_property(TARGET benchmark_node PROPERTY CXX_STANDARD 11)
  set_property(TARGET benchmark_node PROPERTY CXX_STANDARD_REQUIRED ON)
  set_property(TARGET benchmark_node PROPERTY CXX_EXTENSIONS OFF)
endif()

if(CMAKE_VERSION VERSION_LESS "3.1")
  set_target_properties(ground_truth_node PROPERTIES COMPILE_FLAGS "-std=c++11")
else()
  set_property(TARGET ground_truth_node PROPERTY CXX_STANDARD 11)
  set_property(TARGET ground_truth_node PROPERTY CXX_STANDARD_REQUIRED ON)
  set_property(TARGET ground_truth_node PROPERTY CXX_EXTENSIONS OFF)
endif()

#############
## Install ##
#############

install(TARGETS benchmark_node
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch config)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
        PATTERN *.example EXCLUDE)
endforeach(dir)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_laser_odometry_benchmark.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
