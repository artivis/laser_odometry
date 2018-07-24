cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)
project(laser_odometry_node)

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
)

add_message_files(
  FILES
  RMSE.msg
  RMSEs.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} comparison_node
  CATKIN_DEPENDS eigen_conversions laser_odometry_core nav_msgs pluginlib
                 roscpp sensor_msgs tf2_eigen tf2_ros topic_tools message_runtime
)

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

###############
##   Build   ##
###############

add_executable(${PROJECT_NAME} src/laser_odometry_node.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(comparison_node src/comparison_node.cpp)
target_link_libraries(comparison_node ${catkin_LIBRARIES})

add_executable(ground_truth_node src/ground_truth_node.cpp)
target_link_libraries(ground_truth_node ${catkin_LIBRARIES})

#Require C++11
if(CMAKE_VERSION VERSION_LESS "3.1")
  set_target_properties(comparison_node PROPERTIES COMPILE_FLAGS "-std=c++11")
else()
  set_property(TARGET comparison_node PROPERTY CXX_STANDARD 11)
  set_property(TARGET comparison_node PROPERTY CXX_STANDARD_REQUIRED ON)
  set_property(TARGET comparison_node PROPERTY CXX_EXTENSIONS OFF)
endif()

if(CMAKE_VERSION VERSION_LESS "3.1")
  set_target_properties(ground_truth_node PROPERTIES COMPILE_FLAGS "-std=c++11")
else()
  set_property(TARGET ground_truth_node PROPERTY CXX_STANDARD 11)
  set_property(TARGET ground_truth_node PROPERTY CXX_STANDARD_REQUIRED ON)
  set_property(TARGET ground_truth_node PROPERTY CXX_EXTENSIONS OFF)
endif()

if(CMAKE_VERSION VERSION_LESS "3.1")
  set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-std=c++11")
else()
  set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
  set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)
  set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_EXTENSIONS OFF)
endif()

################
##  Install   ##
################

install(TARGETS ${PROJECT_NAME}
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch config)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
        PATTERN *.example EXCLUDE)
endforeach(dir)

#install(DIRECTORY include/${PROJECT_NAME}/
#    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)

################
##    Test    ##
################

#if(CATKIN_ENABLE_TESTING)
#    find_package(rostest REQUIRED)
#    find_package(catkin REQUIRED COMPONENTS
#        roslib
#    )

#    add_rostest_gtest(test_carmen_reader_test
#      test/test_carmen_reader.test
#      test/test_carmen_reader.cpp
#    )

#  target_link_libraries(test_carmen_reader_test ${catkin_LIBRARIES} ${PROJECT_NAME})

#endif()
