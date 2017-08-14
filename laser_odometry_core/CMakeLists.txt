cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)
project(laser_odometry_core)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++11 support.")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  eigen_conversions
  geometry_msgs
  nav_msgs
  pluginlib
  roscpp
  sensor_msgs
  tf
  tf_conversions
)

#find_package(tf_conversions 1.11.8 REQUIRED)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cmake_modules geometry_msgs nav_msgs pluginlib
                 roscpp sensor_msgs tf tf_conversions
  DEPENDS Eigen
)

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

###############
##   Build   ##
###############

add_library(${PROJECT_NAME}
    src/laser_odometry_base.cpp
    src/laser_odometry_instantiater.cpp
    src/laser_odometry_utils.cpp
    src/laser_odometry_conversion.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#Require C++11
if(CMAKE_VERSION VERSION_LESS "3.1")
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(${PROJECT_NAME} INTERFACE "-std=c++11")
  endif()
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
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

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
