cmake_minimum_required(VERSION 2.8.3)
project(laser_odometry_core)

#Set compiler according C++11 support
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++11 support.")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  elseif(COMPILER_SUPPORTS_CXX0X)
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++0x support.")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support.
          Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
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

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

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
