cmake_minimum_required(VERSION 2.8.3)
project(laser_odometry_core)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    pluginlib
    property_bag
)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs pluginlib property_bag
)

include_directories(include ${catkin_INCLUDE_DIRS})

###############
##   Build   ##
###############

#add_library(${PROJECT_NAME} src/pal_carmen_reader.cpp)

#target_link_libraries(${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#)

################
##  Install   ##
################

#install(TARGETS ${PROJECT_NAME}
##   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    #   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(DIRECTORY include/
#        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
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
