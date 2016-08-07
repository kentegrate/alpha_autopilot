cmake_minimum_required(VERSION 2.8.3)
project(alpha_drivers)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -lbcm2835 -std=c++11 -I/home/pi/robidouille/raspicam_cv -L/home/pi/robidouille/raspicam_cv -L/home/pi/userland/build/lib")
#
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  alpha_msgs
  image_transport
  cv_bridge
)
find_package(Boost 1.54.0 REQUIRED)

find_package(OpenCV 2.4.8 REQUIRED)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)


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

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES PCA9685 sbus_decoder raspi_cam
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(alpha_drivers
#   src/${PROJECT_NAME}/alpha_drivers.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(alpha_drivers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(alpha_drivers_node src/alpha_drivers_node.cpp)


add_library(ms5611 src/ms5611.cpp)
target_link_libraries(ms5611
  ${catkin_LIBRARIES}
  bcm2835
)

add_library(mpu9250 src/mpu9250.cpp)
target_link_libraries(mpu9250
  ${catkin_LIBRARIES}
  bcm2835
)



#add_library(pca9685 src/pca9685.cpp)
#target_link_libraries(pca9685
#  ${catkin_LIBRARIES}
#  bcm2835
#)


add_library(PCA9685 SHARED src/PCA9685.cpp)
target_link_libraries(PCA9685
  ${catkin_LIBRARIES}
)

#add_executable(test_mpu9250 src/test_mpu9250.cpp)
#target_link_libraries(test_mpu9250
#  ${catkin_LIBRARIES}
#  mpu9250
#)

#add_executable(test_ms5611 src/test_ms5611.cpp)
#target_link_libraries(test_ms5611
#  ${catkin_LIBRARIES}
#  ms5611
#)
add_library (sbus_decoder src/decoder/GPIO_RPI.cpp src/decoder/RCInput_RPI.cpp
                             src/decoder/sbus.cpp
			     src/decoder/RCInput.cpp)
			    
			   # src/decoder/Scheduler.cpp src/decoder/Thread.cpp
target_link_libraries(sbus_decoder
  ${catkin_LIBRARIES}
)
add_dependencies(sbus_decoder ${alpha_msgs_EXPORTED_TARGETS})

add_library(ahrs src/ahrs.cpp)
target_link_libraries(ahrs
  ${catkin_LIBRARIES}
)

add_executable(spi_sensors src/spi_sensors.cpp src/altitude_estimator.cpp) 
target_link_libraries(spi_sensors
  ${catkin_LIBRARIES}
  mpu9250
  ms5611
  ahrs
  )
add_dependencies(spi_sensors ${alpha_msgs_EXPORTED_TARGETS})

#add_executable(rcout_node src/RCOut_node.cpp)
#target_link_libraries(rcout_node
#  ${catkin_LIBRARIES}
#  PCA9685
#  wiringPi
#  )
#add_dependencies(rcout_node ${alpha_msgs_EXPORTED_TARGETS})



add_library(raspi_cam src/RaspiCam.cpp)
target_link_libraries(raspi_cam
  ${OpenCV_LIBRARIES}
  raspicamcv
  mmal_core
  mmal
  mmal_util
  vcos
  bcm_host
)
add_executable(camera_node src/raspi_camera.cpp)
target_link_libraries(camera_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  raspi_cam
)
add_executable(indicator_node src/status_indicator.cpp)
target_link_libraries(indicator_node
  ${catkin_LIBRARIES}
  wiringPi
)

  
## Add cmake target dependencies of the executable
## same as for the library above


## Specify libraries to link a library or executable target against
# target_link_libraries(alpha_drivers_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS alpha_drivers alpha_drivers_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_alpha_drivers.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
