cmake_minimum_required(VERSION 2.8.3)
project(ice_serval_ros_bridge)


find_package(catkin REQUIRED 
  roscpp
  roslib
  ice
  serval_wrapper
)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")


################################################
## Declare ROS messages, services and actions ##
################################################


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

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
#  LIBRARIES ice_serval_ros_bridge
  CATKIN_DEPENDS ice serval_wrapper
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)  

add_library(${PROJECT_NAME}
  src/IceServalBridge.cpp
  src/Identities.cpp
  src/Identity.cpp
)

add_dependencies(${PROJECT_NAME} ice)
add_dependencies(${PROJECT_NAME} serval_wrapper)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


add_executable(${PROJECT_NAME}_node
  src/IceServalBridgeNode.cpp
)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test 
  tests/test_identity.cpp
  tests/test_main.cpp)
  
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
