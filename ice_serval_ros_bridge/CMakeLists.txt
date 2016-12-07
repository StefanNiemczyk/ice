cmake_minimum_required(VERSION 2.8.3)
project(ice_serval_ros_bridge)


find_package(catkin REQUIRED 
  roscpp
  roslib
  ice
  serval_wrapper
  cmake_modules
  message_generation
  geometry_msgs
)

## tiny xml package
find_package(TinyXML REQUIRED)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

#find_package(TBB REQUIRED)
add_definitions(-DWITH_THREADS=0)


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
  DEPENDS TinyXML
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)  

add_library(${PROJECT_NAME}
  src/IceServalBridge.cpp
  src/RosGContainerPublisher.cpp
  src/ServalCommunication.cpp
  src/ServalInformationMessage.cpp
  src/ServalInformationSender.cpp
  src/ServalInformationReceiver.cpp
  src/XMLInformationReader.cpp
)

add_dependencies(${PROJECT_NAME} ice)
add_dependencies(${PROJECT_NAME} serval_wrapper)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${TinyXML_LIBRARIES}
)


add_executable(${PROJECT_NAME}_node
  src/IceServalBridgeNode.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})


add_executable(knowledge_base_node
  eval/KnowledgeBaseNode.cpp
)
target_link_libraries(knowledge_base_node ${PROJECT_NAME})


add_executable(agent_node
  eval/AgentNode.cpp
)
target_link_libraries(agent_node ${PROJECT_NAME})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test 
  tests/test_entity.cpp
  tests/test_xml.cpp
 # tests/test_gmessagePublisher.cpp
  tests/test_bridge.cpp
  tests/test_main.cpp
  )
  
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
