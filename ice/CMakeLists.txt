cmake_minimum_required(VERSION 2.8.3)
project(ice)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  message_generation
  std_msgs
  ice_msgs
  cling_wrap
  cmake_modules
  muparser
)

## tiny xml package
find_package(TinyXML REQUIRED)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

#find_package(TBB REQUIRED)
add_definitions(-DWITH_THREADS=0)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS 
  system 
  serialization
)

## Java Flags
find_package(Java REQUIRED)
include(UseJava)
# Old script set JAVA_HOME in .bashrc
include(FindJNI)

set(CMAKE_JAVA_INCLUDE_PATH 
  ${CMAKE_CURRENT_SOURCE_DIR}/java/lib/owlapi-distribution-3.5.1.jar
  ${CMAKE_CURRENT_SOURCE_DIR}/java/lib/HermiT.jar
  ${CMAKE_CURRENT_SOURCE_DIR}/java/lib/slf4j-simple-1.7.7.jar
  )

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
   INCLUDE_DIRS include ${JNI_INCLUDE_DIRS}
   LIBRARIES ice ${JNI_LIBRARIES}
   CATKIN_DEPENDS roscpp roslib std_msgs ice_msgs cling_wrap muparser 
   DEPENDS TinyXML Boost
)

###########
## Build ##
###########

## Java Build
#set(CMAKE_JAR_CLASSES_PREFIX java/src/de/unikassel/vs/ice/)
add_jar(
  java_ontology_interface 
  SOURCES java/src/de/unikassel/vs/ice/IceOntologyInterface.java
  java/src/de/unikassel/vs/ice/IceIris.java
  java/src/de/unikassel/vs/ice/IceVisitor.java
  java/src/de/unikassel/vs/ice/InfoStructureVisitor.java
  java/src/de/unikassel/vs/ice/NodeIROVisitor.java
  java/src/de/unikassel/vs/ice/RepresentationVisitor.java
  java/src/de/unikassel/vs/ice/Representation.java
  java/src/de/unikassel/vs/ice/LogLevel.java
  OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/java/lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  ${JNI_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
  src/engine/CommunicationInterface.cpp
  src/engine/InformationReceiver.cpp
  
  src/engine/Message.cpp
  src/engine/CommandMessage.cpp
  src/engine/IdMessage.cpp
  src/engine/InformationMessage.cpp
  src/engine/IntMessage.cpp
  src/engine/OffersMessage.cpp
  src/engine/OntologyIdMessage.cpp
  src/engine/RequestMessage.cpp
  src/engine/SubModelMessage.cpp
  src/engine/SubModelResponseMessage.cpp
  
  src/engine/ComJobBase.cpp
  src/engine/IdentityRequest.cpp
  src/engine/InformationRequest.cpp
  src/engine/CooperationRequest.cpp
  
  src/engine/NodeDescription.cpp
  src/engine/StreamDescription.cpp
  
  src/engine/EventHandler.cpp
  src/engine/LambdaTask.cpp
  src/engine/Node.cpp
  src/engine/NodeStore.cpp
  src/engine/TransformationNode.cpp
  
  src/engine/BaseInformationStream.cpp
  src/engine/InformationSpecification.cpp
  src/engine/InformationStore.cpp
  src/engine/StreamFactory.cpp
  src/engine/StreamStore.cpp
  
  src/engine/OntologyInterface.cpp
  
  src/engine/ProcessingModel.cpp
  src/engine/ProcessingModelGenerator.cpp
  src/engine/ASPModelGenerator.cpp
  src/engine/FastUpdateStrategie.cpp
  src/engine/UpdateStrategie.cpp
  
  src/engine/ASPTransformationGeneration.cpp
  src/engine/GContainerFactory.cpp
  src/engine/split.cpp
  src/engine/Transformation.cpp
  src/engine/XMLTransformationReader.cpp
  
  src/engine/Configuration.cpp
  src/engine/Entity.cpp
  src/engine/EntityDirectory.cpp
  src/engine/Identifier.cpp
  src/engine/ICEngine.cpp 

  src/ros/RosCommunication.cpp
  src/ros/RosInformationSender.cpp
  src/ros/RosInformationReceiver.cpp
  src/ros/RosMessageTransform.cpp
  src/ros/RosTimeFactory.cpp
)

## Declare a cpp executable
add_executable(ice_node src/node/ice_node.cpp)

## Add cmake target dependencies of the executable/library
add_dependencies(ice ice_msgs_generate_messages_cpp)
add_dependencies(ice_node ice_msgs_generate_messages_cpp)
add_dependencies(ice_node java_ontology_interface)

## Specify libraries to link a library or executable target against
target_link_libraries(ice_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${TinyXML_LIBRARIES}
  ${JNI_LIBRARIES}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test 
  test/eval/MemoryMonitor.cpp
  test/eval/eval_ModelGeneration.cpp
  test/eval/ModelGeneration.cpp
  test/eval/EvalScenarios.cpp
 # test/eval/eval_AccessSpeed.cpp
 
 # test/etc/EngineStuff.cpp
 # test/test_ASPCoordinator.cpp
 # test/test_JNI.cpp
 # test/test_ASPComposition.cpp
 # test/test_ASPRepComputation.cpp
 # test/test_GContainer.cpp
 # test/test_RepresentationTransformation.cpp
 # test/test_RepresentationJSON.cpp
 # test/test_EulerRepTransformation.cpp
 # test/test_RingBuffer.cpp
 # test/test_Buffer.cpp
 # test/test_EventHandler.cpp
 # test/test_Time.cpp
 # test/test_StreamStore.cpp
 # test/test_Node.cpp
 # test/test_NodeStore.cpp
 # test/test_CallbackList.cpp
 # test/test_InformationStore.cpp
 # test/test_TransformationNode.cpp

  test/test_main.cpp
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
)

if(TARGET ${PROJECT_NAME}-test)
  add_dependencies(${PROJECT_NAME}-test java_ontology_interface)
  
  target_link_libraries(${PROJECT_NAME}-test
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${TinyXML_LIBRARIES}
  	${JNI_LIBRARIES}
  )
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
