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
)

## tiny xml package
find_package(TinyXML REQUIRED)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

add_definitions(-DWITH_THREADS=1)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

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

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
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

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_msgs  # Or other packages containing msgs
#  ice_msgs
#)

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
   LIBRARIES ice
   CATKIN_DEPENDS roscpp std_msgs ice_msgs cling_wrap
   DEPENDS TinyXML
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
  java/src/de/unikassel/vs/ice/RepresentationIndividual.java
  OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/java/lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  ${JNI_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
  src/engine/Communication.cpp
  src/engine/InformationSender.cpp
  src/engine/InformationReceiver.cpp

  src/engine/Position.cpp
  src/engine/RingBuffer.cpp
  src/engine/Time.cpp
  
  src/engine/CooperationRequest.cpp
  src/engine/CooperationResponse.cpp
  src/engine/Coordinator.cpp
  src/engine/EngineState.cpp
#  src/engine/InformationModel.cpp
#  src/engine/IntersectionInformationModel.cpp
#  src/engine/ModelComperator.cpp
  src/engine/NodeDescription.cpp
  src/engine/StreamDescription.cpp
#  src/engine/StreamTemplateDescription.cpp
  
  src/engine/AsynchronousTask.cpp
  src/engine/EventHandler.cpp
  src/engine/LambdaTask.cpp
  src/engine/Node.cpp
  src/engine/NodeStore.cpp
  
  src/engine/BaseInformationStream.cpp
  src/engine/InformationSpecification.cpp
#  src/engine/InformationStreamTemplate.cpp
  src/engine/InformationStore.cpp
#  src/engine/InformationType.cpp
  src/engine/StreamFactory.cpp
  
  src/engine/ProcessingModel.cpp
  src/engine/ProcessingModelGenerator.cpp
  src/engine/ASPSystem.cpp
  src/engine/ASPModelGenerator.cpp
  src/engine/FastUpdateStrategie.cpp
  src/engine/UpdateStrategie.cpp
  
  src/engine/OntologyInterface.cpp
  
  src/engine/Configuration.cpp
  src/engine/Identifier.cpp
  src/engine/ICEngine.cpp 
  src/engine/TimeFactory.cpp
  src/engine/XMLReader.cpp
  src/engine/RepresentationFactory.cpp
  
  src/engine/split.cpp

  src/ros/RosCommunication.cpp
  src/ros/RosInformationSender.cpp
  src/ros/RosInformationReceiver.cpp
  src/ros/RosMessageTransform.cpp
  src/ros/RosTimeFactory.cpp

)

## Declare a cpp executable
add_executable(ice_node src/node/ice_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(ice ice_msgs_generate_messages_cpp)
add_dependencies(ice_node ice_msgs_generate_messages_cpp)
add_dependencies(ice_node java_ontology_interface)

## Specify libraries to link a library or executable target against
target_link_libraries(ice_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${TinyXML_LIBRARIES}
  ${JNI_LIBRARIES}
)

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
# install(TARGETS ice ice_node
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
catkin_add_gtest(${PROJECT_NAME}-test 
 # test/eval/eval_ModelGeneration.cpp
 # test/eval/ModelGeneration.cpp
 # test/eval/EvalScenarios.cpp
 # test/eval/eval_AccessSpeed.cpp
   
 # test/etc/TestTime.cpp
 # test/etc/EngineStuff.cpp
 # test/test_ASPComposition.cpp
  test/test_ASPRepComputation.cpp
 # test/test_JNI.cpp
 # test/test_ASPCoordinator.cpp
=======
 # test/test_JNI.cpp
  test/test_ASPCoordinator.cpp
 # test/test_serialize.cpp
>>>>>>> Temporary merge branch 2
 # test/test_RingBuffer.cpp
 # test/test_Buffer.cpp
 # test/test_InformationStore.cpp
 # test/test_NodeStore.cpp
 # test/test_EventHandler.cpp
 # test/test_Time.cpp
 # test/test_XMLReader
 # test/test_Node.cpp
 # test/test_ModelComperator.cpp
 # test/test_ICEngine.cpp
 # test/test_Representation.cpp
  test/test_main.cpp
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
)


add_dependencies(${PROJECT_NAME}-test java_ontology_interface)

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${TinyXML_LIBRARIES}
  	${JNI_LIBRARIES}
  )
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
