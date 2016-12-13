cmake_minimum_required(VERSION 2.8.3)
project(tb_knowledge_base)

find_package(catkin REQUIRED 
  roscpp
  roslib
  ice
  serval_wrapper
  cmake_modules
  )

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

#find_package(TBB REQUIRED)
add_definitions(-DWITH_THREADS=0)

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
#  LIBRARIES tb_knowledge_base
  CATKIN_DEPENDS ice
#  DEPENDS system_lib
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
  src/Pos3D.cpp
  src/PositionOrientation3D.cpp
  src/RTLandmark.cpp
  
  src/FusePositions.cpp
  src/Pos3D2RelativeToLandmark.cpp
  src/RelativeToLandmark2Pos3D.cpp
  src/TBLocalization.cpp
  src/VictimDetection.cpp
  
  src/TBCollectionFactory.cpp
  src/TBKnowledgeBase.cpp
)

add_dependencies(${PROJECT_NAME} ice)
add_dependencies(${PROJECT_NAME} serval_wrapper)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${TinyXML_LIBRARIES}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
 catkin_add_gtest(${PROJECT_NAME}-test 
   test/test_GPSTransformation.cpp
   test/test_TBKnowledgeBase.cpp
   
   test/test_main.cpp
 )
   
 
 if(TARGET ${PROJECT_NAME}-test)
   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
 endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
