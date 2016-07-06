cmake_minimum_required(VERSION 2.8.3)
project(serval_wrapper)

find_package(catkin REQUIRED COMPONENTS
  serval
)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# adding cpr for accessing the rest services
SET(BUILD_CPR_TESTS OFF CACHE BOOL "Set to ON to build cpr tests." FORCE)
SET(USE_SYSTEM_GTEST ON CACHE BOOL "If ON, this project will look in the system paths for an installed gtest library" FORCE)
add_subdirectory(cpr)

find_package(Boost REQUIRED COMPONENTS system)


################################################
## Declare ROS messages, services and actions ##
################################################


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


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
  INCLUDE_DIRS include ${CPR_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME} 
#  CATKIN_DEPENDS 
  DEPENDS ${CPR_LIBRARIES} serval
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CPR_INCLUDE_DIRS}
)
  

add_library(${PROJECT_NAME} 
  src/serval_interface.cpp
  
  src/serval_wrapper/keyring.cpp
  src/serval_wrapper/meshms.cpp
  src/serval_wrapper/rhizome.cpp
  src/serval_wrapper/MDPSocket.cpp
  src/serval_wrapper/MSPSocket.cpp
)


target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CPR_LIBRARIES}
)

#############
## Testing ##
#############

add_definitions(-DTEST_FILE_ROOT_PATH="${PROJECT_SOURCE_DIR}")

catkin_add_gtest(${PROJECT_NAME}-test 
  tests/test_keyring.cpp
  tests/test_meshms.cpp
  tests/test_rhizome.cpp
  tests/test_mdp.cpp
  tests/test_msp.cpp

  tests/test_main.cpp
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests
)

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test 
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CPR_LIBRARIES}
  )
endif()

