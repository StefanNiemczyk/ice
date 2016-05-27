cmake_minimum_required(VERSION 2.8.3)
project(serval_wrapper_example)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  serval serval_wrapper
)

catkin_package(
	CATKIN_DEPENDS serval serval_wrapper
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(serval-wrapper-client src/client.cpp)
add_executable(serval-wrapper-server src/server.cpp)

# link against precompiled serval library
#target_link_libraries(serval-wrapper-client
#  ${catkin_LIBRARIES}
#)

target_link_libraries(serval-wrapper-server
  ${catkin_LIBRARIES}
)

target_link_libraries(serval-wrapper-client
  ${catkin_LIBRARIES}
)
