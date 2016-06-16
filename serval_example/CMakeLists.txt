cmake_minimum_required(VERSION 2.8.3)
project(serval_example)

find_package(catkin REQUIRED COMPONENTS
  serval
)

catkin_package(
	CATKIN_DEPENDS serval
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(serval-sender src/sender.cpp)
add_executable(serval-receiver src/receiver.cpp)
add_executable(serval-client src/client.cpp)
add_executable(serval-server src/server.cpp)

# link against precompiled serval library
target_link_libraries(serval-sender
  ${catkin_LIBRARIES}
)
target_link_libraries(serval-receiver
  ${catkin_LIBRARIES}
)
target_link_libraries(serval-client
  ${catkin_LIBRARIES}
)
target_link_libraries(serval-server
  ${catkin_LIBRARIES}
)
