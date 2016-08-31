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

add_executable(client-mdp src/client_mdp.cpp)
add_executable(server-mdp src/server_mdp.cpp)

target_link_libraries(server-mdp ${catkin_LIBRARIES})
target_link_libraries(client-mdp ${catkin_LIBRARIES})

add_executable(client-msp src/client.cpp)
add_executable(server-msp src/server.cpp)

target_link_libraries(server-msp ${catkin_LIBRARIES})
target_link_libraries(client-msp ${catkin_LIBRARIES})
