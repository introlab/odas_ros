cmake_minimum_required(VERSION 2.8.3)
project(odas_server)


find_package(catkin REQUIRED COMPONENTS
  odas_msgs
  roscpp
  rospy
  std_msgs
  message_runtime
)

catkin_package(
  CATKIN_DEPENDS odas_msgs roscpp rospy std_msgs message_runtime
)

#Build odas library
add_subdirectory(odas)

#Setup python
catkin_python_setup()

#Install executables
install(PROGRAMS
   scripts/server.py
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/odas/include
  ${CMAKE_CURRENT_SOURCE_DIR}/odas/include/odas
  ${CMAKE_CURRENT_SOURCE_DIR}/odas/demo
)

#Reuse base client demo files
set(odas_shared_srcs
    odas/demo/configs.c
    odas/demo/objects.c
    odas/demo/parameters.c
    odas/demo/profiler.c
    odas/demo/threads.c
)


## Declare a C++ executable
add_executable(odas_core_node src/odas_core_node.cpp ${odas_shared_srcs})
add_executable(pioneer_odas_core_node src/pioneer_odas_core_node.cpp ${odas_shared_srcs})
add_executable(turtle_odas_core_node src/turtle_odas_core_node.cpp ${odas_shared_srcs})

## Specify libraries to link a library or executable target against
target_link_libraries(odas_core_node
   ${catkin_LIBRARIES}
   odas
)
target_link_libraries(pioneer_odas_core_node
   ${catkin_LIBRARIES}
   odas
)
target_link_libraries(turtle_odas_core_node
   ${catkin_LIBRARIES}
   odas
)


