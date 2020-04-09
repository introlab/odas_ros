cmake_minimum_required(VERSION 2.8.3)
project(sound_tracking)

find_package(catkin REQUIRED COMPONENTS
  message_runtime
  nav_msgs
  odas_msgs
  roscpp
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 # LIBRARIES sound_tracking
 CATKIN_DEPENDS nav_msgs odas_msgs roscpp std_msgs
 # DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${odas_msgs_INCLUDE_DIRS}
)

# Add cmake target dependencies of the library
## add_dependencies(odas_msgs_generate_messages ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(sound_tracking_node src/sound_tracking.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(sound_tracking_node ${catkin_LIBRARIES})
## add_dependencies(sound_tracking_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
