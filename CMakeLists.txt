cmake_minimum_required(VERSION 3.5)
project(odas_ros)


# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(MUSIC_BEAT_DETECTOR_ENABLE_TESTS OFF CACHE BOOL "")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(audio_utils REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OdasSsl.msg"
  "msg/OdasSslArrayStamped.msg"
  "msg/OdasSst.msg"
  "msg/OdasSstArrayStamped.msg"
  DEPENDENCIES std_msgs
)

add_subdirectory(src/odas)

include_directories(
  ${catkin_INCLUDE_DIRS}
  src/odas/include
  src/odas/include/odas
  src/odas/demo/odaslive
)

#Reuse base client demo files
set(odas_shared_srcs
    src/odas/demo/odaslive/configs.c
    src/odas/demo/odaslive/objects.c
    src/odas/demo/odaslive/parameters.c
    src/odas/demo/odaslive/profiler.c
    src/odas/demo/odaslive/threads.c
)


# C++ Nodes
add_executable(odas_core_node src/odas_core_node.cpp ${odas_shared_srcs})
ament_target_dependencies(odas_core_node rclcpp)
target_link_libraries(odas_core_node odas)
install(TARGETS odas_core_node DESTINATION lib/${PROJECT_NAME})


# Python Nodes
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/echocancel.sh
  scripts/movesinks.sh
  scripts/setvolume.sh
  scripts/odas_echocancel_filter_server_node.py
  scripts/odas_server_node.py
  scripts/odas_visualization_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
