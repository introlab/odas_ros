cmake_minimum_required(VERSION 2.8.3)
project(odas_msgs)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Tracked_source.msg
  Tracked_sources.msg
  DoA.msg
  DoAs.msg
  UniqueSSL.msg
  Ssl.msg
  UniqueSST.msg
  Sst.msg
  UniqueSSS.msg
  Sss.msg
  raw_data.msg

)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

