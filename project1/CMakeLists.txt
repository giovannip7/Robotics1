cmake_minimum_required(VERSION 2.8.3)
project(project1)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs nav_msgs message_filters message_generation geometry_msgs tf tf2 dynamic_reconfigure)

add_service_files(
  FILES
  reinit_pose.srv
  reselect_pose.srv
)

add_message_files(
   FILES
   MotorSpeed.msg
   pre_speed.msg
   MethodMessage.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Declare a catkin package
catkin_package( CATKIN_DEPENDS message_filters message_runtime std_msgs)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(sync_pub src/sync_pub.cpp)
add_dependencies(sync_pub custom_messages_generate_messages_cpp)
target_link_libraries(sync_pub ${catkin_LIBRARIES})

add_executable(lin_ang src/lin_ang.cpp)
add_dependencies(lin_ang custom_messages_generate_messages_cpp)
target_link_libraries(lin_ang ${catkin_LIBRARIES})

add_executable(ERK src/ERK.cpp)
add_dependencies(ERK custom_messages_generate_messages_cpp)
add_dependencies(ERK ${project1}_gencfg)
add_dependencies(ERK ${catkin_EXPORTED_TARGETS})
target_link_libraries(ERK ${catkin_LIBRARIES})

add_executable(final_message src/final_message.cpp)
add_dependencies(final_message custom_messages_generate_messages_cpp)
target_link_libraries(final_message ${catkin_LIBRARIES})
