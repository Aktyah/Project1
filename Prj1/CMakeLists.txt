cmake_minimum_required(VERSION 2.8.3)
project(Prj1)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)

## Add custom message and its dependencies
add_message_files(
  FILES
  Num.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs
 )

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build talker and listener

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(Node1 src/Node1.cpp)
add_dependencies(Node1 ${catkin_EXPORTED_TARGETS})
target_link_libraries(Node1 ${catkin_LIBRARIES})
