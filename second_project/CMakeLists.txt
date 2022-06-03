cmake_minimum_required(VERSION 3.0.2)
project(second_project)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
)

catkin_package(

)


include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(odom_tf
  src/odom_tf.cpp
)
add_dependencies(odom_tf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_tf
  ${catkin_LIBRARIES}
)

