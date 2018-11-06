cmake_minimum_required(VERSION 2.8.3)
project(vehicle_control)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  tf
  nav_msgs
  roscpp
  std_msgs
  sensor_msgs
  nodelet
)

  add_message_files(
    FILES
    commendMsg.msg
    motorsMsg.msg
    jointstatesMsg.msg
    posMsgs.msg
    tfMsg.msg
    gnd.msg
  )


## Generate added messages and services with any dependencies listed here
  generate_messages(
    DEPENDENCIES
     std_msgs
  )


catkin_package(
   LIBRARIES vehicle_control
   CATKIN_DEPENDS
    geometry_msgs
    tf
    nav_msgs
    roscpp
    std_msgs
    sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cmd_publish src/cmd_publish.cpp)
target_link_libraries(cmd_publish ${catkin_LIBRARIES})
add_dependencies(cmd_publish vehicle_control_generate_messages_cpp)

add_executable(vehicle_controller src/vehicle_controller.cpp)
target_link_libraries(vehicle_controller ${catkin_LIBRARIES})
add_dependencies(vehicle_controller vehicle_control_generate_messages_cpp)

add_executable(odom_node src/odom_node.cpp)
target_link_libraries(odom_node ${catkin_LIBRARIES})

add_executable(kalman_node src/kalman_node.cpp)
target_link_libraries(kalman_node ${catkin_LIBRARIES})

add_executable(encoder_node src/encoder_node.cpp)
target_link_libraries(encoder_node ${catkin_LIBRARIES})

add_executable(kalman_comb src/kalman_comb.cpp)
target_link_libraries(kalman_comb ${catkin_LIBRARIES})

add_executable(motor_dynamics src/motor_dynamics.cpp)
target_link_libraries(motor_dynamics ${catkin_LIBRARIES})

