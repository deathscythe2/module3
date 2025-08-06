cmake_minimum_required(VERSION 3.8)
project(examine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlesim REQUIRED)
find_package(example_interfaces REQUIRED)


add_executable(turtleclone src/turtleclone.cpp)
ament_target_dependencies(turtleclone rclcpp geometry_msgs tf2 tf2_ros turtlesim)


add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher rclcpp std_msgs)

add_executable(subscriber src/subscriber.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs)

add_executable(draw_square src/draw_square.cpp)
ament_target_dependencies(draw_square rclcpp geometry_msgs)

add_executable(broadcaster src/broadcaster.cpp)
ament_target_dependencies(broadcaster rclcpp geometry_msgs tf2 tf2_ros turtlesim)  

add_executable(broadcaster_sub src/broadcaster_sub.cpp)
ament_target_dependencies(broadcaster_sub rclcpp geometry_msgs tf2 tf2_ros turtlesim)  

add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server rclcpp example_interfaces)

add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client rclcpp example_interfaces)



install(TARGETS
  turtleclone
  broadcaster
  broadcaster_sub
  draw_square
  publisher
  subscriber
  server
  client
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)



ament_package()
