cmake_minimum_required(VERSION 3.5)
project(go2_wifi_graph_announcer)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)

add_executable(go2_wifi_graph_announcer graph_announcer.cpp)
ament_target_dependencies(go2_wifi_graph_announcer rclcpp sensor_msgs unitree_go unitree_api)
install(TARGETS go2_wifi_graph_announcer DESTINATION lib/${PROJECT_NAME})

ament_package()
