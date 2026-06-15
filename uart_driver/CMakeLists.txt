cmake_minimum_required(VERSION 3.8)
project(uart_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# 手动加载类型支持，解决cmake找不到的bug
find_package(rosidl_typesupport_c REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

include_directories(include)

add_executable(uart_node
  src/uart_node.cpp
  src/per_joystick.cpp
  src/s_curve.cpp
)
# 依赖列表补齐两个类型支持库
ament_target_dependencies(uart_node
  rclcpp
  std_msgs
  geometry_msgs
  rosidl_typesupport_c
  rosidl_typesupport_cpp
)

install(TARGETS
  uart_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
