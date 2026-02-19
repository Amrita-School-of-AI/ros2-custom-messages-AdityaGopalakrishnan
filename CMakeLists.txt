
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
)

add_executable(status_publisher src/status_publisher.cpp)

ament_target_dependencies(status_publisher rclcpp)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(status_publisher "${cpp_typesupport_target}")

install(TARGETS
  status_publisher
  DESTINATION lib/${PROJECT_NAME}
)
