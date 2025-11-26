cmake_minimum_required(VERSION 3.8)
project(control_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rusty_racer_interfaces REQUIRED)

# 包含目录
include_directories(include)

# 可执行文件：controller_node
add_executable(controller_node_exe 
  src/controller_node.cpp
)

ament_target_dependencies(controller_node_exe
  rclcpp std_msgs nav_msgs rusty_racer_interfaces
)

# 安装
install(TARGETS 
  controller_node_exe
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs nav_msgs rusty_racer_interfaces)

ament_package()
