cmake_minimum_required(VERSION 3.8)
project(control_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rusty_racer_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)

# 包含目录
include_directories(
  include
)

# 添加可执行文件
add_executable(controller_node_exe 
  src/controller_node.cpp
)

# 链接依赖
ament_target_dependencies(controller_node_exe
  rclcpp
  rusty_racer_interfaces
  nav_msgs
)

# 安装可执行文件
install(TARGETS
  controller_node_exe
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY
  include/
  DESTINATION include
)

ament_package()
