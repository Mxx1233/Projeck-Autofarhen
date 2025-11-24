# Projeck-Autofarhen - ROS2自动驾驶控制系统

## 📋 项目简介

本项目是基于ROS2 Humble的1:10比例自动驾驶模型车控制系统。

### 团队成员
- **控制算法开发**: zx - 负责control_pkg（横向控制和纵向控制）
- **硬件适配器开发**: Li Yixuan - 负责硬件桥接和底层通信

---

## 🏗️ 项目结构
```
autonomous_car_ws/
├── src/
│   ├── control_pkg/              # 控制算法包
│   │   ├── src/
│   │   │   └── controller_node.cpp
│   │   ├── include/
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── rusty_racer_interfaces/   # 自定义消息接口
│       ├── msg/
│       │   ├── LaneDeviation.msg
│       │   └── MotorCommand.msg
│       ├── CMakeLists.txt
│       └── package.xml
```

---

## 📡 系统架构

### 数据流
```
lane_detection_node → /lane_deviation → controller_node → /motor_command → uc_bridge_adapter_node
state_estimation_node → /odom → controller_node
```

### 关键话题

| 话题名 | 消息类型 | 发布者 | 订阅者 | 说明 |
|--------|----------|--------|--------|------|
| `/lane_deviation` | `rusty_racer_interfaces/msg/LaneDeviation` | lane_detection_node | controller_node | 车道偏差信息 |
| `/odom` | `nav_msgs/msg/Odometry` | state_estimation_node | controller_node | 车辆里程计 |
| `/motor_command` | `rusty_racer_interfaces/msg/MotorCommand` | controller_node | uc_bridge_adapter_node | 电机控制命令 |

---

## 🚀 快速开始

### 环境要求
- Ubuntu 22.04
- ROS2 Humble
- Docker (推荐)
- CLion (开发推荐)

### 编译项目
```bash
cd /root/autonomous_car_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 运行控制节点
```bash
ros2 run control_pkg controller_node_exe
```

---

## 🔧 控制参数

- **控制频率**: 50.0 Hz
- **PID参数**: Kp=1.00, Ki=0.00, Kd=0.10
- **目标速度**: 0.50 m/s

---

## 🧪 测试

### 模拟测试（无硬件）
```bash
# 终端1: 启动控制节点
ros2 run control_pkg controller_node_exe

# 终端2: 发布模拟车道偏差
ros2 topic pub /lane_deviation rusty_racer_interfaces/msg/LaneDeviation \
"lateral_error: 0.1
heading_error: 0.05
curvature: 0.0" -r 10

# 终端3: 监听控制输出
ros2 topic echo /motor_command
```

---

## 📝 开发指南

### 编译单个包
```bash
colcon build --packages-select control_pkg
source install/setup.bash
```

---

## 📄 License

本项目仅用于学术研究和学习目的。
