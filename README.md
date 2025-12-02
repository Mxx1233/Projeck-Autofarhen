# Rusty Racer Control Package / 自动驾驶控制包

## English Version

### Overview
This package implements the autonomous vehicle control algorithm for the Rusty Racer project, including lateral (steering) and longitudinal (speed) control using PID controllers.

### System Requirements
- **ROS2:** Humble
- **OS:** Ubuntu 22.04
- **C++ Standard:** C++17
- **Dependencies:** rclcpp, nav_msgs, rusty_racer_interfaces

### Package Contents
```
control_pkg/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package metadata and dependencies
├── config/
│   └── controller_params.yaml  # PID parameters and settings
├── include/
│   └── control_pkg/
│       └── (header files if any)
└── src/
    └── controller_node.cpp     # Main control node implementation
```

### Installation Steps

#### 1. Prerequisites
Make sure you have ROS2 Humble installed:
```bash
source /opt/ros/humble/setup.bash
```

#### 2. Clone or Extract Package
Place the `control_pkg` folder in your ROS2 workspace:
```bash
cd ~/ros2_ws/src
# Extract the package here
```

#### 3. Install Dependencies
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

#### 4. Build
```bash
cd ~/ros2_ws
colcon build --packages-select control_pkg
source install/setup.bash
```

#### 5. Run
```bash
ros2 run control_pkg controller_node_exe
```

### Quick Test
In another terminal, publish test data:
```bash
# Test with 5cm lateral deviation
ros2 topic pub /lane_deviation rusty_racer_interfaces/msg/LaneDeviation \
  "{lateral_error: 0.05, heading_error: 0.0, curvature: 0.0}" --once

# Monitor output
ros2 topic echo /motor_command
```

### Configuration
Edit parameters in `config/controller_params.yaml`:
- `control_frequency`: Control loop frequency (Hz)
- `target_velocity`: Desired velocity (m/s)
- `lateral_pid`: Lateral control PID gains (kp, ki, kd)
- `longitudinal_pid`: Longitudinal control PID gains (kp, ki, kd)

After modifying parameters, restart the node (no rebuild needed).

### Topics

**Subscribed:**
- `/lane_deviation` - Lane tracking errors
- `/odom` - Vehicle odometry

**Published:**
- `/motor_command` - Steering and motor commands

### Troubleshooting

**Problem: Package not found during build**
```bash
# Make sure rusty_racer_interfaces is built first
colcon build --packages-select rusty_racer_interfaces
colcon build --packages-select control_pkg
```

**Problem: Node crashes on startup**
- Check that parameter file exists in `config/`
- Verify ROS2 environment is sourced

**Problem: No output on /motor_command**
- Ensure input topics are being published
- Check node is running: `ros2 node list`

### Documentation
- See `INTERFACE.md` for input/output specifications
- See `TUNING_GUIDE.md` for PID tuning instructions

---

## 中文版本

### 概述
本包实现了 Rusty Racer 项目的自动驾驶控制算法，包括使用 PID 控制器的横向（转向）和纵向（速度）控制。

### 系统要求
- **ROS2:** Humble 版本
- **操作系统:** Ubuntu 22.04
- **C++ 标准:** C++17
- **依赖项:** rclcpp, nav_msgs, rusty_racer_interfaces

### 包内容
```
control_pkg/
├── CMakeLists.txt              # 编译配置
├── package.xml                 # 包元数据和依赖
├── config/
│   └── controller_params.yaml  # PID 参数和设置
├── include/
│   └── control_pkg/
│       └── (头文件，如有)
└── src/
    └── controller_node.cpp     # 主控制节点实现
```

### 安装步骤

#### 1. 前置条件
确保已安装 ROS2 Humble：
```bash
source /opt/ros/humble/setup.bash
```

#### 2. 克隆或解压包
将 `control_pkg` 文件夹放入你的 ROS2 工作空间：
```bash
cd ~/ros2_ws/src
# 在此处解压包
```

#### 3. 安装依赖
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

#### 4. 编译
```bash
cd ~/ros2_ws
colcon build --packages-select control_pkg
source install/setup.bash
```

#### 5. 运行
```bash
ros2 run control_pkg controller_node_exe
```

### 快速测试
在另一个终端发布测试数据：
```bash
# 测试 5cm 横向偏差
ros2 topic pub /lane_deviation rusty_racer_interfaces/msg/LaneDeviation \
  "{lateral_error: 0.05, heading_error: 0.0, curvature: 0.0}" --once

# 监听输出
ros2 topic echo /motor_command
```

### 配置
编辑 `config/controller_params.yaml` 中的参数：
- `control_frequency`: 控制循环频率 (Hz)
- `target_velocity`: 目标速度 (m/s)
- `lateral_pid`: 横向控制 PID 增益 (kp, ki, kd)
- `longitudinal_pid`: 纵向控制 PID 增益 (kp, ki, kd)

修改参数后重启节点即可（无需重新编译）。

### 话题

**订阅：**
- `/lane_deviation` - 车道跟踪误差
- `/odom` - 车辆里程计

**发布：**
- `/motor_command` - 转向和电机命令

### 故障排除

**问题：编译时找不到包**
```bash
# 确保先编译 rusty_racer_interfaces
colcon build --packages-select rusty_racer_interfaces
colcon build --packages-select control_pkg
```

**问题：节点启动时崩溃**
- 检查 `config/` 中的参数文件是否存在
- 确认已 source ROS2 环境

**问题：/motor_command 无输出**
- 确保输入话题正在发布
- 检查节点是否运行：`ros2 node list`

### 文档
- 参见 `INTERFACE.md` 了解输入输出规格
- 参见 `TUNING_GUIDE.md` 了解 PID 调试说明

---

## Contact / 联系方式
**Project:** Rusty Racer Autonomous Vehicle  
**Control Algorithm Developer:** zx  
**Institution:** TU Darmstadt
