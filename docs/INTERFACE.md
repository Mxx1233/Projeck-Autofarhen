# Control Algorithm Interface Specification / 控制算法接口规范

## English Version

### Overview
This document describes the input and output interfaces of the control algorithm node.

---

## Input Specifications / 输入规范

### 1. Lane Deviation Input / 车道偏差输入

**Topic Name:** `/lane_deviation`  
**Message Type:** `rusty_racer_interfaces/msg/LaneDeviation`  
**Frequency:** 10-30 Hz (recommended)

**Message Structure:**
```
float32 lateral_error      # Lateral deviation from center line
float32 heading_error      # Heading angle error
float32 curvature          # Road curvature
```

**Field Descriptions:**

| Field | Type | Unit | Range | Description |
|-------|------|------|-------|-------------|
| `lateral_error` | float32 | meters (m) | -0.5 to 0.5 | Distance from vehicle center to lane center. Positive = right deviation, Negative = left deviation |
| `heading_error` | float32 | radians (rad) | -π to π | Angle difference between vehicle heading and lane direction. Positive = heading right, Negative = heading left |
| `curvature` | float32 | 1/meters (rad/m) | -1.0 to 1.0 | Road curvature. Positive = left turn, Negative = right turn, 0 = straight |

**Typical Values:**
- Small deviation: `lateral_error` = ±0.05m (5cm)
- Large deviation: `lateral_error` = ±0.25m (25cm)
- Normal heading error: `heading_error` = ±0.1 rad (±6°)
- Moderate curve: `curvature` = ±0.1 rad/m (radius ≈ 10m)

**Example:**
```bash
ros2 topic pub /lane_deviation rusty_racer_interfaces/msg/LaneDeviation \
  "{lateral_error: 0.05, heading_error: 0.03, curvature: 0.0}"
```

---

### 2. Odometry Input / 里程计输入

**Topic Name:** `/odom`  
**Message Type:** `nav_msgs/msg/Odometry`  
**Frequency:** 20-50 Hz (recommended)

**Used Fields:**
```
geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear
      float64 x    # Vehicle forward velocity
```

**Field Descriptions:**

| Field | Type | Unit | Range | Description |
|-------|------|------|-------|-------------|
| `twist.twist.linear.x` | float64 | m/s | 0 to 2.0 | Current forward velocity of the vehicle |

**Note:** Only the forward velocity component is used for longitudinal control.

**Example:**
```bash
ros2 topic pub /odom nav_msgs/msg/Odometry \
  "{twist: {twist: {linear: {x: 0.3}}}}"
```

---

## Output Specifications / 输出规范

### Motor Command Output / 电机命令输出

**Topic Name:** `/motor_command`  
**Message Type:** `rusty_racer_interfaces/msg/MotorCommand`  
**Frequency:** 50 Hz (default control frequency)

**Message Structure:**
```
float32 steering_angle    # Desired steering angle
float32 motor_level       # Motor drive level
```

**Field Descriptions:**

| Field | Type | Unit | Range | Description |
|-------|------|------|-------|-------------|
| `steering_angle` | float32 | radians (rad) | -0.52 to 0.52 | Servo steering angle. Positive = right turn, Negative = left turn. Limited by `max_steering_angle` parameter |
| `motor_level` | float32 | normalized | 0.0 to 1.0 | Motor drive level. 0.0 = stop/coast, 1.0 = maximum forward speed. Currently only forward motion is used |

**Physical Meaning:**
- **steering_angle:** Direct control of servo angle
  - Example: 0.26 rad ≈ 15° right turn
  - Typical range during driving: ±0.3 rad (±17°)
  
- **motor_level:** Normalized throttle command
  - 0.0 = No power (vehicle coasts)
  - 0.25 = 25% of maximum speed
  - 1.0 = Full speed
  - Limited by `max_velocity` parameter (default: 2.0 m/s)

**Typical Output Values:**

| Scenario | steering_angle | motor_level | Description |
|----------|---------------|-------------|-------------|
| Straight line, small error | ±0.05 rad (±3°) | 0.25-0.30 | Gentle correction |
| Moderate correction | ±0.20 rad (±11°) | 0.20-0.25 | Active steering |
| Sharp turn | ±0.40 rad (±23°) | 0.15-0.20 | Reduced speed in curve |
| Emergency correction | ±0.52 rad (±30°) | 0.10-0.15 | Maximum steering |

**Example Output:**
```
steering_angle: 0.310
motor_level: 0.25
```
This means: Steer right at 17.8°, drive at 25% speed.

---

## Control Algorithm Logic / 控制算法逻辑

### Lateral Control (Steering) / 横向控制（转向）

**Input:** `lateral_error`, `heading_error`, `curvature`  
**Output:** `steering_angle`

**Algorithm:**
```
steering_angle = lateral_PID(lateral_error, heading_error) + feedforward(curvature)
```

- **PID Control:** Corrects deviations from center line
- **Feedforward:** Anticipates steering needed for road curvature

**Constraints:**
- Limited to `[-max_steering_angle, +max_steering_angle]`
- Default: ±0.52 rad (±30°)

---

### Longitudinal Control (Speed) / 纵向控制（速度）

**Input:** Current velocity from `/odom`, Target velocity from parameters  
**Output:** `motor_level`

**Algorithm:**
```
velocity_error = target_velocity - current_velocity
motor_level = speed_PID(velocity_error)
```

**Speed Reduction in Curves:**
- When `|curvature| > 0.05`: Reduce target velocity
- Sharper curves → Lower speed

**Constraints:**
- Limited to `[0.0, 1.0]`
- Target velocity limited by `max_velocity` parameter

---

## Message Flow Diagram / 消息流程图

```
┌─────────────────────┐
│  Vision/Perception  │
│      System         │
└──────────┬──────────┘
           │
           │ /lane_deviation (10-30 Hz)
           │ - lateral_error
           │ - heading_error  
           │ - curvature
           ▼
┌─────────────────────┐      ┌──────────────┐
│                     │      │   Odometry   │
│  Control Algorithm  │◄─────│    Sensor    │
│   (This Package)    │      │              │
│                     │      └──────────────┘
└──────────┬──────────┘       /odom (20-50 Hz)
           │                  - current velocity
           │
           │ /motor_command (50 Hz)
           │ - steering_angle
           │ - motor_level
           ▼
┌─────────────────────┐
│  Hardware Interface │
│      Layer          │
└─────────────────────┘
```

---

## 中文版本

## 输入规范

### 1. 车道偏差输入

**话题名称：** `/lane_deviation`  
**消息类型：** `rusty_racer_interfaces/msg/LaneDeviation`  
**频率：** 10-30 Hz（推荐）

**消息结构：**
```
float32 lateral_error      # 横向偏差
float32 heading_error      # 航向角偏差
float32 curvature          # 道路曲率
```

**字段说明：**

| 字段 | 类型 | 单位 | 范围 | 描述 |
|------|------|------|------|------|
| `lateral_error` | float32 | 米 (m) | -0.5 到 0.5 | 车辆中心到车道中心的距离。正值 = 右偏，负值 = 左偏 |
| `heading_error` | float32 | 弧度 (rad) | -π 到 π | 车辆朝向与车道方向的角度差。正值 = 朝向右，负值 = 朝向左 |
| `curvature` | float32 | 1/米 (rad/m) | -1.0 到 1.0 | 道路曲率。正值 = 左转弯，负值 = 右转弯，0 = 直道 |

**典型数值：**
- 小偏差：`lateral_error` = ±0.05m（5厘米）
- 大偏差：`lateral_error` = ±0.25m（25厘米）
- 正常航向误差：`heading_error` = ±0.1 rad（±6°）
- 中等弯道：`curvature` = ±0.1 rad/m（半径约10米）

---

### 2. 里程计输入

**话题名称：** `/odom`  
**消息类型：** `nav_msgs/msg/Odometry`  
**频率：** 20-50 Hz（推荐）

**使用字段：**
```
twist.twist.linear.x    # 车辆前向速度
```

**字段说明：**

| 字段 | 类型 | 单位 | 范围 | 描述 |
|------|------|------|------|------|
| `twist.twist.linear.x` | float64 | m/s | 0 到 2.0 | 车辆当前前向速度 |

---

## 输出规范

### 电机命令输出

**话题名称：** `/motor_command`  
**消息类型：** `rusty_racer_interfaces/msg/MotorCommand`  
**频率：** 50 Hz（默认控制频率）

**消息结构：**
```
float32 steering_angle    # 期望转向角
float32 motor_level       # 电机驱动级别
```

**字段说明：**

| 字段 | 类型 | 单位 | 范围 | 描述 |
|------|------|------|------|------|
| `steering_angle` | float32 | 弧度 (rad) | -0.52 到 0.52 | 舵机转向角。正值 = 右转，负值 = 左转。受 `max_steering_angle` 参数限制 |
| `motor_level` | float32 | 归一化值 | 0.0 到 1.0 | 电机驱动级别。0.0 = 停止/滑行，1.0 = 最大前进速度 |

**物理含义：**
- **steering_angle：** 直接控制舵机角度
  - 例如：0.26 rad ≈ 15° 右转
  - 驾驶中典型范围：±0.3 rad（±17°）

- **motor_level：** 归一化油门命令
  - 0.0 = 无动力（车辆滑行）
  - 0.25 = 最大速度的 25%
  - 1.0 = 全速
  - 受 `max_velocity` 参数限制（默认：2.0 m/s）

**典型输出值：**

| 场景 | steering_angle | motor_level | 说明 |
|------|----------------|-------------|------|
| 直线行驶，小误差 | ±0.05 rad (±3°) | 0.25-0.30 | 轻微修正 |
| 中等修正 | ±0.20 rad (±11°) | 0.20-0.25 | 主动转向 |
| 急转弯 | ±0.40 rad (±23°) | 0.15-0.20 | 弯道减速 |
| 紧急修正 | ±0.52 rad (±30°) | 0.10-0.15 | 最大转向 |

---

## 控制算法逻辑

### 横向控制（转向）

**输入：** `lateral_error`、`heading_error`、`curvature`  
**输出：** `steering_angle`

**算法：**
```
steering_angle = lateral_PID(lateral_error, heading_error) + feedforward(curvature)
```

- **PID 控制：** 修正偏离中心线的误差
- **前馈控制：** 根据道路曲率预测所需转向

**约束：**
- 限制在 `[-max_steering_angle, +max_steering_angle]`
- 默认：±0.52 rad（±30°）

---

### 纵向控制（速度）

**输入：** `/odom` 中的当前速度，参数中的目标速度  
**输出：** `motor_level`

**算法：**
```
velocity_error = target_velocity - current_velocity
motor_level = speed_PID(velocity_error)
```

**弯道减速：**
- 当 `|curvature| > 0.05` 时：降低目标速度
- 弯道越急 → 速度越低

**约束：**
- 限制在 `[0.0, 1.0]`
- 目标速度受 `max_velocity` 参数限制

---

## Testing Examples / 测试示例

### Test 1: Straight Line Tracking / 直线跟踪测试
```bash
# Small deviation / 小偏差
ros2 topic pub /lane_deviation rusty_racer_interfaces/msg/LaneDeviation \
  "{lateral_error: 0.05, heading_error: 0.03, curvature: 0.0}" -r 10

# Expected output / 预期输出:
# steering_angle: ~0.05-0.10 rad
# motor_level: ~0.25
```

### Test 2: Large Correction / 大偏差修正测试
```bash
# Large deviation / 大偏差
ros2 topic pub /lane_deviation rusty_racer_interfaces/msg/LaneDeviation \
  "{lateral_error: 0.25, heading_error: 0.15, curvature: 0.0}" --once

# Expected output / 预期输出:
# steering_angle: ~0.30-0.50 rad
# motor_level: ~0.20 (reduced in correction)
```

### Test 3: Curve Following / 弯道跟踪测试
```bash
# Curve / 弯道
ros2 topic pub /lane_deviation rusty_racer_interfaces/msg/LaneDeviation \
  "{lateral_error: 0.0, heading_error: 0.0, curvature: 0.15}" -r 10

# Expected output / 预期输出:
# steering_angle: ~0.15-0.25 rad (following curve)
# motor_level: ~0.15-0.20 (reduced for curve)
```

---

## Notes / 注意事项

1. **Coordinate System / 坐标系统：**
   - Positive lateral_error: Vehicle is to the RIGHT of center / 车辆在中心线右侧
   - Positive steering_angle: Turn RIGHT / 向右转
   - Positive curvature: LEFT turn ahead / 前方左转弯

2. **Unit Consistency / 单位一致性：**
   - All distances in meters / 所有距离单位为米
   - All angles in radians / 所有角度单位为弧度
   - All velocities in m/s / 所有速度单位为 m/s

3. **Safety Limits / 安全限制：**
   - Steering is mechanically limited / 转向受机械限制
   - Motor level saturates at boundaries / 电机命令在边界处饱和
   - Speed reduces automatically in curves / 弯道自动减速

---

**Document Version:** v1.0  
**Last Updated:** 2025-11-30
