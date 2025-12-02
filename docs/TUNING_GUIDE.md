# PID Gain Tuning Guide / PID 增益调试指南

## English Version

### Quick Start

This guide explains how to tune the PID controller gains for optimal vehicle performance.

---

## What is PID Control? / PID 控制简介

PID (Proportional-Integral-Derivative) is a control algorithm that adjusts the vehicle's steering and speed based on errors.

**Three Components:**
- **P (Proportional):** Reacts to current error - larger error → stronger correction
- **I (Integral):** Eliminates persistent steady-state errors
- **D (Derivative):** Dampens oscillations - predicts future error

---

## Parameters Location / 参数位置

Edit the file: `config/controller_params.yaml`

```yaml
lateral_pid:       # Steering control
  kp: 1.0         # Proportional gain
  ki: 0.0         # Integral gain (currently disabled)
  kd: 0.1         # Derivative gain

longitudinal_pid:  # Speed control
  kp: 1.0
  ki: 0.0
  kd: 0.05
```

---

## How to Modify Parameters / 如何修改参数

### Step 1: Edit Configuration File
```bash
cd ~/ros2_ws/src/control_pkg/config
nano controller_params.yaml
```

### Step 2: Modify Values
Change the numbers for kp, ki, or kd.

### Step 3: Save and Exit
- Press `Ctrl + O` to save
- Press `Enter` to confirm
- Press `Ctrl + X` to exit

### Step 4: Restart Node
Stop and restart the control node (no rebuild needed).

---

## Tuning Process / 调试流程

### Phase 1: Lateral Control (Steering) / 横向控制调试

#### Step 1: Adjust Kp (Proportional Gain)

**Goal:** Make the vehicle respond to errors quickly but smoothly.

**Test Command:**
```bash
# Test with 5cm deviation
ros2 topic pub /lane_deviation rusty_racer_interfaces/msg/LaneDeviation \
  "{lateral_error: 0.05, heading_error: 0.0, curvature: 0.0}" --once
```

**Tuning Table:**

| Kp Value | Behavior | Action |
|----------|----------|--------|
| Too small (e.g., 0.5) | Slow response, large steady-state error | ❌ Increase Kp |
| **Good (e.g., 1.0)** | Quick response, minimal oscillation | ✅ Keep or fine-tune |
| Too large (e.g., 2.0) | Aggressive, oscillates | ❌ Decrease Kp |

**Observation:**
- Check steering angle in terminal output
- Vehicle should return to center smoothly
- No excessive back-and-forth motion

---

#### Step 2: Adjust Kd (Derivative Gain)

**Goal:** Reduce oscillations and smooth out the response.

**Test Command:**
```bash
# Test with large sudden deviation
ros2 topic pub /lane_deviation rusty_racer_interfaces/msg/LaneDeviation \
  "{lateral_error: 0.25, heading_error: 0.15, curvature: 0.0}" --once
```

**Tuning Table:**

| Kd Value | Behavior | Action |
|----------|----------|--------|
| Too small (e.g., 0.0) | Oscillates, overshoots | ❌ Increase Kd |
| **Good (e.g., 0.1-0.2)** | Smooth response, minimal overshoot | ✅ Keep |
| Too large (e.g., 0.5) | Sluggish, slow to react | ❌ Decrease Kd |

**Observation:**
- Count oscillations (should be ≤ 2)
- Check if vehicle overshoots center line
- Response should be smooth but not too slow

---

#### Step 3: Decide on Ki (Integral Gain)

**Current Setting:** Ki = 0.0 (disabled)

**When to Enable Ki:**
- ✅ If there's persistent small error that won't go away
- ✅ If vehicle consistently runs slightly off-center
- ❌ NOT recommended for initial tuning

**If Needed:**
```yaml
lateral_pid:
  kp: 1.0
  ki: 0.01    # Start with very small value!
  kd: 0.1
```

**Warning:** Large Ki can cause overshoot and instability.

---

### Phase 2: Longitudinal Control (Speed) / 纵向控制调试

#### Speed Control Tuning

**Test Command:**
```bash
# Simulate current velocity at 0.3 m/s
ros2 topic pub /odom nav_msgs/msg/Odometry \
  "{twist: {twist: {linear: {x: 0.3}}}}" -r 10
```

**Observation:**
- Target velocity is 0.5 m/s (from config)
- Motor_level should increase to accelerate
- Should reach target smoothly without jerking

**Tuning Table:**

| Kp Value | Behavior | Action |
|----------|----------|--------|
| 0.5 | Slow acceleration | ❌ Increase Kp |
| **1.0** | **Balanced** | ✅ Keep |
| 2.0 | Fast but may overshoot | ⚠️ Test carefully |

**Note:** Speed control is typically more conservative than steering.

---

## Test Scenarios / 测试场景

### Test A: Small Deviation (Direct Line)
```bash
ros2 topic pub /lane_deviation rusty_racer_interfaces/msg/LaneDeviation \
  "{lateral_error: 0.05, heading_error: 0.03, curvature: 0.0}" -r 10
```
**Success Criteria:**
- Returns to center within 2 seconds
- Steering angle < ±10°
- No oscillation

---

### Test B: Large Deviation Recovery
```bash
ros2 topic pub /lane_deviation rusty_racer_interfaces/msg/LaneDeviation \
  "{lateral_error: 0.25, heading_error: 0.15, curvature: 0.0}" --once
```
**Success Criteria:**
- Corrects quickly but not aggressively
- Maximum 1-2 small oscillations
- Stabilizes within 5 seconds

---

### Test C: Curve Following
```bash
ros2 topic pub /lane_deviation rusty_racer_interfaces/msg/LaneDeviation \
  "{lateral_error: 0.0, heading_error: 0.0, curvature: 0.15}" -r 10
```
**Success Criteria:**
- Steering angle adjusts to curve
- Speed reduces appropriately
- Smooth tracking, no jitter

---

## Parameter Effects Summary / 参数效果总结

| Parameter | Increase Effect | Decrease Effect |
|-----------|----------------|-----------------|
| **Kp** | Faster response, may oscillate | Slower response, larger error |
| **Ki** | Eliminates steady error, may overshoot | Persistent small errors remain |
| **Kd** | Reduces oscillation, smoother | More overshoot, less stable |

---

## Recommended Tuning Order / 推荐调试顺序

```
1. Start with default parameters
   ↓
2. Tune lateral Kp (keep Ki=0, Kd=0.1)
   ↓
3. Tune lateral Kd (fix Kp from step 2)
   ↓
4. Decide if Ki is needed (usually not initially)
   ↓
5. Tune longitudinal Kp if needed
   ↓
6. Test on actual vehicle
   ↓
7. Fine-tune based on real performance
```

---

## Troubleshooting / 故障排查

### Problem: Vehicle Oscillates (Zig-Zag Motion)
**Cause:** Kp too high or Kd too low  
**Solution:** 
- Decrease Kp by 20-30%
- Increase Kd by 50-100%

### Problem: Slow to Correct Errors
**Cause:** Kp too low  
**Solution:** Increase Kp by 30-50%

### Problem: Overshoots Center Line
**Cause:** Kd too low  
**Solution:** Increase Kd

### Problem: Persistent Small Offset
**Cause:** No integral action  
**Solution:** Enable Ki with small value (0.01-0.05)

### Problem: Steering Angle Saturates (±0.52 rad)
**Cause:** Gains too high or error too large  
**Solution:** Check sensor input, reduce gains slightly

---

## Recording Results / 记录调试结果

Keep a log of parameter changes and results:

```
Date: 2025-11-30
Test #1

Parameters:
- lateral_pid.kp: 1.5
- lateral_pid.kd: 0.2

Test A (small deviation):
- Convergence time: 1.5 sec
- Oscillations: 0
- Result: ✅ Good

Test B (large deviation):
- Convergence time: 3.8 sec
- Oscillations: 1 (small)
- Max steering: 26°
- Result: ✅ Acceptable

Conclusion: Parameters working well, ready for vehicle test
```

---

## Tips for Success / 成功技巧

1. **Change one parameter at a time** - easier to see the effect
2. **Make small adjustments** - change by 20-30% each time
3. **Test with same scenarios** - for fair comparison
4. **Document everything** - you can always revert
5. **Be patient** - tuning takes time and iteration

---

## 中文版本

## 快速入门

本指南说明如何调整 PID 控制器增益以获得最佳车辆性能。

---

## PID 控制简介

PID（比例-积分-微分）是一种根据误差调整车辆转向和速度的控制算法。

**三个组成部分：**
- **P（比例）：** 对当前误差做出反应 - 误差越大 → 修正越强
- **I（积分）：** 消除持续的稳态误差
- **D（微分）：** 抑制振荡 - 预测未来误差

---

## 参数位置

编辑文件：`config/controller_params.yaml`

```yaml
lateral_pid:       # 转向控制
  kp: 1.0         # 比例增益
  ki: 0.0         # 积分增益（当前禁用）
  kd: 0.1         # 微分增益

longitudinal_pid:  # 速度控制
  kp: 1.0
  ki: 0.0
  kd: 0.05
```

---

## 如何修改参数

### 步骤 1：编辑配置文件
```bash
cd ~/ros2_ws/src/control_pkg/config
nano controller_params.yaml
```

### 步骤 2：修改数值
更改 kp、ki 或 kd 的数字。

### 步骤 3：保存并退出
- 按 `Ctrl + O` 保存
- 按 `Enter` 确认
- 按 `Ctrl + X` 退出

### 步骤 4：重启节点
停止并重启控制节点（无需重新编译）。

---

## 调试流程

### 阶段 1：横向控制（转向）调试

#### 步骤 1：调整 Kp（比例增益）

**目标：** 让车辆快速但平稳地响应误差。

**测试命令：**
```bash
# 用 5cm 偏差测试
ros2 topic pub /lane_deviation rusty_racer_interfaces/msg/LaneDeviation \
  "{lateral_error: 0.05, heading_error: 0.0, curvature: 0.0}" --once
```

**调试表：**

| Kp 值 | 行为 | 操作 |
|-------|------|------|
| 太小（如 0.5） | 响应慢，稳态误差大 | ❌ 增大 Kp |
| **合适（如 1.0）** | 响应快，振荡小 | ✅ 保持或微调 |
| 太大（如 2.0） | 激进，振荡 | ❌ 减小 Kp |

**观察：**
- 检查终端输出的转向角
- 车辆应平稳回到中心线
- 无过度来回摆动

---

#### 步骤 2：调整 Kd（微分增益）

**目标：** 减少振荡，使响应更平滑。

**测试命令：**
```bash
# 用大的突然偏差测试
ros2 topic pub /lane_deviation rusty_racer_interfaces/msg/LaneDeviation \
  "{lateral_error: 0.25, heading_error: 0.15, curvature: 0.0}" --once
```

**调试表：**

| Kd 值 | 行为 | 操作 |
|-------|------|------|
| 太小（如 0.0） | 振荡，过冲 | ❌ 增大 Kd |
| **合适（如 0.1-0.2）** | 响应平滑，过冲小 | ✅ 保持 |
| 太大（如 0.5） | 迟钝，反应慢 | ❌ 减小 Kd |

**观察：**
- 计算振荡次数（应 ≤ 2）
- 检查车辆是否冲过中心线
- 响应应平滑但不要太慢

---

#### 步骤 3：决定 Ki（积分增益）

**当前设置：** Ki = 0.0（禁用）

**何时启用 Ki：**
- ✅ 如果有持续的小误差无法消除
- ✅ 如果车辆始终稍微偏离中心
- ❌ 初期调试不推荐

**如果需要：**
```yaml
lateral_pid:
  kp: 1.0
  ki: 0.01    # 从很小的值开始！
  kd: 0.1
```

**警告：** Ki 过大会导致过冲和不稳定。

---

### 阶段 2：纵向控制（速度）调试

#### 速度控制调试

**测试命令：**
```bash
# 模拟当前速度为 0.3 m/s
ros2 topic pub /odom nav_msgs/msg/Odometry \
  "{twist: {twist: {linear: {x: 0.3}}}}" -r 10
```

**观察：**
- 目标速度为 0.5 m/s（来自配置）
- motor_level 应增加以加速
- 应平稳达到目标，无抖动

**调试表：**

| Kp 值 | 行为 | 操作 |
|-------|------|------|
| 0.5 | 加速慢 | ❌ 增大 Kp |
| **1.0** | **平衡** | ✅ 保持 |
| 2.0 | 快但可能过冲 | ⚠️ 谨慎测试 |

**注意：** 速度控制通常比转向控制更保守。

---

## 测试场景

### 测试 A：小偏差（直线）
```bash
ros2 topic pub /lane_deviation rusty_racer_interfaces/msg/LaneDeviation \
  "{lateral_error: 0.05, heading_error: 0.03, curvature: 0.0}" -r 10
```
**成功标准：**
- 2 秒内回到中心
- 转向角 < ±10°
- 无振荡

---

### 测试 B：大偏差恢复
```bash
ros2 topic pub /lane_deviation rusty_racer_interfaces/msg/LaneDeviation \
  "{lateral_error: 0.25, heading_error: 0.15, curvature: 0.0}" --once
```
**成功标准：**
- 快速但不激进地修正
- 最多 1-2 次小振荡
- 5 秒内稳定

---

### 测试 C：弯道跟踪
```bash
ros2 topic pub /lane_deviation rusty_racer_interfaces/msg/LaneDeviation \
  "{lateral_error: 0.0, heading_error: 0.0, curvature: 0.15}" -r 10
```
**成功标准：**
- 转向角适应弯道
- 速度适当降低
- 平稳跟踪，无抖动

---

## 参数效果总结

| 参数 | 增大效果 | 减小效果 |
|------|---------|---------|
| **Kp** | 响应快，可能振荡 | 响应慢，误差大 |
| **Ki** | 消除稳态误差，可能过冲 | 持续小误差 |
| **Kd** | 减少振荡，更平滑 | 过冲多，不稳定 |

---

## 推荐调试顺序

```
1. 从默认参数开始
   ↓
2. 调试横向 Kp（保持 Ki=0, Kd=0.1）
   ↓
3. 调试横向 Kd（固定步骤 2 的 Kp）
   ↓
4. 决定是否需要 Ki（通常初期不需要）
   ↓
5. 如需要调试纵向 Kp
   ↓
6. 在实际车辆上测试
   ↓
7. 根据实际性能微调
```

---

## 故障排查

### 问题：车辆振荡（之字形运动）
**原因：** Kp 太高或 Kd 太低  
**解决：**
- 减小 Kp 20-30%
- 增大 Kd 50-100%

### 问题：修正误差慢
**原因：** Kp 太低  
**解决：** 增大 Kp 30-50%

### 问题：冲过中心线
**原因：** Kd 太低  
**解决：** 增大 Kd

### 问题：持续小偏移
**原因：** 无积分作用  
**解决：** 启用 Ki，使用小值（0.01-0.05）

### 问题：转向角饱和（±0.52 rad）
**原因：** 增益太高或误差太大  
**解决：** 检查传感器输入，略微减小增益

---

## 成功技巧

1. **一次只改一个参数** - 更容易看到效果
2. **小幅度调整** - 每次改变 20-30%
3. **用相同场景测试** - 便于公平比较
4. **记录所有内容** - 可以随时回退
5. **保持耐心** - 调试需要时间和迭代

---

**文档版本：** v1.0  
**最后更新：** 2025-11-30
