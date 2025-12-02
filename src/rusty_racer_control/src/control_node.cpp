/**
 * @file control_node.cpp
 * @brief Rusty Racer控制节点 - 速度域版本（带motor_level映射）
 *        Rusty Racer Control Node - Velocity Domain Version (with motor_level mapping)
 * 
 * 架构说明 / Architecture:
 *   速度域控制：v_ref → [PI] → v_cmd (速度)
 *   映射层：v_cmd → [Mapping] → motor_level (电机驱动)
 *   硬件执行：motor_level → 电机
 * 
 * @author zx
 * @date 2025-11
 * @version 2.0 - Velocity Domain with Mapping
 */

// ROS2头文件
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// 标准库
#include <cmath>
#include <memory>

// 从AF2保留的头文件（速度域版本）
#include "rusty_racer_control/laengsfuehrung_controller.h"  // PI速度控制（速度域）
#include "rusty_racer_control/common.h"                     // angleWrap等工具函数

// ============================================================================
// 速度到电机驱动的映射函数 / Speed to Motor Level Mapping
// ============================================================================
/**
 * @brief 将速度命令转换为电机驱动水平
 *        Convert velocity command to motor level
 * 
 * 映射方法 / Mapping Methods:
 *   方法1: 简单线性映射（推荐入门）
 *   方法2: 分段线性映射（更精确）
 *   方法3: 查找表（最精确，需要大量实测数据）
 * 
 * 当前使用：方法1（简单线性）
 * Currently using: Method 1 (Simple Linear)
 * 
 * @param v_cmd 速度命令 [m/s] / Velocity command [m/s]
 * @param v_max 最大速度 [m/s] / Maximum velocity [m/s]
 * @return motor_level 电机驱动水平 [0, 1]
 * 
 * @note 实测步骤 / Measurement Steps:
 *       1. 设置 motor_level = 1.0
 *       2. 在平直路面加速到稳态
 *       3. 记录稳态速度 → 这就是 v_max
 */
inline double speed_to_motor_level(double v_cmd, double v_max) 
{
    // ========================================================================
    // 方法1: 简单线性映射（当前使用）
    // Method 1: Simple Linear Mapping (Currently Used)
    // ========================================================================
    // 假设：motor_level 与速度成正比
    // Assumption: motor_level is proportional to velocity
    //
    // 映射公式 / Mapping formula:
    //   motor_level = v_cmd / v_max
    //
    // 示例 / Example:
    //   v_max = 1.5 m/s
    //   v_cmd = 0.5 m/s → motor_level = 0.5/1.5 = 0.33
    //   v_cmd = 1.0 m/s → motor_level = 1.0/1.5 = 0.67
    //   v_cmd = 1.5 m/s → motor_level = 1.5/1.5 = 1.00
    
    double motor_level = v_cmd / v_max;
    
    // 限幅到 [0, 1]
    motor_level = std::max(0.0, std::min(motor_level, 1.0));
    
    return motor_level;
    
    // ========================================================================
    // 方法2: 分段线性映射（如果方法1不够精确，取消注释使用）
    // Method 2: Piecewise Linear Mapping (Uncomment if Method 1 is not accurate)
    // ========================================================================
    /*
    // 实测数据点（需要通过实验获得）
    // Measured data points (need to obtain through experiments)
    //
    // 格式 / Format: {速度 [m/s], motor_level}
    // 示例数据（需要替换为你的实测值）/ Example data (replace with your measurements)
    
    if (v_cmd <= 0.0) return 0.0;
    
    // 起步段（静摩擦较大）/ Starting segment (high static friction)
    if (v_cmd <= 0.2) {
        // 0-0.2 m/s 需要 0-0.15 motor_level
        return 0.15 * (v_cmd / 0.2);
    }
    
    // 低速段 / Low speed segment
    if (v_cmd <= 0.5) {
        // 0.2-0.5 m/s 需要 0.15-0.35 motor_level
        return 0.15 + 0.20 * ((v_cmd - 0.2) / 0.3);
    }
    
    // 中速段 / Medium speed segment
    if (v_cmd <= 1.0) {
        // 0.5-1.0 m/s 需要 0.35-0.65 motor_level
        return 0.35 + 0.30 * ((v_cmd - 0.5) / 0.5);
    }
    
    // 高速段 / High speed segment
    if (v_cmd <= v_max) {
        // 1.0-v_max m/s 需要 0.65-1.00 motor_level
        return 0.65 + 0.35 * ((v_cmd - 1.0) / (v_max - 1.0));
    }
    
    // 超过最大速度 / Exceeds maximum velocity
    return 1.0;
    */
}

// ============================================================================
// 从AF1集成的横向控制器类 / Lateral Controller Class from AF1
// ============================================================================
class LateralController
{
public:
  LateralController(double v, double l, double l_h, double kp, double kd)
    : v_(v), l_(l), l_h_(l_h), kp_(kp), kd_(kd)
  {
    if (v_ < 0.1) v_ = 0.1;
  }

  void updateVelocity(double v) 
  {
    v_ = std::max(0.1, v);
  }

  double compute(double y, double phi_k)
  {
    // PD控制: φL* = -kp·y - (kp+kd)·φK
    double p_term = -kp_ * y - kp_ * phi_k;
    double d_term = -kd_ * phi_k;
    double steering_input = p_term + d_term;
    
    // 限幅 ±30°
    const double max_steering = M_PI / 6.0;
    steering_input = std::max(-max_steering, std::min(steering_input, max_steering));
    
    // arctan预滤波器
    double steering_angle = std::atan(steering_input);
    
    return steering_angle;
  }

  double getVelocity() const { return v_; }

private:
  double v_, l_, l_h_, kp_, kd_;
};

// ============================================================================
// 临时消息定义 / Temporary Message Definitions
// ============================================================================
// ⚠️ 实际部署时应使用自定义的MotorCommand消息
// ⚠️ Should use custom MotorCommand message in actual deployment
//
// MotorCommand.msg应该包含 / Should contain:
//   float32 steering_angle  # 转向角 [rad]
//   float32 motor_level     # 电机驱动水平 [-1, 1]

// LaneDetection临时映射
using LaneDetection = geometry_msgs::msg::Twist;
// 临时MotorCommand映射
using MotorCommand = geometry_msgs::msg::Twist;

// ============================================================================
// 控制节点类 / Control Node Class
// ============================================================================
class ControlNode : public rclcpp::Node
{
public:
  ControlNode() : Node("control_node")
  {
    // ========================================================================
    // 车辆参数 / Vehicle Parameters
    // ========================================================================
    const double v_init = 0.5;
    const double l = 0.257;       // 轴距 [m]，需实测
    const double l_h = 0.0;       // Li Yixuan确认：视觉输出后轮偏差
    
    // ========================================================================
    // 横向控制参数（来自AF1）/ Lateral Control Parameters (from AF1)
    // ========================================================================
    const double lat_kp = 1.5;
    const double lat_kd = 0.8;
    
    lateral_controller_ = std::make_unique<LateralController>(
      v_init, l, l_h, lat_kp, lat_kd
    );
    
    // ========================================================================
    // 纵向控制参数（速度域）/ Longitudinal Control Parameters (Velocity Domain)
    // ========================================================================
    // ⚠️ 重要修改 / Important Changes:
    //    原来输出 u [0,1]（油门）
    //    现在输出 v_cmd [m/s]（速度命令）
    
    pi_params_.Kp = 1.0;      // 速度比例增益 / Velocity proportional gain
                              // 物理意义：误差1m/s → 命令增加1.0m/s
    
    pi_params_.Ki = 1.5;      // 速度积分增益 / Velocity integral gain
                              // 物理意义：消除稳态误差
    
    pi_params_.v_min = 0.0;   // 最小速度 [m/s] / Minimum velocity
    
    pi_params_.v_max = 1.5;   // 最大速度 [m/s] / Maximum velocity
                              // ⚠️ 需要实测 / Needs measurement:
                              //    motor_level=1.0时的稳态速度
                              //    Steady-state speed at motor_level=1.0
    
    pi_state_ = init_pi();
    
    // 目标速度 / Target velocity
    v_ref_ = 0.5;  // 阶段1固定速度 0.5 m/s
    
    // 初始化状态
    current_v_ = 0.0;
    current_psi_k_ = 0.0;
    last_update_time_ = this->now();
    
    // ========================================================================
    // 创建订阅者 / Create Subscribers
    // ========================================================================
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&ControlNode::odomCallback, this, std::placeholders::_1)
    );
    
    lane_sub_ = this->create_subscription<LaneDetection>(
      "/perception/lane_detection", 10,
      std::bind(&ControlNode::laneCallback, this, std::placeholders::_1)
    );
    
    // ========================================================================
    // 创建发布者 / Create Publisher
    // ========================================================================
    motor_cmd_pub_ = this->create_publisher<MotorCommand>(
      "/control/motor_command", 10
    );
    
    // ========================================================================
    // 启动日志 / Startup Logging
    // ========================================================================
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "控制节点已启动 - 速度域版本");
    RCLCPP_INFO(this->get_logger(), "Control Node Started - Velocity Domain");
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "架构 / Architecture:");
    RCLCPP_INFO(this->get_logger(), "  横向 / Lateral: AF1 PD+预滤波");
    RCLCPP_INFO(this->get_logger(), "  纵向 / Longitudinal: PI速度控制 + 映射");
    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
    RCLCPP_INFO(this->get_logger(), "纵向控制流程 / Longitudinal Control Flow:");
    RCLCPP_INFO(this->get_logger(), "  输入 / Input: v_ref=%.2f m/s, v_k (测量)", v_ref_);
    RCLCPP_INFO(this->get_logger(), "  PI输出 / PI Output: v_cmd [m/s]");
    RCLCPP_INFO(this->get_logger(), "  映射 / Mapping: motor_level = v_cmd/%.2f", pi_params_.v_max);
    RCLCPP_INFO(this->get_logger(), "  硬件输入 / Hardware: motor_level [0,1]");
    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
    RCLCPP_INFO(this->get_logger(), "PI参数 / PI Parameters:");
    RCLCPP_INFO(this->get_logger(), "  Kp = %.2f", pi_params_.Kp);
    RCLCPP_INFO(this->get_logger(), "  Ki = %.2f", pi_params_.Ki);
    RCLCPP_INFO(this->get_logger(), "  v_max = %.2f m/s (需实测)", pi_params_.v_max);
    RCLCPP_INFO(this->get_logger(), "========================================");
  }

private:
  // ==========================================================================
  // 里程计回调 / Odometry Callback
  // ==========================================================================
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_v_ = msg->twist.twist.linear.x;
    current_psi_k_ = extractYawFromQuaternion(msg->pose.pose.orientation);
    lateral_controller_->updateVelocity(current_v_);
    last_update_time_ = this->now();
  }
  
  // ==========================================================================
  // 车道检测回调 - 主控制循环 / Lane Detection Callback - Main Control Loop
  // ==========================================================================
  void laneCallback(const LaneDetection::SharedPtr msg)
  {
    // ========================================================================
    // 提取数据 / Extract Data
    // ========================================================================
    double y = msg->linear.x;           // 横向偏差 [m]
    double theta_path = msg->angular.z; // 路径航向角 [rad]
    
    // ========================================================================
    // 计算采样时间 / Calculate Sampling Time
    // ========================================================================
    auto current_time = this->now();
    double dt = (current_time - last_update_time_).seconds();
    if (dt <= 0.0 || dt > 0.1) dt = 0.02;
    
    // ========================================================================
    // 计算航向偏差 / Calculate Heading Deviation
    // ========================================================================
    double phi_k = angleWrap(theta_path - current_psi_k_);
    
    // ========================================================================
    // 纵向控制：速度域 + 映射 / Longitudinal Control: Velocity Domain + Mapping
    // ========================================================================
    // 步骤1: PI控制器输出速度命令
    // Step 1: PI controller outputs velocity command
    double v_cmd = pi_step(pi_params_, pi_state_, v_ref_, current_v_, dt);
    
    // 步骤2: 映射到电机驱动水平
    // Step 2: Map to motor level
    double motor_level = speed_to_motor_level(v_cmd, pi_params_.v_max);
    
    // ========================================================================
    // 横向控制：角度域 / Lateral Control: Angle Domain
    // ========================================================================
    double delta = lateral_controller_->compute(y, phi_k);
    
    // ========================================================================
    // 发布命令 / Publish Commands
    // ========================================================================
    auto cmd = MotorCommand();
    cmd.linear.x = motor_level;     // 临时：用linear.x传motor_level
    cmd.angular.z = delta;          // 临时：用angular.z传steering_angle
    motor_cmd_pub_->publish(cmd);
    
    // ⚠️ 实际部署时应该用：
    // ⚠️ In actual deployment should use:
    // MotorCommand cmd;
    // cmd.motor_level = motor_level;
    // cmd.steering_angle = delta;
    // motor_cmd_pub_->publish(cmd);
    
    // ========================================================================
    // 控制日志 / Control Logging
    // ========================================================================
    RCLCPP_INFO(this->get_logger(),
        "y=%+.3fm φ=%+.1f° | v_k=%.3f v_cmd=%.3f m_lv=%.3f | δ=%+.1f°",
        y,                          // 横向偏差
        phi_k * 180.0 / M_PI,      // 航向偏差(度)
        current_v_,                 // 当前速度 [m/s]
        v_cmd,                      // PI输出的速度命令 [m/s]
        motor_level,                // 映射后的电机驱动水平 [0,1]
        delta * 180.0 / M_PI       // 转向角(度)
    );
    
    last_update_time_ = current_time;
  }
  
  // ==========================================================================
  // 从四元数提取航向角 / Extract Yaw from Quaternion
  // ==========================================================================
  double extractYawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
  {
    tf2::Quaternion tf_quat(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 mat(tf_quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    return yaw;
  }
  
  // ==========================================================================
  // 成员变量 / Member Variables
  // ==========================================================================
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<LaneDetection>::SharedPtr lane_sub_;
  rclcpp::Publisher<MotorCommand>::SharedPtr motor_cmd_pub_;
  
  std::unique_ptr<LateralController> lateral_controller_;
  PIParams pi_params_;
  PIState pi_state_;
  
  double v_ref_;
  double current_v_;
  double current_psi_k_;
  rclcpp::Time last_update_time_;
};

// ============================================================================
// 主函数 / Main Function
// ============================================================================
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}