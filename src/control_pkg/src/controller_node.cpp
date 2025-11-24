#include "rclcpp/rclcpp.hpp"
#include "rusty_racer_interfaces/msg/lane_deviation.hpp"
#include "rusty_racer_interfaces/msg/motor_command.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <algorithm>
#include <cmath>

class ControllerNode : public rclcpp::Node {
public:
  ControllerNode() : Node("controller_node") {
    // 声明参数
    this->declare_parameter("control_frequency", 50.0);
    this->declare_parameter("target_velocity", 0.5);
    this->declare_parameter("wheelbase", 0.258);
    this->declare_parameter("max_steering_angle", 0.52);
    this->declare_parameter("max_velocity", 2.0);
    this->declare_parameter("lateral_pid.kp", 1.0);
    this->declare_parameter("lateral_pid.ki", 0.0);
    this->declare_parameter("lateral_pid.kd", 0.1);
    
    // 获取参数
    control_frequency_ = this->get_parameter("control_frequency").as_double();
    target_velocity_ = this->get_parameter("target_velocity").as_double();
    wheelbase_ = this->get_parameter("wheelbase").as_double();
    max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
    max_velocity_ = this->get_parameter("max_velocity").as_double();
    lateral_pid_kp_ = this->get_parameter("lateral_pid.kp").as_double();
    lateral_pid_ki_ = this->get_parameter("lateral_pid.ki").as_double();
    lateral_pid_kd_ = this->get_parameter("lateral_pid.kd").as_double();
    
    // 订阅车道偏差
    lane_deviation_sub_ = this->create_subscription<rusty_racer_interfaces::msg::LaneDeviation>(
      "/lane_deviation", 10,
      std::bind(&ControllerNode::lane_deviation_callback, this, std::placeholders::_1)
    );
    
    // 订阅里程计（可选，用于速度反馈）
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&ControllerNode::odom_callback, this, std::placeholders::_1)
    );
    
    // 发布电机命令
    motor_command_pub_ = this->create_publisher<rusty_racer_interfaces::msg::MotorCommand>(
      "/motor_command", 10
    );
    
    RCLCPP_INFO(this->get_logger(), "Controller node initialized");
    RCLCPP_INFO(this->get_logger(), "  Control frequency: %.1f Hz", control_frequency_);
    RCLCPP_INFO(this->get_logger(), "  Target velocity: %.2f m/s", target_velocity_);
    RCLCPP_INFO(this->get_logger(), "  PID gains - Kp: %.2f, Ki: %.2f, Kd: %.2f", 
                lateral_pid_kp_, lateral_pid_ki_, lateral_pid_kd_);
  }

private:
  void lane_deviation_callback(const rusty_racer_interfaces::msg::LaneDeviation::SharedPtr msg) {
    // ========== 横向控制 ==========
    // PID控制：转向角 = Kp*横向误差 + Kd*航向误差 + 前馈项
    double lateral_control = lateral_pid_kp_ * msg->lateral_error 
                            + lateral_pid_kd_ * msg->heading_error;
    
    // 前馈控制：基于Ackermann转向几何
    // steering_angle = atan(wheelbase * curvature)
    double feedforward = std::atan(wheelbase_ * msg->curvature);
    
    // 总转向角
    double steering_angle = lateral_control + feedforward;
    
    // 限幅
    steering_angle = std::clamp(steering_angle, -max_steering_angle_, max_steering_angle_);
    
    // ========== 纵向控制 ==========
    // 简单速度控制：将目标速度转换为motor_level
    // motor_level = target_velocity / max_velocity
    double motor_level = target_velocity_ / max_velocity_;
    
    // 安全限制：转弯时降低速度
    double abs_curvature = std::abs(msg->curvature);
    if (abs_curvature > 0.05) {  // 曲率 > 0.05 (半径 < 20m)
      motor_level *= 0.7;  // 降低到70%速度
    }
    
    // 限幅到[0, 1]
    motor_level = std::clamp(motor_level, 0.0, 1.0);
    
    // ========== 发布控制命令 ==========
    auto command = rusty_racer_interfaces::msg::MotorCommand();
    command.steering_angle = static_cast<float>(steering_angle);
    command.motor_level = static_cast<float>(motor_level);
    
    motor_command_pub_->publish(command);
    
    // 日志输出
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Control output - Steering: %.3f rad (%.1f°), Motor: %.2f, "
      "Lat_err: %.3f, Head_err: %.3f, Curv: %.3f",
      steering_angle, steering_angle * 180.0 / M_PI, motor_level,
      msg->lateral_error, msg->heading_error, msg->curvature);
  }
  
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // 存储当前速度（未来可用于纵向控制）
    current_velocity_ = std::sqrt(
      msg->twist.twist.linear.x * msg->twist.twist.linear.x +
      msg->twist.twist.linear.y * msg->twist.twist.linear.y
    );
  }
  
  // 参数
  double control_frequency_;
  double target_velocity_;
  double wheelbase_;
  double max_steering_angle_;
  double max_velocity_;
  double lateral_pid_kp_;
  double lateral_pid_ki_;
  double lateral_pid_kd_;
  
  // 状态
  double current_velocity_ = 0.0;
  
  // ROS2接口
  rclcpp::Subscription<rusty_racer_interfaces::msg::LaneDeviation>::SharedPtr lane_deviation_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<rusty_racer_interfaces::msg::MotorCommand>::SharedPtr motor_command_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
