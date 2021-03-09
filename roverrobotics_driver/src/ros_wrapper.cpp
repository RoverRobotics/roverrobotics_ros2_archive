#include "ros_wrapper.hpp"
using namespace RoverRobotics;

RobotWrapper::RobotWrapper()
    : Node("roverrobotics",
           rclcpp::NodeOptions().use_intra_process_comms(false)) {
  RCLCPP_INFO(get_logger(), "Starting rover driver node");
  // closed_loop_ = false;

  double odometry_frequency = declare_parameter("odometry_frequency", 10.0);
  speed_topic_ = declare_parameter("speed_topic", "/cmd_vel/managed");
  estop_trigger_topic_ =
      declare_parameter("estop_trigger_topic", "/soft_estop/trigger");
  estop_reset_topic_ =
      declare_parameter("estop_reset_topic", "/soft_estop/reset");
  robot_status_topic_ =
      declare_parameter("robot_status_topic", "/robot_status");
  robot_status_frequency_ = declare_parameter("robot_status_frequency", 60);
  robot_info_request_topic_ =
      declare_parameter("robot_info_request_topic", "/robot_info/request");
  robot_info_topic_ = declare_parameter("robot_info_topic", "/robot_info");
  robot_type_ = declare_parameter("robot_type", "pro");
  trim_topic_ = declare_parameter("trim_topic", "/trim_event");
  device_port_ = declare_parameter("device_port", "/dev/rover");
  comm_type_ = declare_parameter("comm_type", "serial");
  estop_state_ = declare_parameter("estop_state", false);
  closed_loop_ = declare_parameter("closed_loop", false);
  linear_top_speed_ = declare_parameter("linear_top_speed", 2);
  angular_top_speed_ = declare_parameter("angular_top_speed", 2);
  odom_topic_ = declare_parameter("odom_topic", "/odom_raw");
  // Init 5 Subs
  speed_command_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
      speed_topic_, rclcpp::QoS(1),
      [=](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        velocity_event_callback(msg);
      });
  trim_event_subscriber_ = create_subscription<std_msgs::msg::Float32>(
      trim_topic_, rclcpp::QoS(3),
      [=](std_msgs::msg::Float32::ConstSharedPtr msg) {
        trim_event_callback(msg);
      });
  estop_trigger_subscriber_ = create_subscription<std_msgs::msg::Bool>(
      estop_trigger_topic_, rclcpp::QoS(2),
      [=](std_msgs::msg::Bool::ConstSharedPtr msg) {
        estop_trigger_event_callback(msg);
      });
  estop_reset_subscriber_ = create_subscription<std_msgs::msg::Bool>(
      estop_reset_topic_, rclcpp::QoS(2),
      [=](std_msgs::msg::Bool::ConstSharedPtr msg) {
        estop_reset_event_callback(msg);
      });
  robot_info__request_subscriber = create_subscription<std_msgs::msg::Bool>(
      estop_reset_topic_, rclcpp::QoS(2),
      [=](std_msgs::msg::Bool::ConstSharedPtr msg) {
        robot_info_request_callback(msg);
      });
  // Init Pub
  odometry_publisher_ =
      create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(4));
  robot_info_publisher = create_publisher<std_msgs::msg::Float32MultiArray>(
      robot_info_topic_, rclcpp::QoS(32));
  robot_status_publisher_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      robot_status_topic_, rclcpp::QoS(31));

  odom_frame_id_ = declare_parameter("odom_frame_id", "odom");
  odom_child_frame_id_ =
      declare_parameter("odom_child_frame_id", "base_footprint");
  odometry_timer_ =
      create_wall_timer(1s / odometry_frequency, [=]() { update_odom(); });
  robot_status_timer_ = create_wall_timer(1s / robot_status_frequency_,
                                          [=]() { publish_robot_status(); });
  float pi_p = declare_parameter("motor_control_p_gain", 0.40);
  float pi_i = declare_parameter("motor_control_i_gain", 0.70);
  float pi_d = declare_parameter("motor_control_d_gain", 0.00);
  pid_gains_ = {pi_p, pi_i, pi_d};
  auto now = get_clock()->now();
  if (closed_loop_) {
    RCLCPP_WARN(get_logger(),
                "Closed Loop Control is Enabled; Drive with CAUTION");
    RCLCPP_INFO(get_logger(), "PID is at P:%f I:%f D:%f", pi_p, pi_i, pi_d);
  } else if (!closed_loop_)
    // initialize connection to robot
    if (robot_type_ == "pro") {
      try {
        robot_ = std::make_unique<ProProtocolObject>(
            device_port_.c_str(), comm_type_, closed_loop_, pid_gains_);
      } catch (const std::exception &ex) {
        RCLCPP_FATAL(get_logger(), "Trouble connecting to robot %s",
                     std::to_string(errno));
        rclcpp::shutdown();
      }
    } else {
      rclcpp::shutdown();
    }
  RCLCPP_INFO(get_logger(), "Robot Setup Complete");
}

void RobotWrapper::publish_robot_info() {
  // RCLCPP_INFO(get_logger(), "Updating Robot Info");
  robot_data_ = robot_->info_request();
  std_msgs::msg::Float32MultiArray robot_info;
  robot_info.data.clear();
  robot_info.data.push_back(robot_data_.robot_guid);
  robot_info.data.push_back(robot_data_.robot_firmware);
  robot_info.data.push_back(robot_data_.robot_speed_limit);
  robot_info.data.push_back(robot_data_.robot_fan_speed);
  robot_info.data.push_back(robot_data_.robot_fault_flag);

  robot_info_publisher->publish(robot_info);
}

void RobotWrapper::publish_robot_status() {
  // RCLCPP_INFO(get_logger(), "Updating Robot Status");
  robot_data_ = robot_->status_request();
  std_msgs::msg::Float32MultiArray robot_status;
  robot_status.data.clear();
  // Motor Infos
  robot_status.data.push_back(robot_data_.motor1_id);
  robot_status.data.push_back(robot_data_.motor1_rpm);
  robot_status.data.push_back(robot_data_.motor1_current);
  robot_status.data.push_back(robot_data_.motor1_temp);
  robot_status.data.push_back(robot_data_.motor1_mos_temp);
  robot_status.data.push_back(robot_data_.motor2_id);
  robot_status.data.push_back(robot_data_.motor2_rpm);
  robot_status.data.push_back(robot_data_.motor2_current);
  robot_status.data.push_back(robot_data_.motor2_temp);
  robot_status.data.push_back(robot_data_.motor2_mos_temp);
  robot_status.data.push_back(robot_data_.motor3_id);
  robot_status.data.push_back(robot_data_.motor3_rpm);
  robot_status.data.push_back(robot_data_.motor3_current);
  robot_status.data.push_back(robot_data_.motor3_temp);
  robot_status.data.push_back(robot_data_.motor3_mos_temp);
  robot_status.data.push_back(robot_data_.motor4_id);
  robot_status.data.push_back(robot_data_.motor4_rpm);
  robot_status.data.push_back(robot_data_.motor4_current);
  robot_status.data.push_back(robot_data_.motor4_temp);
  robot_status.data.push_back(robot_data_.motor4_mos_temp);
  // Battery Infos
  robot_status.data.push_back(robot_data_.battery1_voltage);
  robot_status.data.push_back(robot_data_.battery2_voltage);
  robot_status.data.push_back(robot_data_.battery1_temp);
  robot_status.data.push_back(robot_data_.battery2_temp);
  robot_status.data.push_back(robot_data_.battery1_current);
  robot_status.data.push_back(robot_data_.battery2_current);
  robot_status.data.push_back(robot_data_.battery1_SOC);
  robot_status.data.push_back(robot_data_.battery2_SOC);
  robot_status.data.push_back(robot_data_.battery1_fault_flag);
  robot_status.data.push_back(robot_data_.battery2_fault_flag);

  // Flipper Infos
  robot_status.data.push_back(robot_data_.motor3_angle);
  robot_status.data.push_back(robot_data_.motor3_sensor1);
  robot_status.data.push_back(robot_data_.motor3_sensor2);
  robot_status_publisher_->publish(robot_status);
}

void RobotWrapper::update_odom() {
  // RCLCPP_INFO(get_logger(), "Updating Robot Odom");
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = odom_frame_id_;
  odom.child_frame_id = odom_child_frame_id_;
  odom.header.stamp = get_clock()->now();
  odom.twist.twist.linear.x = robot_data_.linear_vel;
  odom.twist.twist.angular.z = robot_data_.angular_vel;
  odometry_publisher_->publish(odom);
}

void RobotWrapper::velocity_event_callback(
    geometry_msgs::msg::Twist::ConstSharedPtr msg) {
  static double speeddata[3];
  speeddata[0] = msg->linear.x;
  speeddata[1] = msg->angular.z;
  speeddata[2] = msg->angular.y;
  robot_->set_robot_velocity(speeddata);
}

void RobotWrapper::trim_event_callback(
    std_msgs::msg::Float32::ConstSharedPtr &msg) {
  RCLCPP_INFO(get_logger(), "Trim Event triggered");
  robot_->update_drivetrim(msg->data);
}

void RobotWrapper::estop_trigger_event_callback(
    std_msgs::msg::Bool::ConstSharedPtr &msg) {
  if (msg->data == true) {
    RCLCPP_INFO(get_logger(), "Software Estop activated");
    estop_state_ = true;
    robot_->send_estop(estop_state_);
  }
}

void RobotWrapper::estop_reset_event_callback(
    std_msgs::msg::Bool::ConstSharedPtr &msg) {
  if (msg->data == true) {
    RCLCPP_INFO(get_logger(), "Software Estop deactivated");
    estop_state_ = false;
    robot_->send_estop(estop_state_);
  }
}

void RobotWrapper::robot_info_request_callback(
    std_msgs::msg::Bool::ConstSharedPtr &msg) {
  if (msg->data == true) {
    publish_robot_info();
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;

  auto rover_node = std::make_shared<RobotWrapper>();
  executor.add_node(rover_node);

  executor.spin();
  std::cerr << "Ros Node is running" << std::endl;
  return 0;
}