#include "../include/ros_wrapper.hpp"
using namespace RoverRobotics;

RobotWrapper::RobotWrapper() : Node("roverrobotics", rclcpp::NodeOptions().use_intra_process_comms(true))
{
    RCLCPP_INFO(get_logger(), "Starting rover driver node");
    // closed_loop_ = false;
    robot_data_ = {};
    double odometry_frequency = declare_parameter("odometry_frequency", 10.0);
    tmr_odometry = create_wall_timer(1s / odometry_frequency, [=]() { update_odom(); });
    speed_topic_ = declare_parameter("speed_topic", "/cmd_vel/managed");
    estop_trigger_topic_ = declare_parameter("estop_trigger_topic", "/soft_estop/trigger");
    estop_reset_topic_ = declare_parameter("estop_reset_topic", "/soft_estop/reset");
    robot_status_topic_ = declare_parameter("robot_status_topic", "/robot_status");
    robot_status_frequency_ = declare_parameter("robot_status_frequency", 30);
    robot_info_request_topic_ = declare_parameter("robot_info_request_topic", "/robot_info/request");
    robot_info_topic_ = declare_parameter("robot_info_topic", "/robot_info");
    robot_type_ = declare_parameter("robot_type", "pro");
    trim_topic_ = declare_parameter("trim_topic", "/trim_increment");
    device_port_ = declare_parameter("device_port", "/dev/rover");
    comm_type_ = declare_parameter("comm_type", "serial");
    estop_state_ = declare_parameter("estop_state", false);
    closed_loop_ = declare_parameter("closed_loop", false);
    publish_tf = declare_parameter("publish_tf", false);
    linear_top_speed_ = declare_parameter("linear_top_speed", 2);
    angular_top_speed_ = declare_parameter("angular_top_speed", 2);
    speed_command_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
        speed_topic_, rclcpp::QoS(1), [=](geometry_msgs::msg::Twist::ConstSharedPtr msg) { velocity_callback(msg); });
    trim_command_subscriber_ = create_subscription<std_msgs::msg::Float32>(
        trim_topic_, rclcpp::QoS(2), [=](std_msgs::msg::Float32::ConstSharedPtr msg) { trim_callback(msg); });
    pub_odom = create_publisher<nav_msgs::msg::Odometry>("odom_raw", rclcpp::QoS(4));
    robot_info_publisher = create_publisher<std_msgs::msg::Float32MultiArray>(robot_info_topic_, rclcpp::QoS(32));
    robot_status_publisher_ = create_publisher<std_msgs::msg::Float32MultiArray>(robot_status_topic_, rclcpp::QoS(31));

    br = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    odom_frame_id = declare_parameter("odom_frame_id", "odom");
    odom_child_frame_id = declare_parameter("odom_child_frame_id", "base_footprint");
    odom_pose_x = 0.0;
    odom_pose_y = 0.0;
    odom_orientation_z = 0.0;
    robot_status_timer_ = create_wall_timer(1s / robot_status_frequency_, [=]() { publish_robot_status(); });
    float pi_p = declare_parameter("motor_control_p_gain", 0.00);
    float pi_i = declare_parameter("motor_control_i_gain", 0.00);
    float pi_d = declare_parameter("motor_control_d_gain", 0.00);
    pidGains_ = {pi_p, pi_i, pi_d};
    auto now = get_clock()->now();
    //initialize connection to robot
    if (robot_type_ == "pro")
    {
        try
        {
            robot_ = std::make_unique<ProProtocolObject>(
                device_port_.c_str(), comm_type_, closed_loop_, pidGains_);
        }
        catch (const std::exception &ex)
        {
            RCLCPP_FATAL(get_logger(), "Trouble connecting to robot %s", std::to_string(errno));
            rclcpp::shutdown();
        }
    }
    else
    {
        rclcpp::shutdown();
    }
    RCLCPP_INFO(get_logger(), "Robot Setup Complete");
}

void RobotWrapper::velocity_callback(geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
    static double speeddata[2];
    speeddata[0] = msg->linear.x;
    speeddata[1] = msg->angular.z;
    speeddata[2] = msg->angular.y;
    // expecting values in the range of +/- linear_top_speed and +/- angular top
    // speed
    // if (linear_top_speed_ < std::abs(speeddata[0]))
    // {
    //     RCLCPP_WARN(
    //         get_logger(), "Requested linear velocity %f higher than maximum %", speeddata[0],
    //         linear_top_speed_);
    // }

    // auto turn_rate = msg->angular.z;
    // if (angular_top_speed_ < std::abs(turn_rate))
    // {
    //     RCLCPP_WARN(
    //         get_logger(), "Requested angular velocity %f higher than maximum %f", speeddata[1],
    //         angular_top_speed_);
    // }
    robot_->set_robot_velocity(speeddata);
}

void RobotWrapper::trim_callback(std_msgs::msg::Float32::ConstSharedPtr &msg)
{
    robot_->update_drivetrim(msg->data);
}

void RobotWrapper::estop_trigger_callback(std_msgs::msg::Bool::ConstSharedPtr &msg)
{
    if (msg->data == true)
    {
        estop_state_ = true;
        robot_->send_estop(estop_state_);
    }
}

void RobotWrapper::estop_reset_callback(std_msgs::msg::Bool::ConstSharedPtr &msg)
{
    if (msg->data == true)
    {
        estop_state_ = false;
        robot_->send_estop(estop_state_);
    }
}

void RobotWrapper::robot_info_request_callback(std_msgs::msg::Bool::ConstSharedPtr &msg)
{
    if (msg->data == true)
    {
        publish_robot_info();
    }
}

void RobotWrapper::publish_robot_info()
{
    // if (!robot_->is_connected())
    // {
    //     RCLCPP_FATAL(get_logger(), "Unexpectedly disconnected from serial port while trying to get robot unique info");
    //     rclcpp::shutdown();
    //     //* retry ?
    //     return;
    // }
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

void RobotWrapper::publish_robot_status()
{
    // if (!robot_->is_connected())
    // {
    //     RCLCPP_FATAL(get_logger(), "Unexpectedly disconnected from serial port while trying to get robot status");
    //     rclcpp::shutdown();
    //     return;
    // }
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

void RobotWrapper::update_odom()
{
    static auto prev_time = get_clock()->now();
    static auto current_time = get_clock()->now();
    auto odom = std::make_unique<nav_msgs::msg::Odometry>();
    odom->header.frame_id = odom_frame_id;
    odom->child_frame_id = odom_child_frame_id;
    odom->header.stamp = get_clock()->now();
    auto dt = (current_time - prev_time).seconds();
    if (dt > 0)
    {
        // In the odom_frame_id
        tf2::Quaternion quat;

        odom->pose.covariance.fill(-1.0);
        // We don't have any odom pose, but rviz complains if the Quat is not
        // normalized
        odom_pose_x += cos(odom_orientation_z) * robot_data_.linear_vel * dt;
        odom_pose_y += sin(odom_orientation_z) * robot_data_.linear_vel * dt;
        odom_orientation_z += robot_data_.angular_vel * dt;

        quat.setRPY(0, 0, odom_orientation_z);

        odom->pose.pose.position.x = odom_pose_x;
        odom->pose.pose.position.y = odom_pose_y;
        odom->pose.pose.orientation.z = quat.z();
        odom->pose.pose.orientation.w = quat.w();

        odom->twist.twist.linear.x = robot_data_.linear_vel;
        odom->twist.twist.angular.z = robot_data_.angular_vel;

        odom->twist.covariance.fill(0.0);

        //update TF;
        if (publish_tf == true)
        {
            tf.transform.translation.x = odom->pose.pose.position.x;
            tf.transform.translation.y = odom->pose.pose.position.y;
            tf.transform.translation.z = odom->pose.pose.position.z;
            tf.transform.rotation.x = quat.x();
            tf.transform.rotation.y = quat.y();
            tf.transform.rotation.z = quat.z();
            tf.transform.rotation.w = quat.w();
            tf.header.stamp = current_time;
            tf.header.frame_id = odom_frame_id;
            tf.child_frame_id = odom_child_frame_id;
            br->sendTransform(tf);
        }
        pub_odom->publish(std::move(odom));
    }
    prev_time = get_clock()->now();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor executor;

    auto rover_node = std::make_shared<RobotWrapper>();
    executor.add_node(rover_node);

    executor.spin();
    std::cerr << "Ros Node is running" << std::endl;
    return 0;
}