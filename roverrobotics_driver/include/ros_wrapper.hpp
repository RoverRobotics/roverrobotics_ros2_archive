#pragma once

#include <chrono>
#include "eigen3/Eigen/Dense"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <librover/protocol_pro.hpp>
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include "rclcpp/node_options.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <librover/protocol_pro.hpp>
// #include <librover/status_data.hpp>
using namespace std::chrono_literals;

using duration = std::chrono::nanoseconds;
namespace RoverRobotics
{
    /// This node supervises a Connection node and translates between low-level
    /// commands and high-level commands.
    class RobotWrapper : public rclcpp::Node
    {
    public:
        RobotWrapper();

    private:
        //robot protocol pointer
        std::unique_ptr<BaseProtocolObject> robot_;
        //universal robot data structure
        robotData robot_data_;
        PidGains pid_gains_;
        // Pub Sub

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr speed_command_subscriber_; // listen to cmd_vel inputs
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr trim_command_subscriber_;  // listen to trim value broadcast
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_trigger_subscriber_; // listen to estop trigger inputs
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_reset_subscriber_;   // listen to estop reset inputs
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr robot_info_subscriber;     // listen to robot_info request

        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr robot_info_publisher; // publish robot_unique info
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr robot_status_publisher_;       // publish robot state (battery,
                                                                                             // estop_status, speed)

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;

        // parameter variables
        std::string speed_topic_;
        std::string estop_trigger_topic_;
        std::string estop_reset_topic_;
        std::string robot_status_topic_;
        float robot_status_frequency_;
        std::string robot_info_request_topic_;
        std::string robot_info_topic_;
        std::string robot_type_;
        std::string trim_topic_;
        std::string device_port_;
        std::string comm_type_;
        // Timer
        rclcpp::TimerBase::SharedPtr robot_status_timer_;
        PidGains pidGains_ = {0, 0, 0};
        // motors
        float trimvalue;
        int motors_id_[4];
        bool estop_state_;
        bool closed_loop_;
        double linear_top_speed_;
        double angular_top_speed_;

        std::string odom_frame_id;
        std::string odom_child_frame_id;
        float odom_pose_x;
        float odom_pose_y;
        float odom_orientation_z;
        geometry_msgs::msg::TransformStamped tf;
        std::shared_ptr<tf2_ros::TransformBroadcaster> br;
        bool publish_tf;

        void velocity_callback(geometry_msgs::msg::Twist::ConstSharedPtr msg); //on_cmd_vel
        void trim_callback(std_msgs::msg::Float32::ConstSharedPtr &msg);
        void estop_trigger_callback(std_msgs::msg::Bool::ConstSharedPtr &msg);
        void estop_reset_callback(std_msgs::msg::Bool::ConstSharedPtr &msg);
        void robot_info_request_callback(std_msgs::msg::Bool::ConstSharedPtr &msg);
        void publish_robot_status();
        void publish_robot_info();
        void update_odom();

        // /// Callback for new raw data received
        // void on_raw_data(rover_msgs::msg::RawData::ConstSharedPtr data);

        // void update_firmware_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &status);
        // void update_power_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &status);
        // void update_drive_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &status);
        // std::shared_ptr<diagnostic_updater::Updater> updater;

        rclcpp::Time odom_last_time;
        rclcpp::TimerBase::SharedPtr tmr_odometry;
    };
} // namespace rover
