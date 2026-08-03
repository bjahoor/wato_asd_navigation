#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr path);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
    void controlLoop();

    void stop();

    robot::ControlCore control_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    nav_msgs::msg::Path current_path_;
    nav_msgs::msg::Odometry robot_odom_;

    bool path_received_ = false;
    bool odom_received_ = false;
    bool arrived_ = false;
};

#endif
