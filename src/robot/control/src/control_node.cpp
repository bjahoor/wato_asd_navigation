#include "control_node.hpp"

#include <chrono>

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr path) {
  current_path_ = *path;
  path_received_ = true;

  // A fresh path means a goal is live again, even if the last one was finished.
  arrived_ = false;
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
  robot_odom_ = *odom;
  odom_received_ = true;
}

void ControlNode::stop() {
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
}

void ControlNode::controlLoop() {
  if (!path_received_ || !odom_received_) {
    return;
  }

  const auto& position = robot_odom_.pose.pose.position;

  if (arrived_) {
    return;
  }

  if (control_.goalReached(current_path_, position)) {
    RCLCPP_INFO(this->get_logger(), "Arrived, stopping");
    arrived_ = true;
    stop();
    return;
  }

  const auto lookahead = control_.findLookaheadPoint(current_path_, position);
  if (!lookahead) {
    // An empty path is the planner saying it has nothing for us. Don't coast.
    stop();
    return;
  }

  const double yaw = control_.extractYaw(robot_odom_.pose.pose.orientation);
  cmd_vel_pub_->publish(control_.computeVelocity(position, yaw, lookahead->pose.position));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
