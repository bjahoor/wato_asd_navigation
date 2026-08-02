#include "planner_node.hpp"

#include <chrono>
#include <cmath>

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));

  goal_start_time_ = this->now();
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
  current_map_ = *map;
  map_received_ = true;

  // A new map can invalidate the route we are driving, so redo it.
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr goal) {
  goal_ = *goal;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  goal_start_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "New goal: (%.2f, %.2f)", goal_.point.x, goal_.point.y);
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
  robot_pose_ = odom->pose.pose;
  odom_received_ = true;
}

bool PlannerNode::goalReached() const {
  const double dx = goal_.point.x - robot_pose_.position.x;
  const double dy = goal_.point.y - robot_pose_.position.y;
  return std::hypot(dx, dy) < kGoalTolerance;
}

void PlannerNode::timerCallback() {
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    return;
  }

  if (goalReached()) {
    RCLCPP_INFO(this->get_logger(), "Goal reached");
    state_ = State::WAITING_FOR_GOAL;
    return;
  }

  if ((this->now() - goal_start_time_).seconds() > kGoalTimeout) {
    RCLCPP_WARN(this->get_logger(), "Gave up on the goal after %.0fs", kGoalTimeout);
    state_ = State::WAITING_FOR_GOAL;
    return;
  }

  // The robot has moved since the last plan, so the route from where it is now
  // is not the one we published.
  planPath();
}

void PlannerNode::planPath() {
  if (!goal_received_ || !map_received_ || !odom_received_) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan: waiting on map, goal or odometry");
    return;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = current_map_.header.frame_id;

  if (!planner_.planPath(current_map_,
                         robot_pose_.position.x, robot_pose_.position.y,
                         goal_.point.x, goal_.point.y,
                         path)) {
    return;
  }

  path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
