#include "map_memory_node.hpp"

#include <chrono>
#include <cmath>

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  map_memory_.initializeMap();

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap) {
  latest_costmap_ = *costmap;
  costmap_received_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
  robot_x_ = odom->pose.pose.position.x;
  robot_y_ = odom->pose.pose.position.y;

  // Yaw out of the quaternion. Only the heading matters on flat ground.
  const auto& q = odom->pose.pose.orientation;
  robot_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  map_frame_ = odom->header.frame_id;

  // The first reading has nothing to compare against, so fuse straight away and
  // use it as the starting point.
  if (!odom_received_) {
    odom_received_ = true;
    last_x_ = robot_x_;
    last_y_ = robot_y_;
    should_update_map_ = true;
    return;
  }

  const double distance = std::hypot(robot_x_ - last_x_, robot_y_ - last_y_);
  if (distance >= kDistanceThreshold) {
    last_x_ = robot_x_;
    last_y_ = robot_y_;
    should_update_map_ = true;
  }
}

void MapMemoryNode::updateMap() {
  if (should_update_map_ && costmap_received_) {
    map_memory_.integrateCostmap(latest_costmap_, robot_x_, robot_y_, robot_yaw_);
    should_update_map_ = false;
  }

  // Published every tick, not just after a fusion, so anything that starts late
  // still gets a map instead of waiting for the robot to drive.
  map_pub_->publish(map_memory_.toOccupancyGrid(map_frame_, this->get_clock()->now()));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
