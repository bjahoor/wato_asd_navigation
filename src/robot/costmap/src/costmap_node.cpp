#include <cmath>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // Step 1: start from a blank grid - the costmap only shows this scan
  costmap_.initializeCostmap();

  // Step 2: convert each valid reading to a cell and mark it
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    const double range = scan->ranges[i];

    // Readings with no return come back as inf or nan
    if (!std::isfinite(range) || range < scan->range_min || range > scan->range_max) {
      continue;
    }

    const double angle = scan->angle_min + i * scan->angle_increment;

    int x_grid, y_grid;
    if (costmap_.convertToGrid(range, angle, x_grid, y_grid)) {
      costmap_.markObstacle(x_grid, y_grid);
    }
  }

  // Step 3: fade cost outward from each obstacle
  costmap_.inflateObstacles();

  // Step 4: publish, in the same frame the scan came in
  costmap_pub_->publish(costmap_.toOccupancyGrid(scan->header.frame_id, scan->header.stamp));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
