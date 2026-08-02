#include "map_memory_core.hpp"

#include <algorithm>
#include <cmath>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
  : logger_(logger), grid_(kWidth * kHeight, kUnknown) {}

void MapMemoryCore::initializeMap() {
  std::fill(grid_.begin(), grid_.end(), kUnknown);
}

bool MapMemoryCore::inBounds(int x_grid, int y_grid) const {
  return x_grid >= 0 && x_grid < kWidth && y_grid >= 0 && y_grid < kHeight;
}

int8_t& MapMemoryCore::cell(int x_grid, int y_grid) {
  return grid_[y_grid * kWidth + x_grid];
}

int8_t MapMemoryCore::cell(int x_grid, int y_grid) const {
  return grid_[y_grid * kWidth + x_grid];
}

void MapMemoryCore::integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap,
                                     double robot_x, double robot_y, double robot_yaw) {
  const double cos_yaw = std::cos(robot_yaw);
  const double sin_yaw = std::sin(robot_yaw);

  // Several costmap cells land in the same map cell. The first one overwrites
  // whatever the map remembered; the rest only raise it, so a free cell can't
  // rub out an obstacle sharing the same map cell.
  std::vector<bool> touched(grid_.size(), false);

  for (unsigned int j = 0; j < costmap.info.height; ++j) {
    for (unsigned int i = 0; i < costmap.info.width; ++i) {
      const int8_t value = costmap.data[j * costmap.info.width + i];

      // Unknown in the costmap means no new information, so keep what we had.
      if (value < 0) {
        continue;
      }

      // Cell index -> metres, measured from the robot. The half cell puts us at
      // the centre of the cell rather than its corner.
      const double local_x =
        costmap.info.origin.position.x + (i + 0.5) * costmap.info.resolution;
      const double local_y =
        costmap.info.origin.position.y + (j + 0.5) * costmap.info.resolution;

      // Rotate by the robot's heading, then shift by its position.
      const double global_x = robot_x + local_x * cos_yaw - local_y * sin_yaw;
      const double global_y = robot_y + local_x * sin_yaw + local_y * cos_yaw;

      const int x_grid = static_cast<int>(std::floor((global_x - kOriginX) / kResolution));
      const int y_grid = static_cast<int>(std::floor((global_y - kOriginY) / kResolution));

      if (!inBounds(x_grid, y_grid)) {
        continue;
      }

      const size_t index = static_cast<size_t>(y_grid) * kWidth + x_grid;
      if (!touched[index] || value > cell(x_grid, y_grid)) {
        cell(x_grid, y_grid) = value;
        touched[index] = true;
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::toOccupancyGrid(const std::string& frame_id,
                                                            const rclcpp::Time& stamp) const {
  nav_msgs::msg::OccupancyGrid msg;

  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;

  msg.info.resolution = kResolution;
  msg.info.width = kWidth;
  msg.info.height = kHeight;
  msg.info.origin.position.x = kOriginX;
  msg.info.origin.position.y = kOriginY;
  msg.info.origin.orientation.w = 1.0;

  msg.data = grid_;

  return msg;
}

}
