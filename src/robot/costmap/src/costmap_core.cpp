#include "costmap_core.hpp"

#include <algorithm>
#include <cmath>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger)
  : logger_(logger), grid_(kWidth * kHeight, 0) {}

void CostmapCore::initializeCostmap() {
  std::fill(grid_.begin(), grid_.end(), 0);
  obstacles_.clear();
}

bool CostmapCore::inBounds(int x_grid, int y_grid) const {
  return x_grid >= 0 && x_grid < kWidth && y_grid >= 0 && y_grid < kHeight;
}

int8_t& CostmapCore::cell(int x_grid, int y_grid) {
  return grid_[y_grid * kWidth + x_grid];
}

int8_t CostmapCore::cell(int x_grid, int y_grid) const {
  return grid_[y_grid * kWidth + x_grid];
}

bool CostmapCore::convertToGrid(double range, double angle, int& x_grid, int& y_grid) const {
  const double x = range * std::cos(angle);
  const double y = range * std::sin(angle);

  // Shift by the origin so the robot sits in the middle of the grid, then
  // divide by cell size to get which cell the point falls in.
  x_grid = static_cast<int>(std::floor((x - kOriginX) / kResolution));
  y_grid = static_cast<int>(std::floor((y - kOriginY) / kResolution));

  return inBounds(x_grid, y_grid);
}

void CostmapCore::markObstacle(int x_grid, int y_grid) {
  if (!inBounds(x_grid, y_grid)) {
    return;
  }

  cell(x_grid, y_grid) = kMaxCost;
  obstacles_.emplace_back(x_grid, y_grid);
}

void CostmapCore::inflateObstacles() {
  const int radius_cells = static_cast<int>(kInflationRadius / kResolution);

  // Walk the square around each obstacle and keep the cells inside the radius.
  for (const auto& obstacle : obstacles_) {
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        const int x_grid = obstacle.first + dx;
        const int y_grid = obstacle.second + dy;

        if (!inBounds(x_grid, y_grid)) {
          continue;
        }

        const double distance = std::hypot(dx, dy) * kResolution;
        if (distance > kInflationRadius) {
          continue;
        }

        const auto cost =
          static_cast<int8_t>(kMaxCost * (1.0 - distance / kInflationRadius));

        // Overlapping obstacles: keep whichever cost is worse.
        if (cost > cell(x_grid, y_grid)) {
          cell(x_grid, y_grid) = cost;
        }
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid CostmapCore::toOccupancyGrid(const std::string& frame_id,
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
