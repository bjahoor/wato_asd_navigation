#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    // Reset every cell to unknown. Runs once, at startup.
    void initializeMap();

    // Merge one costmap into the global map, placed by the robot's pose.
    void integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap,
                          double robot_x, double robot_y, double robot_yaw);

    // Package the global map up for publishing.
    nav_msgs::msg::OccupancyGrid toOccupancyGrid(const std::string& frame_id,
                                                 const rclcpp::Time& stamp) const;

  private:
    bool inBounds(int x_grid, int y_grid) const;
    int8_t& cell(int x_grid, int y_grid);
    int8_t cell(int x_grid, int y_grid) const;

    rclcpp::Logger logger_;

    // 200 cells at 0.2m covers 40m, enough for the 30m arena. Coarser than the
    // costmap so its cells always land inside a map cell, never between two.
    static constexpr double kResolution = 0.2;
    static constexpr int kWidth = 200;
    static constexpr int kHeight = 200;
    static constexpr double kOriginX = -kWidth * kResolution / 2.0;
    static constexpr double kOriginY = -kHeight * kResolution / 2.0;

    static constexpr int8_t kUnknown = -1;

    std::vector<int8_t> grid_;
};

}

#endif
