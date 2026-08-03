#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    // Reset every cell to free space. Runs at the top of each scan.
    void initializeCostmap();

    // Polar reading -> grid indices. False if the point lands off the grid.
    bool convertToGrid(double range, double angle, int& x_grid, int& y_grid) const;

    // Mark a cell occupied and remember it for the inflation pass.
    void markObstacle(int x_grid, int y_grid);

    // Fade cost outward from every marked obstacle.
    void inflateObstacles();

    // Package the grid up for publishing.
    nav_msgs::msg::OccupancyGrid toOccupancyGrid(const std::string& frame_id,
                                                 const rclcpp::Time& stamp) const;

  private:
    bool inBounds(int x_grid, int y_grid) const;
    int8_t& cell(int x_grid, int y_grid);
    int8_t cell(int x_grid, int y_grid) const;

    rclcpp::Logger logger_;

    // 200 cells at 0.1m covers 20m, centred on the robot.
    static constexpr double kResolution = 0.1;
    static constexpr int kWidth = 200;
    static constexpr int kHeight = 200;
    static constexpr double kOriginX = -kWidth * kResolution / 2.0;
    static constexpr double kOriginY = -kHeight * kResolution / 2.0;

    // How far the buffer reaches. This is the most clearance the planner can
    // ever ask for, since past it every cell costs the same.
    static constexpr double kInflationRadius = 2.0;
    static constexpr int8_t kMaxCost = 100;

    std::vector<int8_t> grid_;
    std::vector<std::pair<int, int>> obstacles_;
};

}

#endif
