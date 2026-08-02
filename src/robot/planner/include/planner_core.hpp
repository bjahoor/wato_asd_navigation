#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot
{

// 2D grid index
struct CellIndex
{
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }

  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};

// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    // A simple hash combining x and y
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// Structure representing a node in the A* open set
struct AStarNode
{
  CellIndex index;
  double f_score;  // f = g + h

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// Comparator for the priority queue (min-heap by f_score)
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    // We want the node with the smallest f_score on top
    return a.f_score > b.f_score;
  }
};

class PlannerCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit PlannerCore(const rclcpp::Logger& logger);

    // A* from the robot to the goal across the map. False if no route exists.
    bool planPath(const nav_msgs::msg::OccupancyGrid& map,
                  double start_x, double start_y,
                  double goal_x, double goal_y,
                  nav_msgs::msg::Path& path) const;

  private:
    bool worldToGrid(const nav_msgs::msg::OccupancyGrid& map,
                     double x, double y, CellIndex& index) const;
    void gridToWorld(const nav_msgs::msg::OccupancyGrid& map,
                     const CellIndex& index, double& x, double& y) const;
    bool inBounds(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& index) const;
    int8_t cost(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& index) const;

    rclcpp::Logger logger_;

    // Only a hard obstacle is impassable. Everything under it is the buffer the
    // costmap laid down, which the planner may cross when it has to.
    static constexpr int8_t kBlockedCost = 100;

    // How much a cell's cost counts against the distance to cross it. Higher
    // keeps to the middle of gaps, lower cuts corners.
    static constexpr double kCostWeight = 4.0;
};

}

#endif
