#include "planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <vector>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
: logger_(logger) {}

bool PlannerCore::inBounds(const nav_msgs::msg::OccupancyGrid& map,
                           const CellIndex& index) const {
  return index.x >= 0 && index.x < static_cast<int>(map.info.width) &&
         index.y >= 0 && index.y < static_cast<int>(map.info.height);
}

int8_t PlannerCore::cost(const nav_msgs::msg::OccupancyGrid& map,
                         const CellIndex& index) const {
  return map.data[index.y * map.info.width + index.x];
}

bool PlannerCore::worldToGrid(const nav_msgs::msg::OccupancyGrid& map,
                              double x, double y, CellIndex& index) const {
  index.x = static_cast<int>(
    std::floor((x - map.info.origin.position.x) / map.info.resolution));
  index.y = static_cast<int>(
    std::floor((y - map.info.origin.position.y) / map.info.resolution));

  return inBounds(map, index);
}

void PlannerCore::gridToWorld(const nav_msgs::msg::OccupancyGrid& map,
                              const CellIndex& index, double& x, double& y) const {
  // Half a cell puts the waypoint in the middle of the cell, not on its corner.
  x = map.info.origin.position.x + (index.x + 0.5) * map.info.resolution;
  y = map.info.origin.position.y + (index.y + 0.5) * map.info.resolution;
}

bool PlannerCore::planPath(const nav_msgs::msg::OccupancyGrid& map,
                           double start_x, double start_y,
                           double goal_x, double goal_y,
                           nav_msgs::msg::Path& path) const {
  CellIndex start, goal;

  if (!worldToGrid(map, start_x, start_y, start) ||
      !worldToGrid(map, goal_x, goal_y, goal)) {
    RCLCPP_WARN(logger_, "Start or goal is off the map");
    return false;
  }

  if (cost(map, goal) >= kBlockedCost) {
    RCLCPP_WARN(logger_, "Goal sits on an obstacle");
    return false;
  }

  // g_score is the cheapest route found so far to a cell. came_from lets us
  // walk the finished route backwards to the start.
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, bool, CellIndexHash> closed;

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;

  const auto heuristic = [&](const CellIndex& a) {
    return std::hypot(a.x - goal.x, a.y - goal.y) * map.info.resolution;
  };

  g_score[start] = 0.0;
  open.emplace(start, heuristic(start));

  bool found = false;

  while (!open.empty()) {
    const CellIndex current = open.top().index;
    open.pop();

    if (current == goal) {
      found = true;
      break;
    }

    // The queue can hold the same cell twice, from before and after we found a
    // cheaper route to it. The stale copy is skipped here.
    if (closed[current]) {
      continue;
    }
    closed[current] = true;

    // Eight neighbours, so the path can cut diagonally instead of stair-stepping.
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) {
          continue;
        }

        const CellIndex neighbour(current.x + dx, current.y + dy);

        if (!inBounds(map, neighbour)) {
          continue;
        }

        const int8_t neighbour_cost = cost(map, neighbour);
        if (neighbour_cost >= kBlockedCost) {
          continue;
        }

        // Distance to step there, made more expensive by how close that cell is
        // to an obstacle. Unknown cells count as free, so the robot is willing
        // to head into ground it has not seen yet.
        const double step = std::hypot(dx, dy) * map.info.resolution;
        const double penalty =
          neighbour_cost > 0 ? kCostWeight * neighbour_cost / 100.0 : 0.0;
        const double tentative = g_score[current] + step * (1.0 + penalty);

        const auto existing = g_score.find(neighbour);
        if (existing == g_score.end() || tentative < existing->second) {
          g_score[neighbour] = tentative;
          came_from[neighbour] = current;
          open.emplace(neighbour, tentative + heuristic(neighbour));
        }
      }
    }
  }

  if (!found) {
    RCLCPP_WARN(logger_, "No path to the goal");
    return false;
  }

  // Walk back from the goal, then flip it so the path reads start to goal.
  std::vector<CellIndex> route;
  for (CellIndex at = goal; at != start; at = came_from[at]) {
    route.push_back(at);
  }
  route.push_back(start);
  std::reverse(route.begin(), route.end());

  path.poses.clear();
  path.poses.reserve(route.size());

  for (const auto& index : route) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    gridToWorld(map, index, pose.pose.position.x, pose.pose.position.y);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  return true;
}

}
