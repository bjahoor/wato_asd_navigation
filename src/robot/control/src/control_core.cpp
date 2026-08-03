#include "control_core.hpp"

#include <algorithm>
#include <cmath>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

double ControlCore::computeDistance(const geometry_msgs::msg::Point& a,
                                    const geometry_msgs::msg::Point& b) const {
  return std::hypot(a.x - b.x, a.y - b.y);
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& quat) const {
  return std::atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                    1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z));
}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(
  const nav_msgs::msg::Path& path, const geometry_msgs::msg::Point& robot_position) const {
  if (path.poses.empty()) {
    return std::nullopt;
  }

  // First waypoint at least a lookahead away. Walking forward from the start
  // means the nearer waypoints are skipped as the robot passes them.
  for (const auto& pose : path.poses) {
    if (computeDistance(robot_position, pose.pose.position) >= kLookaheadDistance) {
      return pose;
    }
  }

  // Every waypoint is closer than that, which means the end of the path is in
  // reach. Aim at it.
  return path.poses.back();
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(
  const geometry_msgs::msg::Point& robot_position, double robot_yaw,
  const geometry_msgs::msg::Point& target) const {
  geometry_msgs::msg::Twist cmd;

  // How far off the robot's nose the target sits.
  const double heading_to_target =
    std::atan2(target.y - robot_position.y, target.x - robot_position.x);
  double alpha = heading_to_target - robot_yaw;

  // Wrap into [-pi, pi], so turning 10 degrees left never reads as 350 right.
  while (alpha > M_PI) {
    alpha -= 2.0 * M_PI;
  }
  while (alpha < -M_PI) {
    alpha += 2.0 * M_PI;
  }

  const double distance = computeDistance(robot_position, target);

  if (std::abs(alpha) > kTurnInPlaceAngle) {
    // Facing too far off to make progress. Stop and turn.
    cmd.linear.x = 0.0;
    cmd.angular.z = std::copysign(kMaxAngularSpeed, alpha);
    return cmd;
  }

  // Pure pursuit: the curvature of the arc from the robot to the target is
  // 2*sin(alpha)/distance. Speed along that arc gives the turn rate.
  const double curvature = distance > 0.0 ? 2.0 * std::sin(alpha) / distance : 0.0;

  cmd.linear.x = kLinearSpeed;
  cmd.angular.z =
    std::clamp(kLinearSpeed * curvature, -kMaxAngularSpeed, kMaxAngularSpeed);

  return cmd;
}

bool ControlCore::goalReached(const nav_msgs::msg::Path& path,
                              const geometry_msgs::msg::Point& robot_position) const {
  if (path.poses.empty()) {
    return false;
  }

  return computeDistance(robot_position, path.poses.back().pose.position) < kGoalTolerance;
}

}
