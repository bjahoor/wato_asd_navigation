#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <optional>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    // The point on the path to aim at. Empty if the path has nothing usable on it.
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(
      const nav_msgs::msg::Path& path, const geometry_msgs::msg::Point& robot_position) const;

    // Speeds that curve the robot towards the point it is aiming at.
    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::Point& robot_position,
                                              double robot_yaw,
                                              const geometry_msgs::msg::Point& target) const;

    // Straight-line distance between two points, ignoring height.
    double computeDistance(const geometry_msgs::msg::Point& a,
                           const geometry_msgs::msg::Point& b) const;

    // Heading out of a quaternion. Only the heading matters on flat ground.
    double extractYaw(const geometry_msgs::msg::Quaternion& quat) const;

    // Has the robot arrived at the end of the path?
    bool goalReached(const nav_msgs::msg::Path& path,
                     const geometry_msgs::msg::Point& robot_position) const;

  private:
    rclcpp::Logger logger_;

    static constexpr double kLookaheadDistance = 1.0;
    static constexpr double kGoalTolerance = 0.1;
    static constexpr double kLinearSpeed = 0.5;

    // Past this heading error, driving forward would swing the robot wide, so it
    // turns on the spot instead. A differential drive can do that; a car cannot.
    static constexpr double kTurnInPlaceAngle = 1.05;  // 60 degrees

    // Ceiling on how fast it may spin, so a near-reversal doesn't ask for a
    // rotation the robot can't deliver.
    static constexpr double kMaxAngularSpeed = 1.5;
};

}

#endif
