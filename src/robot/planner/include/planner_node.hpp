#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    // Waiting for a goal, or driving to one. Nothing else to be.
    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr goal);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
    void timerCallback();

    bool goalReached() const;
    void planPath();

    robot::PlannerCore planner_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Close enough to call it arrived.
    static constexpr double kGoalTolerance = 0.5;

    // Give up on a goal that has taken this long, so a stuck robot stops
    // replanning a route it is never going to drive.
    static constexpr double kGoalTimeout = 60.0;

    State state_ = State::WAITING_FOR_GOAL;

    nav_msgs::msg::OccupancyGrid current_map_;
    geometry_msgs::msg::PointStamped goal_;
    geometry_msgs::msg::Pose robot_pose_;

    bool map_received_ = false;
    bool goal_received_ = false;
    bool odom_received_ = false;

    rclcpp::Time goal_start_time_;
};

#endif
