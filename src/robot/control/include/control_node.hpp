#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <utility>

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

    double extractYaw(double x, double y, double z, double w);
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void controlLoop();
    void timerCallback();

  private:
    void setupCommunication();
    bool isPathValid();
    void stopRobot();
    std::pair<size_t, double> findNearestPathPoint();
    std::pair<size_t, double> findLookaheadPoint(size_t start_index);
    bool isPathEndReached(size_t current_index, double distance);
    geometry_msgs::msg::Twist computeSteeringCommand(size_t target_index);

    // Subscriber and Publisher
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Path and robot state
    nav_msgs::msg::Path current_path_;
    double robot_x_;
    double robot_y_;
    double robot_theta_;
    bool have_path_;

    // Parameters
    const std::string path_topic_ = "/path";
    const std::string odom_topic_ = "/odom/filtered";
    const std::string cmd_vel_topic_ = "/cmd_vel";
    
    const int control_period_ms_ = 100;
    const double lookahead_distance_ = 1.0;
    const double steering_gain_ = 1.0;
    const double max_steering_angle_ = 1.0;
    const double linear_velocity_ = 0.5;
};

#endif
