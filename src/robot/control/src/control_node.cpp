#include "control_node.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <algorithm>
#include <cmath>
#include <limits>

ControlNode::ControlNode()
  : Node("control") 
{
  robot_x_ = 0.0;
  robot_y_ = 0.0;
  robot_theta_ = 0.0;
  have_path_ = false;
  
  setupCommunication();
}

double ControlNode::extractYaw(double x, double y, double z, double w) {
  tf2::Quaternion quaternion(x, y, z, w);
  tf2::Matrix3x3 rotation_matrix(quaternion);
  
  // Extract Euler angles
  double roll, pitch, yaw;
  rotation_matrix.getRPY(roll, pitch, yaw);
  
  return yaw;
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  // Ignore empty paths
  if (msg->poses.size() == 0) {
    RCLCPP_WARN(this->get_logger(), "Empty path received - ignoring");
    have_path_ = false;
    return;
  }
  
  // Store the path for processing
  current_path_ = *msg;
  have_path_ = true;
  
  RCLCPP_INFO(this->get_logger(), "Path received with %zu waypoints", msg->poses.size());
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  auto& pose = msg->pose.pose;
  
  // Store position
  robot_x_ = pose.position.x;
  robot_y_ = pose.position.y;
  
  // Convert orientation to heading angle
  auto& q = pose.orientation;
  robot_theta_ = extractYaw(q.x, q.y, q.z, q.w);
}

void ControlNode::controlLoop() {
  // Check if we have a valid path to follow
  if (!isPathValid()) {
    stopRobot();
    return;
  }
  
  // Find nearest point on path
  auto [nearest_idx, nearest_distance] = findNearestPathPoint();
  
  // Determine target point with lookahead distance
  auto [target_idx, target_distance] = findLookaheadPoint(nearest_idx);
  
  // Check if we've reached the end of the path
  if (isPathEndReached(target_idx, target_distance)) {
    RCLCPP_INFO(this->get_logger(), "Path end reached");
    stopRobot();
    have_path_ = false;
    return;
  }
  
  // Get steering command to follow the path
  auto velocity_command = computeSteeringCommand(target_idx);
  
  // Send command to the robot
  cmd_vel_publisher_->publish(velocity_command);
  
  RCLCPP_DEBUG(this->get_logger(), "Velocity command: linear=%0.2f, angular=%0.2f", 
              velocity_command.linear.x, velocity_command.angular.z);
}

void ControlNode::timerCallback() {
  // Execute the main control loop at regular intervals
  controlLoop();
}

void ControlNode::setupCommunication() {
  // Create subscribers
  path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
    path_topic_, 10, 
    std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
    
  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10, 
    std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
  
  // Create publisher
  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
    cmd_vel_topic_, 10);
  
  // Create control timer
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(control_period_ms_),
    std::bind(&ControlNode::timerCallback, this));
}

bool ControlNode::isPathValid() {
  return have_path_ && !current_path_.poses.empty();
}

void ControlNode::stopRobot() {
  geometry_msgs::msg::Twist zero_velocity;
  zero_velocity.linear.x = 0.0;
  zero_velocity.angular.z = 0.0;
  cmd_vel_publisher_->publish(zero_velocity);
}

std::pair<size_t, double> ControlNode::findNearestPathPoint() {
  size_t nearest_index = 0;
  double min_distance = std::numeric_limits<double>::max();
  
  // Find the closest point on the path to the robot
  for (size_t i = 0; i < current_path_.poses.size(); ++i) {
    const auto& waypoint = current_path_.poses[i].pose.position;
    double dx = waypoint.x - robot_x_;
    double dy = waypoint.y - robot_y_;
    double distance = std::hypot(dx, dy); 
    
    if (distance < min_distance) {
      min_distance = distance;
      nearest_index = i;
    }
  }
  
  return {nearest_index, min_distance};
}

std::pair<size_t, double> ControlNode::findLookaheadPoint(size_t start_index) {
  size_t target_index = start_index;
  double target_distance = 0.0;
  
  // Look for a point that's approximately the lookahead distance away
  while (target_index < current_path_.poses.size() - 1 && 
         target_distance < lookahead_distance_) {
    ++target_index;
    
    // Calculate distance from robot to this point
    const auto& waypoint = current_path_.poses[target_index].pose.position;
    double dx = waypoint.x - robot_x_;
    double dy = waypoint.y - robot_y_;
    target_distance = std::hypot(dx, dy);
  }
  
  return {target_index, target_distance};
}

bool ControlNode::isPathEndReached(size_t current_index, double distance) {
  // Reached end of path if at last point and within half the lookahead distance
  return (current_index == current_path_.poses.size() - 1 && 
          distance < lookahead_distance_ / 2.0);
}

geometry_msgs::msg::Twist ControlNode::computeSteeringCommand(size_t target_index) {
  // Get target location
  const auto& target_pos = current_path_.poses[target_index].pose.position;
  
  // Calculate vector to target
  double dx = target_pos.x - robot_x_;
  double dy = target_pos.y - robot_y_;
  
  // Calculate desired heading and error
  double desired_heading = std::atan2(dy, dx);
  double heading_error = desired_heading - robot_theta_;
  heading_error = std::atan2(std::sin(heading_error), std::cos(heading_error));
  
  // Proportional control for steering
  double angular_z = steering_gain_ * heading_error;
  
  // Apply limits
  angular_z = std::clamp(angular_z, -max_steering_angle_, max_steering_angle_);
  
  // Create command
  geometry_msgs::msg::Twist velocity_cmd;
  velocity_cmd.linear.x = linear_velocity_;
  velocity_cmd.angular.z = angular_z;
  
  return velocity_cmd;
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
