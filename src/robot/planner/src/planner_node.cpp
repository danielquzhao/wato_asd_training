#include <chrono>
#include <memory>
#include <algorithm>
#include <limits>
#include <cmath>

#include "planner_node.hpp"

PlannerNode::PlannerNode() 
  : Node("planner"), 
    planner_(robot::PlannerCore(this->get_logger())), 
    current_state_(State::WAITING_FOR_GOAL)
{
  setupCommunication();
}

void PlannerNode::setupCommunication() {
  // Initialize subscribers
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, 
    std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, 
    std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, 
    std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  
  // Initialize publisher
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  
  // Initialize timer for checking goal status
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), 
    std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_ = msg;
  }
  
  // If we have a goal, replan
  if (have_goal_ && current_state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    RCLCPP_INFO(this->get_logger(), "Map updated, replanning...");
    replan_attempts_++;
    
    if (replan_attempts_ > max_replan_attempts_) {
      RCLCPP_WARN(this->get_logger(), "Too many replans (%d) without progress. Resetting goal.", replan_attempts_);
      resetGoal();
      return;
    }
    
    if (planPath()) {
      publishPath();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to replan path after map update");
      resetGoal();
    }
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received goal: (%.2f, %.2f)", msg->point.x, msg->point.y);
  
  // Store the goal and mark that we have a goal
  goal_ = *msg;
  have_goal_ = true;
  
  // Initialize recording
  current_state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  plan_start_time_ = this->now();
  last_progress_time_ = this->now();
  
  // Calculate initial distance to goal if we have odometry
  if (have_odom_) {
    last_distance_to_goal_ = std::sqrt(
      std::pow(robot_x_ - goal_.point.x, 2) + 
      std::pow(robot_y_ - goal_.point.y, 2));
  } else {
    last_distance_to_goal_ = std::numeric_limits<double>::max();
  }
  
  replan_attempts_ = 0;
  
  // Plan path to the goal
  if (planPath()) {
    publishPath();
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to plan path to the goal");
    resetGoal();
  }
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Store the robot's position
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  
  // Extract yaw from quaternion
  double qx = msg->pose.pose.orientation.x;
  double qy = msg->pose.pose.orientation.y;
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;
  
  // Convert quaternion to yaw
  double siny_cosp = 2.0 * (qw * qz + qx * qy);
  double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  robot_theta_ = std::atan2(siny_cosp, cosy_cosp);
  
  have_odom_ = true;
}

void PlannerNode::timerCallback() {
  if (!have_goal_ || current_state_ == State::WAITING_FOR_GOAL) {
    return;
  }
  
  // Check if we've reached the goal
  double distance_to_goal = std::sqrt(
    std::pow(robot_x_ - goal_.point.x, 2) + 
    std::pow(robot_y_ - goal_.point.y, 2));
  
  if (distance_to_goal <= goal_tolerance_) {
    RCLCPP_INFO(this->get_logger(), "Goal reached! Distance: %.2f", distance_to_goal);
    resetGoal();
    return;
  }
  
  // Check if we've timed out
  double elapsed_time = (this->now() - plan_start_time_).seconds();
  if (elapsed_time > planning_timeout_) {
    RCLCPP_WARN(this->get_logger(), "Planning timed out after %.2f seconds", elapsed_time);
    resetGoal();
    return;
  }
  
  // Check if we're making progress toward the goal
  double progress = last_distance_to_goal_ - distance_to_goal;
  double time_since_progress = (this->now() - last_progress_time_).seconds();
  
  if (progress >= min_progress_distance_) {
    last_distance_to_goal_ = distance_to_goal;
    last_progress_time_ = this->now();
    replan_attempts_ = 0; 
    RCLCPP_DEBUG(this->get_logger(), "Making progress toward goal. Current distance: %.2f m", distance_to_goal);
  } 

  else if (time_since_progress > progress_timeout_) {
    replan_attempts_++;
    
    if (replan_attempts_ > max_replan_attempts_) {
      RCLCPP_WARN(this->get_logger(), "Failed to make progress after %d replan attempts. Giving up.", max_replan_attempts_);
      resetGoal();
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), 
                "No progress for %.1f seconds (attempt %d/%d). Replanning...", 
                time_since_progress, replan_attempts_, max_replan_attempts_);
    
    last_progress_time_ = this->now();
    
    if (planPath()) {
      publishPath();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to replan path to the goal");
      resetGoal();
    }
  }
}

bool PlannerNode::planPath() {
  if (!have_odom_ || !have_goal_) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan: missing odometry or goal");
    return false;
  }
  
  // Lock the map to ensure it doesn't change during planning
  std::lock_guard<std::mutex> lock(map_mutex_);
  
  if (!map_) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan: no map available");
    return false;
  }
  
  // Convert start and goal to grid cells
  CellIndex start_cell, goal_cell;
  if (!worldToGrid(robot_x_, robot_y_, start_cell) || 
      !worldToGrid(goal_.point.x, goal_.point.y, goal_cell)) {
    RCLCPP_WARN(this->get_logger(), "Start or goal position is outside the map");
    return false;
  }
  
  // Check if the goal is in a free cell
  if (!isFreeCell(goal_cell)) {
    RCLCPP_WARN(this->get_logger(), "Goal is in an occupied or unknown cell");
    return false;
  }
  
  // Plan path using A*
  std::vector<CellIndex> path_cells;
  if (!aStarSearch(start_cell, goal_cell, path_cells)) {
    RCLCPP_WARN(this->get_logger(), "A* search failed to find a path");
    return false;
  }
  
  // Convert the grid cells to a ROS path message
  current_path_.poses.clear();
  current_path_.header.frame_id = map_->header.frame_id;
  current_path_.header.stamp = this->now();
  
  for (const auto& cell : path_cells) {
    double wx, wy;
    gridToWorld(cell, wx, wy);
    
    geometry_msgs::msg::PoseStamped pose;
    pose.header = current_path_.header;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0; 
    
    current_path_.poses.push_back(pose);
  }
  
  return true;
}

bool PlannerNode::aStarSearch(const CellIndex& start, const CellIndex& goal, std::vector<CellIndex>& path) {
  // Initialize data structures for A*
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, bool, CellIndexHash> closed_set;
  
  // Add start node to open set
  open_set.push(AStarNode(start, heuristic(start, goal)));
  g_score[start] = 0;
  
  while (!open_set.empty()) {
    CellIndex current = open_set.top().index;
    open_set.pop();
    
    if (closed_set.find(current) != closed_set.end()) {
      continue;
    }
    
    closed_set[current] = true;
    
    if (current == goal) {
      reconstructPath(came_from, current, path);
      return true;
    }
    
    for (const auto& neighbor : getNeighbors(current)) {
      if (closed_set.find(neighbor) != closed_set.end() || !isFreeCell(neighbor)) {
        continue;
      }
      
      double movement_cost = (neighbor.x == current.x || neighbor.y == current.y) ? 1.0 : 1.414; 
      
      double obstacle_cost = getCellObstacleCost(neighbor);
      movement_cost += obstacle_cost;
      
      double tentative_g_score = g_score[current] + movement_cost;
      
      if (g_score.find(neighbor) == g_score.end() || tentative_g_score < g_score[neighbor]) {
        came_from[neighbor] = current;
        g_score[neighbor] = tentative_g_score;
        
        double f_score = tentative_g_score + heuristic(neighbor, goal);
        open_set.push(AStarNode(neighbor, f_score));
      }
    }
  }
  
  return false;
}

void PlannerNode::publishPath() {
  path_pub_->publish(current_path_);
  RCLCPP_INFO(this->get_logger(), "Published path with %zu points", current_path_.poses.size());
}

void PlannerNode::resetGoal() {
  have_goal_ = false;
  current_state_ = State::WAITING_FOR_GOAL;
  
  // Reset progress tracking
  last_distance_to_goal_ = std::numeric_limits<double>::max();
  replan_attempts_ = 0;
  
  // Publish an empty path to stop the robot
  nav_msgs::msg::Path empty_path;
  empty_path.header.stamp = this->now();
  empty_path.header.frame_id = map_ ? map_->header.frame_id : "map";
  path_pub_->publish(empty_path);
  
  RCLCPP_INFO(this->get_logger(), "Goal reset, waiting for new goal");
}

std::vector<CellIndex> PlannerNode::getNeighbors(const CellIndex& cell) {
  std::vector<CellIndex> neighbors;
  
  // 8-connected grid (including diagonals)
  for (int dx = -1; dx <= 1; dx++) {
    for (int dy = -1; dy <= 1; dy++) {
      if (dx == 0 && dy == 0) continue;
      
      CellIndex neighbor(cell.x + dx, cell.y + dy);
      if (isValidCell(neighbor)) {
        neighbors.push_back(neighbor);
      }
    }
  }
  
  return neighbors;
}

double PlannerNode::heuristic(const CellIndex& a, const CellIndex& b) {
  // Euclidean distance heuristic
  double euclidean_distance = std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
  
  return euclidean_distance;
}

bool PlannerNode::isValidCell(const CellIndex& cell) {
  if (!map_) return false;
  
  return (cell.x >= 0 && cell.x < static_cast<int>(map_->info.width) && 
          cell.y >= 0 && cell.y < static_cast<int>(map_->info.height));
}

bool PlannerNode::isFreeCell(const CellIndex& cell) {
  if (!isValidCell(cell)) return false;
  
  // Convert 2D cell coordinates to 1D index in the map data
  size_t index = cell.y * map_->info.width + cell.x;
  
  // Check if the cell is free (value < 50 in the occupancy grid)
  return (map_->data[index] >= 0 && map_->data[index] < 50);
}

double PlannerNode::getCellObstacleCost(const CellIndex& cell) {
  if (!isValidCell(cell)) return std::numeric_limits<double>::max();

  size_t index = cell.y * map_->info.width + cell.x;
  
  // Get the occupancy value
  int occupancy = map_->data[index];
  if (occupancy < 0) return 10.0;
  
  // Calculate cost
  double cost = (occupancy / 10.0) + 0.1;
  
  // Check surrounding cells to find the nearest obstacle
  double min_distance_to_obstacle = obstacle_check_radius_ + 1.0;
  
  for (int dx = -obstacle_check_radius_; dx <= obstacle_check_radius_; dx++) {
    for (int dy = -obstacle_check_radius_; dy <= obstacle_check_radius_; dy++) {
      if (dx == 0 && dy == 0) continue;
      
      CellIndex neighbor(cell.x + dx, cell.y + dy);
      if (!isValidCell(neighbor)) continue;
      
      size_t neighbor_index = neighbor.y * map_->info.width + neighbor.x;
      int neighbor_occupancy = map_->data[neighbor_index];
      
      if (neighbor_occupancy >= 50) {
        double distance = std::sqrt(dx*dx + dy*dy);
        min_distance_to_obstacle = std::min(min_distance_to_obstacle, distance);
      }
    }
  }
  
  if (min_distance_to_obstacle <= obstacle_check_radius_) {
    cost += obstacle_avoidance_factor_ / min_distance_to_obstacle;
  }
  
  return cost;
}

void PlannerNode::reconstructPath(const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from, 
                                 const CellIndex& current, std::vector<CellIndex>& path) {
  CellIndex current_cell = current;
  path.push_back(current_cell);
  
  // Trace back to the start
  while (came_from.find(current_cell) != came_from.end()) {
    current_cell = came_from.at(current_cell);
    path.push_back(current_cell);
  }
  
  // Reverse to get path from start to goal
  std::reverse(path.begin(), path.end());
}

bool PlannerNode::worldToGrid(double wx, double wy, CellIndex& cell) {
  if (!map_) return false;
  
  // Convert world coordinates to grid coordinates
  cell.x = static_cast<int>((wx - map_->info.origin.position.x) / map_->info.resolution);
  cell.y = static_cast<int>((wy - map_->info.origin.position.y) / map_->info.resolution);
  
  return isValidCell(cell);
}

void PlannerNode::gridToWorld(const CellIndex& cell, double& wx, double& wy) {
  if (!map_) {
    wx = 0.0;
    wy = 0.0;
    return;
  }
  
  // Convert grid coordinates to world coordinates
  wx = cell.x * map_->info.resolution + map_->info.origin.position.x;
  wy = cell.y * map_->info.resolution + map_->info.origin.position.y;
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
