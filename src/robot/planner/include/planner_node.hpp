#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <mutex>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <memory>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "planner_core.hpp"

// ------------------- Supporting Structures -------------------
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

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    void setupCommunication();
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();
    bool planPath();
    void publishPath();
    void resetGoal();

    // A* algorithm implementation
    bool aStarSearch(const CellIndex& start, const CellIndex& goal, std::vector<CellIndex>& path);
    std::vector<CellIndex> getNeighbors(const CellIndex& cell);
    double heuristic(const CellIndex& a, const CellIndex& b);
    bool isValidCell(const CellIndex& cell);
    bool isFreeCell(const CellIndex& cell);
    double getCellObstacleCost(const CellIndex& cell);
    void reconstructPath(const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from, 
                        const CellIndex& current, std::vector<CellIndex>& path);
    
    // Coordinate conversion methods
    bool worldToGrid(double wx, double wy, CellIndex& cell);
    void gridToWorld(const CellIndex& cell, double& wx, double& wy);

    // Subscriber and Publisher
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Planner core
    robot::PlannerCore planner_;

    // State machine states
    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };
    State current_state_;

    // Progress tracking for replanning
    rclcpp::Time last_progress_time_;
    double last_distance_to_goal_ = std::numeric_limits<double>::max();
    int replan_attempts_ = 0;

    // Map data
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    std::mutex map_mutex_;

    // Path data
    nav_msgs::msg::Path current_path_;

    // Goal and robot positions
    geometry_msgs::msg::PointStamped goal_;
    bool have_goal_ = false;
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_theta_ = 0.0;
    bool have_odom_ = false;

    // Timing
    rclcpp::Time plan_start_time_;

    // Parameters
    const double goal_tolerance_ = 0.5;
    const double planning_timeout_ = 30.0;
    const double obstacle_avoidance_factor_ = 5.0;   
    const int obstacle_check_radius_ = 3;
    const double progress_timeout_ = 5.0;
    const double min_progress_distance_ = 0.1;
    const int max_replan_attempts_ = 3;              
};

#endif 
