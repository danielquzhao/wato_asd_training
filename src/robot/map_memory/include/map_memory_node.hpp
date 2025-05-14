#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include <memory>
#include <vector>
#include <limits>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    void setupCommunication();
    void initGlobalMap();
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void updateMap();
    double extractYaw(double x, double y, double z, double w);
    void integrateCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr local_costmap, 
                         double robot_x, double robot_y, double robot_theta);
    bool robotToMap(double rx, double ry, int& mx, int& my);

    // Core logic reference
    robot::MapMemoryCore map_memory_;
    
    // Global map
    nav_msgs::msg::OccupancyGrid global_map_;
    
    // Subscriber and Publisher
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Parameters
    double update_distance_ = 1.5;
    int timer_interval_ = 3000;
    
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_theta_ = 0.0;
    
    double last_update_x_ = std::numeric_limits<double>::quiet_NaN();
    double last_update_y_ = std::numeric_limits<double>::quiet_NaN();
    
    double resolution_ = 0.4;
    int width_ = 80;
    int height_ = 80;
    geometry_msgs::msg::Pose origin_;
};

#endif 
