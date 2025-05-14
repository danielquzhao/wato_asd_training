#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <memory>
#include <vector>
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
  private:
    void setupCommunication();
    void initializeCostmap();
    void updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr laserscan);
    void inflateObstacle(int origin_x, int origin_y);
    
    // Subscriber and Publisher
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    
    // Costmap data
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> costmap_data_;
    
    // Parameters
    const std::string laser_topic_ = "/lidar";
    const std::string costmap_topic_ = "/costmap";
    
    double resolution_ = 0.2;
    int width_ = 240;
    int height_ = 240;
    geometry_msgs::msg::Pose origin_{};  
    double inflation_radius_ = 1.5;
    int inflation_cells_; 
};
 
#endif