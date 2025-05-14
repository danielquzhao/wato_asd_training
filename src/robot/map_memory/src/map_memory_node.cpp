#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  // Initialize default parameters for the origin
  origin_.position.x = -16.0; 
  origin_.position.y = -16.0;  
  origin_.orientation.w = 1.0;

  setupCommunication();
  
  initGlobalMap();
}

void MapMemoryNode::setupCommunication() {
  // Initialize subscribers
  local_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, 
    std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, 
    std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  
  // Initialize publisher
  global_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/map", 10);
  
  // Initialize timer for map publication
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(timer_interval_),
    std::bind(&MapMemoryNode::updateMap, this));
}

void MapMemoryNode::initGlobalMap() {
  global_map_.info.resolution = resolution_;
  global_map_.info.width = width_;
  global_map_.info.height = height_;
  global_map_.info.origin = origin_;
  
  global_map_.data.assign(width_ * height_, -1);
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  const auto& costmap_data = msg->data;
  
  // Check if costmap has obstacle data
  int obstacle_count = 0;
  for (const auto& cell_value : costmap_data) {
    if (cell_value > 0) {
      obstacle_count++;
      break;
    }
  }
  
  // Skip if no obstacles detected
  if (obstacle_count == 0) {
    RCLCPP_DEBUG(this->get_logger(), "No obstacles detected in costmap data");
    return;
  }
  
  // Calculate displacement since last map update
  bool should_update = true;
  if (std::isfinite(last_update_x_)) {
    // Compute Euclidean distance from last update position
    const double dx = robot_x_ - last_update_x_;
    const double dy = robot_y_ - last_update_y_;
    const double displacement = std::sqrt(dx*dx + dy*dy);
    
    should_update = displacement >= update_distance_;
    if (!should_update) {
      RCLCPP_DEBUG(this->get_logger(), "Insufficient robot movement (%.2f m) for map update", displacement);
    }
  }
  
  // Process the costmap data if needed
  if (should_update) {
    // Integrate costmap data into global map
    integrateCostmap(msg, robot_x_, robot_y_, robot_theta_);
    
    last_update_x_ = robot_x_;
    last_update_y_ = robot_y_;
    
    RCLCPP_INFO(this->get_logger(), "Integrated new costmap data at robot position [%.2f, %.2f]", 
                robot_x_, robot_y_);
  }
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  const auto& pose = msg->pose.pose;
  
  // Store the robot's position
  robot_x_ = pose.position.x;
  robot_y_ = pose.position.y;
  
  // Get orientation quaternion components
  const auto& orientation = pose.orientation;
  
  // Calculate heading angle from quaternion
  robot_theta_ = extractYaw(
    orientation.x,
    orientation.y,
    orientation.z,
    orientation.w
  );
}

void MapMemoryNode::updateMap() {
  // Publish the current map
  global_map_.header.stamp = this->now();
  global_map_.header.frame_id = "sim_world"; 
  global_map_pub_->publish(global_map_);
}

double MapMemoryNode::extractYaw(double x, double y, double z, double w) {
  tf2::Quaternion quaternion(x, y, z, w);
  tf2::Matrix3x3 rotation_matrix(quaternion);
  
  // Extract Euler angles
  double roll, pitch, yaw;
  rotation_matrix.getRPY(roll, pitch, yaw);
  
  return yaw;
}

void MapMemoryNode::integrateCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr local_costmap, 
                              double robot_x, double robot_y, double robot_theta) {
  double cos_theta = std::cos(robot_theta);
  double sin_theta = std::sin(robot_theta);
  
  // Iterate through each cell in the local costmap
  for (unsigned int local_y = 0; local_y < local_costmap->info.height; ++local_y) {
    for (unsigned int local_x = 0; local_x < local_costmap->info.width; ++local_x) {
      unsigned int local_index = local_y * local_costmap->info.width + local_x;
      int8_t local_value = local_costmap->data[local_index];
      
      if (local_value < 0) {
        continue;
      }
      
      // Calculate the position of cell in robot's coordinate frame
      double cell_x_robot = (local_x * local_costmap->info.resolution) + local_costmap->info.origin.position.x;
      double cell_y_robot = (local_y * local_costmap->info.resolution) + local_costmap->info.origin.position.y;
      
      // Transform to global coordinate frame
      double cell_x_global = (cos_theta * cell_x_robot - sin_theta * cell_y_robot) + robot_x;
      double cell_y_global = (sin_theta * cell_x_robot + cos_theta * cell_y_robot) + robot_y;
      
      // Convert to grid coordinates in the global map
      int global_x, global_y;
      if (robotToMap(cell_x_global, cell_y_global, global_x, global_y)) {
        unsigned int global_index = global_y * global_map_.info.width + global_x;
        
        if (global_index < global_map_.data.size()) {
          if (global_map_.data[global_index] < local_value) {
            global_map_.data[global_index] = local_value;
          }
        }
      }
    }
  }
}

bool MapMemoryNode::robotToMap(double rx, double ry, int& mx, int& my) {
  // Convert from global coordinates to grid coordinates
  mx = static_cast<int>((rx - global_map_.info.origin.position.x) / global_map_.info.resolution);
  my = static_cast<int>((ry - global_map_.info.origin.position.y) / global_map_.info.resolution);
  
  // Check if the cell is within the global map bounds
  return (mx >= 0 && mx < static_cast<int>(global_map_.info.width) && 
          my >= 0 && my < static_cast<int>(global_map_.info.height));
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
