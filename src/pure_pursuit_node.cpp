#include "pure_pursuit_node.hpp"

#include <cmath>
#include <utility>
#include <algorithm>

using std::placeholders::_1;

PurePursuitNode::PurePursuitNode()
    : Node("pure_pursuit_node"),
      lookahead_distance_(declare_parameter("lookahead_distance", 0.9)),
      wheelbase_length_(declare_parameter("wheelbase_length", 0.5)),
      max_linear_speed_(declare_parameter("max_linear_speed", 0.5)),
      pose_topic_(declare_parameter("pose_topic", "/current_pose")),
      path_topic_(declare_parameter("path_topic", "/waypoints")),
      cmd_vel_topic_(declare_parameter("cmd_vel_topic", "/cmd_vel")) {
  RCLCPP_INFO(get_logger(), "Pure Pursuit Node Initialized");
  RCLCPP_INFO(get_logger(), "Subscribing to pose: %s", pose_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Subscribing to path: %s", path_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Publishing to cmd_vel: %s", cmd_vel_topic_.c_str());

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, 10, std::bind(&PurePursuitNode::PoseCallback, this, _1));

  path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, 10, std::bind(&PurePursuitNode::PathCallback, this, _1));

  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);


  // Publishers for visualization markers
  lookahead_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("lookahead_marker", 10);
  waypoint_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("waypoint_marker", 10);
  trajectory_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("trajectory_marker", 10);

  

}

void PurePursuitNode::PoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  current_pose_ = msg;
  RunPurePursuit();
}

void PurePursuitNode::PathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  waypoints_.clear();
  for (const auto& pose : msg->poses) {
    waypoints_.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
  if (original_path_.empty()) {
    original_path_ = waypoints_;
  }
}

void PurePursuitNode::RunPurePursuit() {
  if (!current_pose_ || waypoints_.size() < 2) {
    return;
  }

  double x = current_pose_->pose.position.x;
  double y = current_pose_->pose.position.y;
  double yaw = GetYawFromQuaternion(current_pose_->pose.orientation);
  // double yaw_degree = yaw * 180.0 / M_PI;
  // Degree/180 = Radian/M_PI

  // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,



// Remove waypoints behind the robot based on heading
  auto it = std::find_if(waypoints_.begin(), waypoints_.end(), [&](const auto& wp) {
  double dx = wp.first - x;
  double dy = wp.second - y;

  // Check if point is in front of robot using dot product
  double forward_x = std::cos(yaw);
  double forward_y = std::sin(yaw);
  double dot = dx * forward_x + dy * forward_y;

  return dot > 0.0;  // point is in front
});

if (it != waypoints_.begin()) {
  waypoints_.erase(waypoints_.begin(), it);
}



  std::optional<std::pair<double, double>> lookahead_point;
  for (const auto& [wx, wy] : waypoints_) {
    double dx = wx - x;
    double dy = wy - y;
    double dist = std::hypot(dx, dy);
    if (dist > lookahead_distance_) {
      lookahead_point = std::make_pair(wx, wy);
      break;
    }
  }

  if (!lookahead_point.has_value()) {
    RCLCPP_INFO(get_logger(), "No valid lookahead point.");
    return;
  }

  double Ld = std::hypot(lookahead_point->first - x, lookahead_point->second - y);
  double path_angle = std::atan2(lookahead_point->second - y, lookahead_point->first - x);
  RCLCPP_INFO(get_logger(), "path_angle: %.2f degrees", path_angle * 180.0 / M_PI);

  double alpha = NormalizeAngle(path_angle - yaw);

  double curvature = (std::abs(alpha) < 1e-6) ? 0.0 : 1.0 / (Ld / (2.0 * std::sin(alpha)));
  double steering_angle = std::atan(curvature * wheelbase_length_);
  RCLCPP_INFO(get_logger(),"Yaw degree: %.2f", yaw * 180.0 / M_PI);
  RCLCPP_INFO(get_logger(),"Alpha degree: %.2f", alpha * 180.0 / M_PI);


  trajectory_.emplace_back(x, y);

  auto cmd = geometry_msgs::msg::Twist();
  cmd.linear.x = max_linear_speed_;
  cmd.angular.z = steering_angle;
  cmd_pub_->publish(cmd);

  // WAYPOINT MARKER
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = this->get_clock()->now();
  marker.ns = "pure_pursuit";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = lookahead_point->first;
  marker.pose.position.y = lookahead_point->second;
  marker.pose.position.z = 0.0;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.lifetime = rclcpp::Duration::from_seconds(0.1);
  lookahead_marker_pub_->publish(marker);


  // Publish path marker
  visualization_msgs::msg::Marker path_marker;
  path_marker.header.frame_id = "map";
  path_marker.header.stamp = this->get_clock()->now();
  path_marker.ns = "pure_pursuit_path";
  path_marker.id = 1;
  path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  path_marker.action = visualization_msgs::msg::Marker::ADD;
  path_marker.pose.orientation.w = 1.0; // No rotation  
  path_marker.scale.x = 0.15; // Width of the line
  path_marker.color.a = 1.0; // Fully opaque
  path_marker.color.r = 0.0; // Red color
  path_marker.color.g = 1.0; // Green color
  path_marker.color.b = 0.0; // Blue color    
  path_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
  path_marker.points.clear();
  for (const auto& [px, py] : original_path_) {
    geometry_msgs::msg::Point point;
    point.x = px;
    point.y = py;
    point.z = 0.0; // Assuming a flat path
    path_marker.points.push_back(point);
  }
  waypoint_marker_pub_->publish(path_marker);


  // Publish trajectory marker
  visualization_msgs::msg::Marker trajectory_marker;
  trajectory_marker.header.frame_id = "map";
  trajectory_marker.header.stamp = this->get_clock()->now();
  trajectory_marker.ns = "pure_pursuit_trajectory";
  trajectory_marker.id = 2;
  trajectory_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  trajectory_marker.action = visualization_msgs::msg::Marker::ADD;
  trajectory_marker.pose.orientation.w = 1.0; // No rotation
  trajectory_marker.scale.x = 0.1; // Width of the line
  trajectory_marker.color.a = 1.0; // Fully opaque
  trajectory_marker.color.r = 0.0; // Red color
  trajectory_marker.color.g = 0.0; // Green color
  trajectory_marker.color.b = 1.0; // Blue color
  trajectory_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
  trajectory_marker.points.clear();
  for (const auto& [tx, ty] : trajectory_) {
    geometry_msgs::msg::Point point;
    point.x = tx;
    point.y = ty;
    point.z = 0.0; // Assuming a flat trajectory
    trajectory_marker.points.push_back(point);      
}
  trajectory_marker_pub_->publish(trajectory_marker);
}

double PurePursuitNode::NormalizeAngle(double angle) const {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

double PurePursuitNode::GetYawFromQuaternion(const geometry_msgs::msg::Quaternion& q) const {
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

