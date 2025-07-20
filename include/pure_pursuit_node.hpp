#ifndef PURE_PURSUIT__PURE_PURSUIT_NODE_HPP_
#define PURE_PURSUIT__PURE_PURSUIT_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


class PurePursuitNode : public rclcpp::Node
{
 public:
  PurePursuitNode();
  

 private:

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr waypoint_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_marker_pub_;



  // Parameters
  double lookahead_distance_;
  double wheelbase_length_;
  double max_linear_speed_;
  std::string pose_topic_;
  std::string path_topic_;
  std::string cmd_vel_topic_;


  // State
  geometry_msgs::msg::PoseStamped::SharedPtr current_pose_;
  std::vector<std::pair<double, double>> waypoints_;
  std::vector<std::pair<double, double>> original_path_;
  std::vector<std::pair<double, double>> trajectory_;

  // Callbacks
  void PoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void PathCallback(const nav_msgs::msg::Path::SharedPtr msg);

  // Main logic
  void RunPurePursuit();

  // Helpers
  double NormalizeAngle(double angle) const;
  double GetYawFromQuaternion(const geometry_msgs::msg::Quaternion& q) const;
};

#endif  // PURE_PURSUIT__PURE_PURSUIT_NODE_HPP_
