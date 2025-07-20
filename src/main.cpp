#include "rclcpp/rclcpp.hpp"
#include "pure_pursuit_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PurePursuitNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
