#include "aruco_det/aruco_node.hpp"
#include <memory>

int main(int argc, char ** argv) {
  
  rclcpp::init(argc, argv);
  auto node = std::make_shared<aruco::ArucoDetc>("aruco_det", rclcpp::NodeOptions());

  rclcpp::spin(node);
  rclcpp::shutdown();

  node = nullptr;

  return 0;
}
