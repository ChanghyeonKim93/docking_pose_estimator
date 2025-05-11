#include <exception>
#include <iostream>
#include <string>

#include "ros2/docking_pose_estimator_ros2.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    std::string node_name = "docking_pose_estimator_node";
    rclcpp::spin(
        std::make_shared<docking_pose_estimator::DockingPoseEstimatorNode>(
            node_name));
    rclcpp::shutdown();
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
