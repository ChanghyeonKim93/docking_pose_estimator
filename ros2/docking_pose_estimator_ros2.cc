#include "ros2/docking_pose_estimator_ros2.h"

#include "core/docking_pose_estimator.h"

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace docking_pose_estimator {

DockingPoseEstimatorNode::DockingPoseEstimatorNode(const std::string& node_name)
    : Node(node_name) {
  std::cerr << "node starts.\n";

  Parameters parameters;
  estimator_ = std::make_shared<DockingPoseEstimator>(parameters);

  // Publishers
  // pub_pose_ =
  //     this->create_publisher<nav_msgs::msg::Odometry>(topicname_pose_, 10);

  subscriber_imu_ = this->create_subscription<ImuMsg>(
      "/imu/data", rclcpp::SensorDataQoS(),
      std::bind(&DockingPoseEstimatorNode::CallbackImu, this,
                std::placeholders::_1));
  subscriber_lidar_scan_ = this->create_subscription<LaserScanMsg>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&DockingPoseEstimatorNode::CallbackLaserScan, this,
                std::placeholders::_1));
}

void DockingPoseEstimatorNode::CallbackImu(const ImuMsg::SharedPtr msg) {
  ImuData imu_data;
  const double timestamp =
      msg->header.stamp.sec +
      static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
  imu_data.timestamp = timestamp;
  imu_data.linear_acceleration.x() = msg->linear_acceleration.x;
  imu_data.linear_acceleration.y() = msg->linear_acceleration.y;
  imu_data.linear_acceleration.z() = msg->linear_acceleration.z;
  imu_data.angular_velocity.x() = msg->angular_velocity.x;
  imu_data.angular_velocity.y() = msg->angular_velocity.y;
  imu_data.angular_velocity.z() = msg->angular_velocity.z;

  estimator_->SetImuData(imu_data);
}

void DockingPoseEstimatorNode::CallbackLaserScan(
    const LaserScanMsg::SharedPtr msg) {
  LaserScanData scan_data;
  const double timestamp =
      msg->header.stamp.sec +
      static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
  scan_data.timestamp = timestamp;
  scan_data.angle_min = msg->angle_min;
  scan_data.angle_max = msg->angle_max;
  scan_data.angle_increment = msg->angle_increment;
  scan_data.range_min = msg->range_min;
  scan_data.range_max = msg->range_max;
  scan_data.time_increment = msg->time_increment;
  scan_data.scan_time = msg->scan_time;
  const size_t num_points = msg->ranges.size();
  scan_data.ranges.reserve(num_points);
  scan_data.intensities.reserve(num_points);
  for (size_t index = 0; index < num_points; ++index) {
    scan_data.ranges.push_back(msg->ranges.at(index));
    scan_data.intensities.push_back(msg->intensities.at(index));
  }

  estimator_->SetLaserScanData(scan_data);
}

}  // namespace docking_pose_estimator
