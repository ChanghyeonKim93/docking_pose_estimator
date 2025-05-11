#ifndef DOCKING_POSE_ESTIMATOR_ROS2_H_
#define DOCKING_POSE_ESTIMATOR_ROS2_H_

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "core/docking_pose_estimator.h"

using namespace std::chrono_literals;

using ImuMsg = sensor_msgs::msg::Imu;
using LaserScanMsg = sensor_msgs::msg::LaserScan;
using PointCloud2Msg = sensor_msgs::msg::PointCloud2;

namespace docking_pose_estimator {

class DockingPoseEstimatorNode : public rclcpp::Node {
 public:
  DockingPoseEstimatorNode(const std::string& node_name);

 private:
  void CallbackImu(const ImuMsg::SharedPtr msg);
  void CallbackLaserScan(const LaserScanMsg::SharedPtr msg);

 private:
  struct {
    std::string lidar_scan;
    std::string imu;
  } topic_name_;

  rclcpp::Subscription<ImuMsg>::SharedPtr subscriber_imu_;
  rclcpp::Subscription<LaserScanMsg>::SharedPtr subscriber_lidar_scan_;

 private:
  std::shared_ptr<DockingPoseEstimator> estimator_{nullptr};
};

}  // namespace docking_pose_estimator

#endif  // DOCKING_POSE_ESTIMATOR_ROS2_H_
