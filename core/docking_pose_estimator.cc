#include "core/docking_pose_estimator.h"

#include <iostream>

namespace docking_pose_estimator {

DockingPoseEstimator::DockingPoseEstimator(const Parameters& parameters)
    : parameters_(parameters) {}

void DockingPoseEstimator::SetImuData(const ImuData& data) {
  imu_data_queue_.push_back(data);
  std::cerr << "imu queue size: " << imu_data_queue_.size() << std::endl;

  constexpr double kMaxQueueSize = 1000;
  if (imu_data_queue_.size() > kMaxQueueSize) imu_data_queue_.pop_front();
}

void DockingPoseEstimator::SetLaserScanData(const LaserScanData& data) {
  scan_data_queue_.push_back(data);
  std::cerr << "scan_data_queue_ size: " << scan_data_queue_.size()
            << std::endl;

  constexpr double kMaxQueueSize = 100;
  if (scan_data_queue_.size() > kMaxQueueSize) scan_data_queue_.pop_front();
}

}  // namespace docking_pose_estimator