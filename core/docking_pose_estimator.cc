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
  timed_point_cloud_queue_.push_back(ConvertToTimedPointCloud(data));
  std::cerr << "timed_point_cloud_queue_ size: "
            << timed_point_cloud_queue_.size() << std::endl;

  if (timed_point_cloud_queue_.empty()) return;

  local_dock_pose_ = ComputeLocalDockPose(timed_point_cloud_queue_.back());

  constexpr double kMaxQueueSize = 100;
  if (timed_point_cloud_queue_.size() > kMaxQueueSize)
    timed_point_cloud_queue_.pop_front();
}

std::optional<TimedPose2D> DockingPoseEstimator::GetLocalDockPose() const {
  return local_dock_pose_;
}

TimedPointCloud DockingPoseEstimator::ConvertToTimedPointCloud(
    const LaserScanData& data) {
  TimedPointCloud timed_point_cloud;
  timed_point_cloud.time = data.timestamp;
  timed_point_cloud.data.reserve(data.ranges.size());
  for (size_t index = 0; index < data.ranges.size(); ++index) {
    const auto range = data.ranges.at(index);
    if (range < data.range_min || range > data.range_max) continue;
    const double point_time =
        data.time_increment * index + timed_point_cloud.time;
    const double angle = data.angle_min + index * data.angle_increment;
    const double x = std::cos(angle) * range;
    const double y = std::sin(angle) * range;
    TimedPoint timed_point;
    timed_point.time = point_time;
    timed_point.point.x() = x;
    timed_point.point.y() = y;
    timed_point_cloud.data.emplace_back(timed_point);
  }
  return timed_point_cloud;
}

std::optional<TimedPose2D> DockingPoseEstimator::ComputeLocalDockPose(
    const TimedPointCloud& timed_point_cloud) {
  const auto& type = parameters_.pose_estimator_type;
  switch (type) {
    case PoseEstimatorType::kVShape: {
      return ComputeLocalDockPoseUsingVShape(timed_point_cloud);
    } break;
    default:
      return std::nullopt;
  }
}

std::optional<TimedPose2D>
DockingPoseEstimator::ComputeLocalDockPoseUsingVShape(
    const TimedPointCloud& timed_point_cloud) {
  if (timed_point_cloud.data.size() < 3) return std::nullopt;

  constexpr size_t kSamplingStep{3};
  constexpr double kEps{0.05};

  bool v_shape_found{false};

  std::optional<TimedPose2D> local_dock_pose{std::nullopt};
  local_dock_pose = TimedPose2D();
  for (size_t index = kSamplingStep;
       index < timed_point_cloud.data.size() - kSamplingStep; ++index) {
    const auto& p1 = timed_point_cloud.data.at(index - kSamplingStep);
    const auto& p2 = timed_point_cloud.data.at(index);
    const auto& p3 = timed_point_cloud.data.at(index + kSamplingStep);

    // TODO(@ChanghyeonKim93): Add FoV check
    Vec2 v12 = p1.point - p2.point;
    v12.normalize();
    Vec2 v32 = p3.point - p2.point;
    v32.normalize();

    const double dot_product = v12.dot(v32);
    const double cross_product = v12.x() * v32.y() - v12.y() * v32.x();

    if (dot_product < kEps && std::abs(std::abs(cross_product) - 1.0) < kEps) {
      v_shape_found = true;
      std::cerr << "dot: " << dot_product << " , cross: " << cross_product
                << std::endl;
      std::cerr << "Found V shape at index: " << index << std::endl;
      break;
    }
  }

  if (!v_shape_found) {
    std::cerr << "V shape not found" << std::endl;
    return std::nullopt;
  }

  // local_dock_pose->pose.x() /= timed_point_cloud.data.size();
  // local_dock_pose->pose.y() /= timed_point_cloud.data.size();
  // local_dock_pose->pose.z() =
  //     std::atan2(local_dock_pose->pose.y(), local_dock_pose->pose.x());
  return local_dock_pose;
}

}  // namespace docking_pose_estimator