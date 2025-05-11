#ifndef DOCKING_POSE_ESTIMATOR_H_
#define DOCKING_POSE_ESTIMATOR_H_

#include <deque>
#include <optional>
#include <vector>

#include <Eigen/Dense>

namespace docking_pose_estimator {

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Pose2D = Eigen::Isometry2d;
using Pose3D = Eigen::Isometry3d;

enum class PoseEstimatorType { kVShape = 0, kQRCode = 1 };

struct Parameters {
  PoseEstimatorType pose_estimator_type{PoseEstimatorType::kVShape};
};

struct ImuData {
  double timestamp{0.0};
  Vec3 linear_acceleration{Vec3::Zero()};
  Vec3 angular_velocity{Vec3::Zero()};
};

struct LaserScanData {
  double timestamp{0.0};
  double angle_min{0.0};
  double angle_max{0.0};
  double angle_increment{0.0};
  double time_increment{0.0};  // time between points
  double scan_time{0.0};       // time between scans
  double range_min{0.0};
  double range_max{0.0};
  std::vector<double> ranges;
  std::vector<double> intensities;
};

struct TimedPoint {
  double time{0.0};
  Vec2 point{Vec2::Zero()};
};

struct TimedPointCloud {
  double time{0.0};
  std::vector<TimedPoint> data;  // <timestamp, point>
};

struct TimedPose2D {
  double time{0.0};
  Pose2D pose{Pose2D::Identity()};
};

class DockingPoseEstimator {
 public:
  DockingPoseEstimator(const Parameters& parameters);

  void SetImuData(const ImuData& data);

  void SetLaserScanData(const LaserScanData& data);

  std::optional<TimedPose2D> GetLocalDockPose() const;

 private:
  TimedPointCloud ConvertToTimedPointCloud(const LaserScanData& data);
  std::optional<TimedPose2D> ComputeLocalDockPose(
      const TimedPointCloud& timed_point_cloud);
  std::optional<TimedPose2D> ComputeLocalDockPoseUsingVShape(
      const TimedPointCloud& timed_point_cloud);

  std::deque<ImuData> imu_data_queue_;
  std::deque<TimedPointCloud> timed_point_cloud_queue_;

  std::optional<TimedPose2D> local_dock_pose_{std::nullopt};

  const Parameters parameters_;
};

}  // namespace docking_pose_estimator

#endif  // DOCKING_POSE_ESTIMATOR_H_
