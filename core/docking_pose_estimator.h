#ifndef DOCKING_POSE_ESTIMATOR_H_
#define DOCKING_POSE_ESTIMATOR_H_

#include <deque>
#include <vector>

#include <eigen3/Eigen/Dense>

namespace docking_pose_estimator {

using Vec3 = Eigen::Vector3d;

struct Parameters {};

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
  std::vector<double> range_list;
  std::vector<double> intensity_list;
};

class DockingPoseEstimator {
 public:
  DockingPoseEstimator(const Parameters& parameters);

  void SetImuData(const ImuData& data);

  void SetLaserScanData(const LaserScanData& data);

 private:
  std::deque<ImuData> imu_data_queue_;
  std::deque<LaserScanData> scan_data_queue_;

  const Parameters parameters_;
};

}  // namespace docking_pose_estimator

#endif  // DOCKING_POSE_ESTIMATOR_H_
