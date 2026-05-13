#pragma once

#include "common.h"

// GTSAM
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
// GTSAM IMU
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/NavState.h>

namespace lidar_scan_match_c {

class BackEndOptimization {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  BackEndOptimization(ros::NodeHandle &nh);

  // Add Odometry derived from Fast-ICP
  void addOdomFactor(double timestamp, const Eigen::Matrix4f &odom_transform,
                     bool is_hd_map_anchor = false, bool scan_match_failed = false, bool is_turning = false);

  // Add Zero Velocity Update Factor
  void addZUPTFactor();

  // Integrate incoming IMU measurements continuously (100Hz+)
  void integrateImuMeasurement(const ImuMeasurement &imu, double dt);

  // Add GPS factor
  void addGpsFactor(const GpsMeasurement &gps);

  // Add HD Map factor
  void addHDMapFactor(const Eigen::Matrix4f &map_matched_pose,
                      double noise_score);

  // Add Loop Closure factor
  void addLoopClosureFactor(int key_idx_1, int key_idx_2,
                            const Eigen::Matrix4f &relative_transform,
                            double noise_score);

  // Perform Optimization step
  void optimize(int num_iterations = 1);

  // Get current optimized graph pose
  gtsam::Pose3 getCurrentPose() const;

  gtsam::Pose3 getPredictedPose() const;
  void setCurrentPose(const gtsam::Pose3 &pose);

  // Loop Closure support
  void addLoopFactor(int historical_idx, const Eigen::Matrix4f &relative_pose,
                    double fitness);

  // Get extrinsics (IMU to Lidar)
  gtsam::Pose3 getExtrinsic() const { return imu2Lidar_; }

private:
  gtsam::NonlinearFactorGraph gtsam_graph_;
  gtsam::Values initial_estimates_;
  gtsam::Values optimized_estimates_;

  std::unique_ptr<gtsam::ISAM2> isam_;
  mutable std::recursive_mutex backend_mutex_;
  int key_index_;

  // IMU Preintegration
  boost::shared_ptr<gtsam::PreintegrationParams> p_;
  std::unique_ptr<gtsam::PreintegratedImuMeasurements> imu_preintegrator_;
  gtsam::NavState prev_state_;
  gtsam::imuBias::ConstantBias prev_bias_;
  gtsam::Pose3 prev_lidar_pose_;

  // Extrinsics
  gtsam::Pose3 imu2Lidar_;
  gtsam::Pose3 imu2Gps_;

  // Noise params
  double accBiasN_, gyrBiasN_;
  
  // Flag to toggle IMU usage for both factor graph and prediction
  bool use_imu_{false};
};

} // namespace lidar_scan_match_c
