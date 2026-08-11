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

#include <deque>
#include <vector>

namespace lidar_scan_match_c {

class BackEndOptimization {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  BackEndOptimization(ros::NodeHandle &nh);

  // Add Odometry derived from Fast-ICP
  void addOdomFactor(double timestamp, const Eigen::Matrix4f &odom_transform,
                     bool is_hd_map_anchor = false,
                     bool scan_match_failed = false, bool is_turning = false);

  // Add Zero Velocity Update Factor
  void addZUPTFactor();

  // Integrate incoming IMU measurements continuously (100Hz+)
  void integrateImuMeasurement(const ImuMeasurement &imu, double dt);

  // Queue a GPS factor. The measurement is NOT bound to a node here: it is
  // held until the graph contains a node at or after gps.timestamp, then bound
  // to whichever node is nearest in time (see flushGpsQueue).
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
  // Bind queued GPS measurements to the graph node nearest in time. Called
  // whenever a fix arrives or a new node is created.
  void flushGpsQueue();

  // Index of the node closest in time to t, or -1 if there are none.
  // dt_out = t - key_times_[result] (positive means t is after the node).
  int findNearestKeyByTime(double t, double &dt_out) const;

  gtsam::NonlinearFactorGraph gtsam_graph_;
  gtsam::Values initial_estimates_;
  gtsam::Values optimized_estimates_;

  std::unique_ptr<gtsam::ISAM2> isam_;
  mutable std::recursive_mutex backend_mutex_;
  int key_index_;

  // Timestamp of every graph node, so measurements can be associated by time
  // rather than by "whatever key exists right now". key_times_[i] is the
  // timestamp of X(i), hence key_times_.size() == key_index_ at all times.
  std::vector<double> key_times_;

  // GPS fixes awaiting a time-matched node.
  std::deque<GpsMeasurement> gps_queue_;
  int last_gps_bound_key_{-1};
  double gps_assoc_max_dt_;
  size_t gps_queue_max_;

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
