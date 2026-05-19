#include "back_end_optimization.hpp"
#include <gtsam/inference/Symbol.h>

using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V; // Vel   (x,y,z)
using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

namespace lidar_scan_match_c {

BackEndOptimization::BackEndOptimization(ros::NodeHandle &nh) : key_index_(0) {
  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  isam_ = std::make_unique<gtsam::ISAM2>(parameters);

  // Initialize IMU Preintegration Params
  double accNoise, gyrNoise, accBias, gyrBias, gravity;
  nh.param<double>("imuAccNoise", accNoise, 1e-3);
  nh.param<double>("imuGyrNoise", gyrNoise, 1e-3);
  nh.param<double>("imuAccBiasN", accBias, 1e-5);
  nh.param<double>("imuGyrBiasN", gyrBias, 1e-5);
  nh.param<double>("imuGravity", gravity, 9.81);

  p_ = gtsam::PreintegrationParams::MakeSharedU(gravity);
  p_->accelerometerCovariance = gtsam::Matrix33::Identity() * pow(accNoise, 2);
  p_->gyroscopeCovariance = gtsam::Matrix33::Identity() * pow(gyrNoise, 2);
  p_->integrationCovariance = gtsam::Matrix33::Identity() * pow(1e-4, 2);

  // 保存 Noise params 用於後續 Graph Factor
  accBiasN_ = accBias;
  gyrBiasN_ = gyrBias;

  prev_bias_ = gtsam::imuBias::ConstantBias();
  imu_preintegrator_ =
      std::make_unique<gtsam::PreintegratedImuMeasurements>(p_, prev_bias_);

  // Extrinsics
  std::vector<double> extT, extR, extGpsT;
  nh.getParam("extrinsicTrans", extT);
  nh.getParam("extrinsicRot", extR);
  nh.getParam("extrinsicGpsTrans", extGpsT);

  Eigen::Matrix3d R;
  if (extR.size() == 9) {
    R << extR[0], extR[1], extR[2], extR[3], extR[4], extR[5], extR[6], extR[7],
        extR[8];
  } else {
    R = Eigen::Matrix3d::Identity();
  }
  Eigen::Vector3d T = (extT.size() == 3)
                          ? Eigen::Vector3d(extT[0], extT[1], extT[2])
                          : Eigen::Vector3d::Zero();
  imu2Lidar_ = gtsam::Pose3(gtsam::Rot3(R), gtsam::Point3(T));

  Eigen::Vector3d gpsT =
      (extGpsT.size() == 3)
          ? Eigen::Vector3d(extGpsT[0], extGpsT[1], extGpsT[2])
          : Eigen::Vector3d::Zero();
  // Assuming rotation from IMU to GPS is identity for now
  imu2Gps_ = gtsam::Pose3(gtsam::Rot3::Identity(), gtsam::Point3(gpsT));
}

void BackEndOptimization::integrateImuMeasurement(const ImuMeasurement &imu,
                                                  double dt) {
  imu_preintegrator_->integrateMeasurement(imu.linear_acceleration,
                                           imu.angular_velocity, dt);
}

void BackEndOptimization::addOdomFactor(double timestamp,
                                        const Eigen::Matrix4f &odom_transform,
                                        bool is_hd_map_anchor,
                                        bool scan_match_failed,
                                        bool is_turning) {
  if (odom_transform.hasNaN()) {
    ROS_ERROR_THROTTLE(1.0, "[BackEnd] Received NaN in odom_transform! "
                            "Resetting IMU preintegration.");
    imu_preintegrator_->resetIntegrationAndSetBias(prev_bias_);
    return;
  }

  gtsam::Pose3 lidar_pose(odom_transform.cast<double>());
  gtsam::Pose3 imu_pose = lidar_pose.compose(imu2Lidar_.inverse());

  use_imu_ = true; // Set to false by user
  if (key_index_ > 0 && use_imu_) {
    const gtsam::PreintegratedImuMeasurements &preint_imu = *imu_preintegrator_;
    if (preint_imu.deltaTij() < 1e-3) {
      ROS_WARN_THROTTLE(1.0,
                        "[BackEnd] IMU msg lacked or dt too small (%.6f). "
                        "Falling back to Lidar Odom for frame %d",
                        preint_imu.deltaTij(), key_index_);
      use_imu_ = false;
    }
  }

  if (key_index_ == 0) {
    // ... setup priors ...
    prev_state_ = gtsam::NavState(imu_pose, gtsam::Vector3::Zero());
    prev_lidar_pose_ = lidar_pose;

    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise =
        gtsam::noiseModel::Diagonal::Variances(
            (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2)
                .finished());
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise =
        gtsam::noiseModel::Isotropic::Sigma(3, 1e-1);
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise =
        gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);

    gtsam_graph_.addPrior(X(0), prev_state_.pose(), priorPoseNoise);
    gtsam_graph_.addPrior(V(0), prev_state_.velocity(), priorVelNoise);
    gtsam_graph_.addPrior(B(0), prev_bias_, priorBiasNoise);

    initial_estimates_.insert(X(0), prev_state_.pose());
    initial_estimates_.insert(V(0), prev_state_.velocity());
    initial_estimates_.insert(B(0), prev_bias_);
  } else {
    const gtsam::PreintegratedImuMeasurements &preint_imu = *imu_preintegrator_;

    // Add IMU Factor
    if (use_imu_) {
      gtsam_graph_.add(gtsam::ImuFactor(X(key_index_ - 1), V(key_index_ - 1),
                                        X(key_index_), V(key_index_),
                                        B(key_index_ - 1), preint_imu));
    }

    // Add Bias Random Walk Factor (使用使用者自訂參數，並依據實際時間間隔進行
    // scaling)
    double dt_ij = (key_index_ > 0) ? imu_preintegrator_->deltaTij() : 0.1;
    if (dt_ij < 1e-6)
      dt_ij = 0.1;
    gtsam::Vector6 bias_sigmas;
    bias_sigmas << accBiasN_ * sqrt(dt_ij), accBiasN_ * sqrt(dt_ij),
        accBiasN_ * sqrt(dt_ij), gyrBiasN_ * sqrt(dt_ij),
        gyrBiasN_ * sqrt(dt_ij), gyrBiasN_ * sqrt(dt_ij);
    gtsam::noiseModel::Diagonal::shared_ptr bias_noise =
        gtsam::noiseModel::Diagonal::Sigmas(bias_sigmas);

    gtsam_graph_.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
        B(key_index_ - 1), B(key_index_), gtsam::imuBias::ConstantBias(),
        bias_noise));

    // Add Velocity Continuity Factor (Extra robustness)
    gtsam::noiseModel::Diagonal::shared_ptr vel_noise =
        gtsam::noiseModel::Isotropic::Sigma(3, 1e-1);
    gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Vector3>(
        V(key_index_ - 1), V(key_index_), gtsam::Vector3::Zero(), vel_noise));

    // Add Lidar Odometry Factor
    gtsam::Pose3 prev_imu_pose = prev_lidar_pose_.compose(imu2Lidar_.inverse());
    gtsam::Pose3 relative_odom = prev_imu_pose.between(imu_pose);

    // Safety check for lidar odom jump or HD Map Anchor
    if (is_hd_map_anchor || relative_odom.translation().norm() > 10.0) {
      if (is_hd_map_anchor) {
        // Allow snapping to HD map
      } else {
        ROS_WARN_THROTTLE(1.0,
                          "[BackEnd] Large odom jump detected (%.2fm)! Using "
                          "loose constraints.",
                          relative_odom.translation().norm());
      }
      gtsam::noiseModel::Diagonal::shared_ptr loose_noise =
          gtsam::noiseModel::Isotropic::Sigma(
              6, 5.0); // 5.0 meter/rad std dev, extremely loose!
      gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
          X(key_index_ - 1), X(key_index_), relative_odom, loose_noise));
    } else {
      if (scan_match_failed) {
        // 如果是 Skip Frame (例如依賴 IMU 預測)，放寬 Odom 約束
        gtsam::noiseModel::Diagonal::shared_ptr loose_noise =
            gtsam::noiseModel::Isotropic::Sigma(6, 5.0);
        gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
            X(key_index_ - 1), X(key_index_), relative_odom, loose_noise));
      } else {
        // 轉彎時，我們將 Lidar Odometry 的變異數大幅放大
        double odom_rot_var = is_turning ? 1e-3 : 1e-4;
        double odom_trans_var = is_turning ? 2e-2 : 1e-2;
        gtsam::noiseModel::Diagonal::shared_ptr odom_noise =
            gtsam::noiseModel::Diagonal::Variances(
                (gtsam::Vector(6) << odom_rot_var, odom_rot_var, odom_rot_var,
                 odom_trans_var, odom_trans_var, odom_trans_var)
                    .finished());
        gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
            X(key_index_ - 1), X(key_index_), relative_odom, odom_noise));
      }

      // 側向移動與垂直移動約制 (Non-Holonomic Constraint, NHC)
      // 非常重要：即使是 Skip Frame，車子物理上依然不能橫向滑行，必須保留 NHC！
      gtsam::Pose3 nhc_pose(
          relative_odom.rotation(),
          gtsam::Point3(relative_odom.translation().x(), 0.0, 0.0));

      double nhc_y_var = is_turning ? 1e-1 : 1e-4; // 放寬轉彎時的側向變異數
      gtsam::noiseModel::Diagonal::shared_ptr nhc_noise =
          gtsam::noiseModel::Diagonal::Variances(
              (gtsam::Vector(6) << 1e6, 1e6, 1e6, 1e6, nhc_y_var, 1e-3)
                  .finished());
      gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
          X(key_index_ - 1), X(key_index_), nhc_pose, nhc_noise));
    }

    // Predict and Insert
    gtsam::NavState prop_state;
    if (use_imu_) {
      prop_state = imu_preintegrator_->predict(prev_state_, prev_bias_);
      prop_state = gtsam::NavState(imu_pose, prop_state.velocity());
    } else {
      prop_state = gtsam::NavState(imu_pose, prev_state_.velocity());
    }
    initial_estimates_.insert(X(key_index_), prop_state.pose());
    initial_estimates_.insert(V(key_index_), prop_state.velocity());
    initial_estimates_.insert(B(key_index_), prev_bias_);

    prev_state_ = prop_state;
    prev_lidar_pose_ = lidar_pose;
  }

  // Reset IMU Integrator
  imu_preintegrator_->resetIntegrationAndSetBias(prev_bias_);
  key_index_++;
}

void BackEndOptimization::addZUPTFactor() {
  std::lock_guard<std::recursive_mutex> lock(backend_mutex_);
  if (key_index_ == 0)
    return;

  // 零速度更新 (ZUPT): 強制讓當前節點的速度趨近於 0
  // 這可以極大程度消除 IMU 靜止時累積的漂移
  gtsam::noiseModel::Diagonal::shared_ptr zero_vel_noise =
      gtsam::noiseModel::Isotropic::Sigma(3, 1e-3);
  gtsam_graph_.add(gtsam::PriorFactor<gtsam::Vector3>(
      V(key_index_ - 1), gtsam::Vector3::Zero(), zero_vel_noise));

  // 同時清空 IMU 預測器的殘餘速度，防止預測位姿發生漂移，從而破壞前端的靜止偵測
  prev_state_ = gtsam::NavState(prev_state_.pose(), gtsam::Vector3::Zero());
}

void BackEndOptimization::addGpsFactor(const GpsMeasurement &gps) {
  std::lock_guard<std::recursive_mutex> lock(backend_mutex_);
  if (key_index_ == 0)
    return; // Graph not ready

  // Create a GPS factor.
  // CRITICAL FIX: The GPS measures the antenna position, not the IMU center!
  // We must transform the GPS measurement back to the IMU center using the
  // current heading/orientation.
  gtsam::Point3 gps_point(gps.latitude, gps.longitude, gps.altitude);

  gtsam::Pose3 current_imu_pose;
  if (initial_estimates_.exists(X(key_index_ - 1))) {
    current_imu_pose = initial_estimates_.at<gtsam::Pose3>(X(key_index_ - 1));
  } else {
    current_imu_pose = prev_state_.pose();
  }

  // gps_point = imu_position + R_world_imu * imu2Gps_.translation()
  // imu_position = gps_point - R_world_imu * imu2Gps_.translation()
  gtsam::Point3 imu_position =
      gps_point - current_imu_pose.rotation() * imu2Gps_.translation();

  gtsam::noiseModel::Diagonal::shared_ptr gps_noise =
      gtsam::noiseModel::Diagonal::Variances(gps.covariance_diag);

  gtsam::GPSFactor gps_factor(X(key_index_ - 1), imu_position, gps_noise);
  gtsam_graph_.add(gps_factor);
}

void BackEndOptimization::addHDMapFactor(
    const Eigen::Matrix4f &map_matched_pose, double noise_score) {
  if (key_index_ == 0)
    return;

  gtsam::Pose3 lidar_pose(map_matched_pose.cast<double>());
  gtsam::Pose3 imu_pose = lidar_pose.compose(imu2Lidar_.inverse());

  // 強制讓 HD Map 的 Prior Factor 具有「絕對權威」，權重必須遠高於 Odom，
  // 否則 ISAM2 會因為 Odom Factor 導致當前幀「被往回拉」。
  double trans_n = 1e-4; // std dev 0.1mm
  double rot_n = 1e-4;

  // Diagonal::Sigmas takes (Vector6): (roll, pitch, yaw, x, y, z)
  gtsam::Vector6 noise_sigmas;
  noise_sigmas << rot_n, rot_n, rot_n, trans_n, trans_n, trans_n;
  gtsam::noiseModel::Diagonal::shared_ptr map_noise =
      gtsam::noiseModel::Diagonal::Sigmas(noise_sigmas);

  // PriorFactor pulls the specific keyframe to the HD MAP global coordinates
  gtsam_graph_.addPrior(X(key_index_ - 1), imu_pose, map_noise);

  if (initial_estimates_.exists(X(key_index_ - 1))) {
    initial_estimates_.update(X(key_index_ - 1), imu_pose);
  }
}

void BackEndOptimization::addLoopClosureFactor(
    int key_idx_1, int key_idx_2, const Eigen::Matrix4f &relative_transform,
    double noise_score) {
  gtsam::Pose3 relative_pose(relative_transform.cast<double>());
  gtsam::Pose3 imu_relative =
      imu2Lidar_.inverse().compose(relative_pose).compose(imu2Lidar_);

  double n = std::max(1e-2, noise_score);
  gtsam::noiseModel::Diagonal::shared_ptr loop_noise =
      gtsam::noiseModel::Isotropic::Sigma(6, n);

  gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
      X(key_idx_1), X(key_idx_2), imu_relative, loop_noise));
}

void BackEndOptimization::setCurrentPose(const gtsam::Pose3 &pose) {
  std::lock_guard<std::recursive_mutex> lock(backend_mutex_);

  // Transform Lidar pose back to IMU pose for state update
  gtsam::Pose3 imu_pose = pose.compose(imu2Lidar_.inverse());

  prev_lidar_pose_ = pose;
  prev_state_ = gtsam::NavState(imu_pose, prev_state_.velocity());

  ROS_DEBUG("[BackEnd] External pose sync applied.");
}

void BackEndOptimization::optimize(int num_iterations) {
  std::lock_guard<std::recursive_mutex> lock(backend_mutex_);
  if (gtsam_graph_.empty())
    return;

  try {
    isam_->update(gtsam_graph_, initial_estimates_);

    // 如果大於 1，代表發生了 HD Map 匹配，圖中產生了極大的 Prior 錨點張力。
    // 為了讓這股張力能徹底回推給所有歷史軌跡，我們強制給予足夠的迭代次數 (例如
    // 15 次)
    // 否則優化器來不及收斂，當前幀就會停在半路，產生「被往回拉」的視覺誤差。
    int iters = (num_iterations > 1) ? 15 : 1;
    for (int i = 1; i < iters; ++i) {
      isam_->update();
    }
    optimized_estimates_ = isam_->calculateEstimate();

    // Update prev_state and bias with optimized results
    if (key_index_ > 0 && optimized_estimates_.exists(X(key_index_ - 1))) {
      prev_state_ = gtsam::NavState(
          optimized_estimates_.at<gtsam::Pose3>(X(key_index_ - 1)),
          optimized_estimates_.at<gtsam::Vector3>(V(key_index_ - 1)));
      prev_bias_ = optimized_estimates_.at<gtsam::imuBias::ConstantBias>(
          B(key_index_ - 1));
      imu_preintegrator_->resetIntegrationAndSetBias(prev_bias_);
    }
  } catch (const gtsam::IndeterminantLinearSystemException &e) {
    ROS_ERROR(
        "[BackEnd] ISAM2 Singular! Resetting ISAM2 internal state... (Key %d)",
        key_index_);

    // Soft Reset: Re-initialize ISAM2 to clear bad internal state
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam_ = std::make_unique<gtsam::ISAM2>(parameters);

    // Re-add current state as prior to keep continuity
    gtsam::NonlinearFactorGraph reset_graph;
    gtsam::Values reset_values;

    gtsam::noiseModel::Diagonal::shared_ptr priorNoise =
        gtsam::noiseModel::Isotropic::Sigma(6, 0.1);
    reset_graph.addPrior(X(key_index_ - 1), prev_state_.pose(), priorNoise);
    reset_graph.addPrior(V(key_index_ - 1), prev_state_.velocity(),
                         gtsam::noiseModel::Isotropic::Sigma(3, 0.1));
    reset_graph.addPrior(B(key_index_ - 1), prev_bias_,
                         gtsam::noiseModel::Isotropic::Sigma(6, 0.01));

    reset_values.insert(X(key_index_ - 1), prev_state_.pose());
    reset_values.insert(V(key_index_ - 1), prev_state_.velocity());
    reset_values.insert(B(key_index_ - 1), prev_bias_);

    isam_->update(reset_graph, reset_values);
    optimized_estimates_ = isam_->calculateEstimate();
  } catch (const std::exception &e) {
    ROS_ERROR("[BackEnd] General Optimization Exception: %s", e.what());
  }

  // Clear for next frame
  gtsam_graph_.resize(0);
  initial_estimates_.clear();
}

gtsam::Pose3 BackEndOptimization::getCurrentPose() const {
  std::lock_guard<std::recursive_mutex> lock(backend_mutex_);
  if (optimized_estimates_.empty() || key_index_ == 0)
    return gtsam::Pose3::Identity();

  // Safety: If the last optimization failed or didn't include the latest key,
  // find the most recent available key instead of crashing.
  for (int i = key_index_ - 1; i >= 0; --i) {
    if (optimized_estimates_.exists(X(i))) {
      gtsam::Pose3 imu_pose = optimized_estimates_.at<gtsam::Pose3>(X(i));
      return imu_pose.compose(imu2Lidar_);
    }
  }

  return gtsam::Pose3::Identity();
}

gtsam::Pose3 BackEndOptimization::getPredictedPose() const {
  std::lock_guard<std::recursive_mutex> lock(backend_mutex_);
  if (!imu_preintegrator_ || key_index_ == 0)
    return gtsam::Pose3::Identity();

  // CRITICAL FIX: If IMU is disabled, DO NOT use it to predict the next pose!
  // This was causing massive drift because bad IMU data was still being
  // integrated to predict the frontend's guess, leading to a jump > 3.0m and
  // causing ICP failure!
  if (!use_imu_) {
    return prev_state_.pose().compose(imu2Lidar_);
  }

  // Use IMU preintegration to predict state from the last optimized keyframe
  try {
    gtsam::NavState predicted_state =
        imu_preintegrator_->predict(prev_state_, prev_bias_);
    return predicted_state.pose().compose(imu2Lidar_);
  } catch (...) {
    return prev_state_.pose().compose(imu2Lidar_);
  }
}

void BackEndOptimization::addLoopFactor(int historical_idx,
                                        const Eigen::Matrix4f &relative_pose,
                                        double fitness) {
  std::lock_guard<std::recursive_mutex> lock(backend_mutex_);

  // relative_pose is from current_frame back to historical_frame (T_hist_curr)
  gtsam::Pose3 rel_pose(relative_pose.cast<double>());

  // Noise model based on fitness (lower is better, but we cap the minimum)
  double sigma = std::max(0.1, fitness);
  gtsam::noiseModel::Diagonal::shared_ptr loop_noise =
      gtsam::noiseModel::Isotropic::Sigma(6, sigma);

  gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
      X(historical_idx), X(key_index_), rel_pose, loop_noise));

  ROS_INFO("[BackEnd] Added Loop Closure Factor: %d <-> %d (fitness: %.3f)",
           historical_idx, key_index_, fitness);
}

} // namespace lidar_scan_match_c
