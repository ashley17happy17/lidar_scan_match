#pragma once

#include "common.h"
#include <memory>
#include <mutex>
#include <pcl/filters/voxel_grid.h>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/registration/registration_helper.hpp>

namespace lidar_scan_match_c {

class FrontEndICP {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  FrontEndICP(ros::NodeHandle &nh);

  // Provide initial guess (from IMU predict or constant velocity)
  void setInitialGuess(const Eigen::Matrix4f &guess);

  // Update the HD Map cloud safely from the background thread
  void updateHDMapCloud(const CloudType::Ptr &new_map);

  // Scan matching (small_gicp)
  bool scanMatch(const CloudType::Ptr &source_cloud,
                 const CloudType::Ptr &target_cloud,
                 Eigen::Matrix4f &out_transform, double &out_fitness_score,
                 bool is_turning = false);

  // HD Map matching
  bool scanToHDMapMatch(const CloudType::Ptr &current_cloud,
                        Eigen::Matrix4f &out_transform,
                        double &out_fitness_score, bool is_turning = false, bool is_initialization = false);

  // Loop closure specific functions
  void addKeyframeCloud(const CloudType::Ptr &cloud,
                        const Eigen::Matrix4f &pose);
  void getLocalMap(CloudType::Ptr &out_local_map);
  void shiftLocalMap(const Eigen::Matrix4f &delta_transform);
  void clearLocalMap();
  bool hasKeyframes() const { return !keyframe_clouds_.empty(); }
  size_t getKeyframesSize() const { return keyframe_clouds_.size(); }
  int getKeyframeCount() const { return keyframe_clouds_.size(); }
  bool detectLoopClosure(const Eigen::Matrix4f &current_pose,
                         double search_radius, int &out_loop_index,
                         Eigen::Matrix4f &out_relative_pose,
                         double &out_fitness_score);

  const LoopClosureThreshold &getLoopClosureSettings() const {
    return loop_closure_threshold_;
  }
  const ICPThreshold &getICPThreshold() const { return icp_threshold; }

private:
  small_gicp::RegistrationSetting gicp_settings_;
  LoopClosureThreshold loop_closure_threshold_;
  ICPThreshold icp_threshold;
  bool use_vgicp_;

  // Timing Accumulators
  double total_icp_time_ms_ = 0.0;
  int total_icp_frames_ = 0;
  double first_data_time_ = -1.0;

  std::mutex hd_map_mutex_;
  // holds precomputed downsampled map cloud and its search tree
  std::shared_ptr<small_gicp::PointCloud> hd_map_gicp_cloud_;
  std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> hd_map_gicp_tree_;
  std::shared_ptr<small_gicp::GaussianVoxelMap> hd_map_voxelmap_;

  // Loop closure history
  std::vector<CloudType::Ptr> keyframe_clouds_;
  std::vector<Eigen::Matrix4f> keyframe_poses_;
};

} // namespace lidar_scan_match_c
