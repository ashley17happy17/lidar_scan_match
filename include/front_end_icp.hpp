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
  void updateHDMapCloud(const PointCloudType::Ptr &new_map);

  // Scan matching (small_gicp)
  bool scanMatch(const PointCloudType::Ptr &source_cloud,
                 const PointCloudType::Ptr &target_cloud,
                 Eigen::Matrix4f &out_transform, double &out_fitness_score, bool is_turning = false);

  // HD Map matching
  bool scanToHDMapMatch(const PointCloudType::Ptr &current_cloud,
                        Eigen::Matrix4f &out_transform,
                        double &out_fitness_score, bool is_turning = false);

  // Loop closure specific functions
  void addKeyframeCloud(const PointCloudType::Ptr &cloud,
                        const Eigen::Matrix4f &pose);
  void getLocalMap(PointCloudType::Ptr &out_local_map);
  void shiftLocalMap(const Eigen::Matrix4f &delta_transform);
  bool hasKeyframes() const { return !keyframe_clouds_.empty(); }
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

  std::mutex hd_map_mutex_;
  // holds precomputed downsampled map cloud and its search tree
  std::shared_ptr<small_gicp::PointCloud> hd_map_gicp_cloud_;
  std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> hd_map_gicp_tree_;

  // Loop closure history
  std::vector<PointCloudType::Ptr> keyframe_clouds_;
  std::vector<Eigen::Matrix4f> keyframe_poses_;
};

} // namespace lidar_scan_match_c
