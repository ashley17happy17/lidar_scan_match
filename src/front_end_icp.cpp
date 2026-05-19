#include "front_end_icp.hpp"
#include <memory>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/registration_helper.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>
#include <thread>

namespace lidar_scan_match_c {

std::shared_ptr<small_gicp::PointCloud>
toGicpCloud(const PointCloudType::Ptr &pcl_cloud) {
  auto gicp_cloud = std::make_shared<small_gicp::PointCloud>();
  gicp_cloud->points.resize(pcl_cloud->size());
  for (size_t i = 0; i < pcl_cloud->size(); ++i) {
    gicp_cloud->points[i] =
        Eigen::Vector4f(pcl_cloud->points[i].x, pcl_cloud->points[i].y,
                        pcl_cloud->points[i].z, 1.0f)
            .cast<double>();
  }
  return gicp_cloud;
}

FrontEndICP::FrontEndICP(ros::NodeHandle &nh) {
  bool loop_enable;
  int max_iterations, num_threads;
  double rot_eps, trans_eps, fit_eps, max_dist_sq, loop_search_radius,
      loop_fitness_score, max_fitness, max_trans, max_rot;
  // --- ICP Parameters ---
  nh.param<int>("icp_max_iterations", max_iterations, 100);
  nh.param<double>("icp_rotation_epsilon", rot_eps, 1e-2);
  nh.param<double>("icp_transformation_epsilon", trans_eps, 1e-6);
  nh.param<double>("icp_euclidean_fitness_epsilon", fit_eps, 1e-4);
  nh.param<double>("icp_max_dist_sq", max_dist_sq, 9.0);
  nh.param<int>("icp_num_threads", num_threads, 10);

  // Set to GICP (Generalized ICP) - much more robust than standard ICP
  gicp_settings_.type = small_gicp::RegistrationSetting::GICP;

  gicp_settings_.max_correspondence_distance = std::sqrt(max_dist_sq);

  gicp_settings_.max_iterations = max_iterations;
  gicp_settings_.rotation_eps = rot_eps;
  gicp_settings_.translation_eps = trans_eps;
  // Note: Following members are not present in some small_gicp versions
  // gicp_settings_.euclidean_fitness_epsilon = fit_eps;
  // gicp_settings_.max_dist_sq = max_dist_sq;
  gicp_settings_.num_threads = num_threads;

  // --- Loop Closure Parameters ---
  nh.param<bool>("loop_closure_enabled", loop_closure_threshold_.enable, true);
  nh.param<double>("loop_closure_search_radius",
                   loop_closure_threshold_.search_radius, 15.0);
  nh.param<double>("loop_closure_fitness_score",
                   loop_closure_threshold_.fitness_score, 0.3);
  nh.param<double>("loop_closure_travel_distance",
                   loop_closure_threshold_.travel_distance, 30.0);

  // --- ICP Robustness Parameters ---
  nh.param<double>("icp_max_acceptable_fitness_score",
                   icp_threshold.max_fitness, 0.5);
  nh.param<double>("max_single_frame_translation", icp_threshold.max_trans,
                   3.0);
  nh.param<double>("max_single_frame_rotation", icp_threshold.max_rot, 0.5);

  hd_map_gicp_cloud_ = nullptr;
  hd_map_gicp_tree_ = nullptr;
}

void FrontEndICP::setInitialGuess(const Eigen::Matrix4f &guess) {
  // Left for future IMU/Odom prediction seeding
}

void FrontEndICP::updateHDMapCloud(const PointCloudType::Ptr &cloud) {
  if (cloud->empty())
    return;

  // 1. Compute GICP cloud and KdTree OUTSIDE the lock
  auto gicp_cloud = toGicpCloud(cloud);
  auto gicp_tree =
      std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(gicp_cloud);

  // Precompute covariances for GICP
  small_gicp::estimate_covariances_omp(*gicp_cloud, *gicp_tree, 20,
                                       gicp_settings_.num_threads);

  // 2. Only lock for the pointer assignment
  {
    std::lock_guard<std::mutex> lock(hd_map_mutex_);
    hd_map_gicp_cloud_ = gicp_cloud;
    hd_map_gicp_tree_ = gicp_tree;
  }

  ROS_INFO("HD Map updated and small_gicp tree precomputed (non-blocking).");
}

bool FrontEndICP::scanMatch(const PointCloudType::Ptr &source_cloud,
                            const PointCloudType::Ptr &target_cloud,
                            Eigen::Matrix4f &out_transform,
                            double &out_fitness_score, bool is_turning) {
  if (source_cloud->empty() || target_cloud->empty())
    return false;

  auto target_gicp = toGicpCloud(target_cloud);
  auto source_gicp = toGicpCloud(source_cloud);

  small_gicp::KdTree<small_gicp::PointCloud> target_tree(target_gicp);
  small_gicp::KdTree<small_gicp::PointCloud> source_tree(source_gicp);

  auto current_settings = gicp_settings_;
  if (is_turning) {
    // 轉彎時加大搜尋範圍 (原本是 2.0m 太小)，允許 ICP
    // 在旋轉預測稍有偏差時仍能抓到特徵
    current_settings.max_correspondence_distance = 15.0;
    current_settings.max_iterations =
        gicp_settings_.max_iterations * 2; // 增加迭代次數
  }

  // Estimate covariances for both (Requirement for GICP)
  small_gicp::estimate_covariances_omp(*target_gicp, target_tree, 20,
                                       current_settings.num_threads);
  small_gicp::estimate_covariances_omp(*source_gicp, source_tree, 20,
                                       current_settings.num_threads);

  Eigen::Isometry3d init_guess(out_transform.cast<double>());
  auto t0 = std::chrono::steady_clock::now();
  ROS_INFO_THROTTLE(1.0, "[Detect Distance] Max Distance: %.2f.",
                    current_settings.max_correspondence_distance);

  // 使用 GICP 進行對齊，考慮表面幾何，精度與魯棒性更高
  auto result = small_gicp::align(*target_gicp, *source_gicp, target_tree,
                                  init_guess, current_settings);

  auto t1 = std::chrono::steady_clock::now();

  // == 無論成功與否，都先將最後的結果提取出來 ==
  Eigen::Matrix4d T_res = result.T_target_source.matrix();

  // Safety Check: Avoid large jumps
  Eigen::Vector3d trans = T_res.block<3, 1>(0, 3) - init_guess.translation();
  double dist = trans.norm();

  Eigen::Matrix3d rot_diff =
      T_res.block<3, 3>(0, 0) * init_guess.linear().transpose();
  double angle = Eigen::AngleAxisd(rot_diff).angle();

  if (dist > icp_threshold.max_trans ||
      std::abs(angle) > icp_threshold.max_rot) {
    ROS_WARN_THROTTLE(1.0, "[Match] Rejected jump: dist=%.2f, angle=%.2f", dist,
                      angle);
    return false;
  }

  out_transform = T_res.cast<float>();
  out_fitness_score =
      (result.num_inliers > 0) ? (result.error / result.num_inliers) : 1e6;

  /*
  // === DEBUG: 永遠存下匹配完的點雲供 CloudCompare 檢視 ===
  {
    PointCloudType::Ptr matched_pcl_debug(new PointCloudType());
    // 用最後算出的轉換矩陣 (T_res) 來轉換來源點雲
    pcl::transformPointCloud(*source_cloud, *matched_pcl_debug, out_transform);

    // 存下「被轉換後的目前幀」 (Source Matched)
    pcl::io::savePCDFileBinary(
        "/root/catkin_ws/src/lidar_scan_match_c/scan_match_source_matched.pcd",
        *matched_pcl_debug);
    // 存下「上一幀/局部地圖」 (Target)
    pcl::io::savePCDFileBinary(
        "/root/catkin_ws/src/lidar_scan_match_c/scan_match_target.pcd",
        *target_cloud);
  }
  // -------------------------------------------------
  */

  double d_align = std::chrono::duration<double, std::milli>(t1 - t0).count();

  ROS_INFO_THROTTLE(1.0,
                    "[HD Profiler] Align:%.1fms, Fitness: %.4f, Inliers: %ld",
                    d_align, out_fitness_score, result.num_inliers);

  if (result.num_inliers > 100 &&
      out_fitness_score < icp_threshold.max_fitness) {
    return true;
  } else {
    ROS_WARN_THROTTLE(
        1.0,
        "[Match] Local ICP failed geometrically: inliers=%ld, fitness=%.4f",
        result.num_inliers, out_fitness_score);
    return false;
  }
}

bool FrontEndICP::scanToHDMapMatch(const PointCloudType::Ptr &current_cloud,
                                   Eigen::Matrix4f &out_transform,
                                   double &out_fitness_score, bool is_turning) {
  if (!current_cloud || current_cloud->empty())
    return false;

  auto t0 = std::chrono::steady_clock::now();

  // 1. Get HD Map Pointers (Mutex check)
  std::shared_ptr<small_gicp::PointCloud> target_gicp;
  std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> target_tree;
  {
    std::lock_guard<std::mutex> lock(hd_map_mutex_);
    if (!hd_map_gicp_cloud_ || !hd_map_gicp_tree_)
      return false;
    target_gicp = hd_map_gicp_cloud_;
    target_tree = hd_map_gicp_tree_;
  }
  auto t1 = std::chrono::steady_clock::now();

  auto current_settings = gicp_settings_;
  if (is_turning) {
    // 全域地圖匹配同樣加大範圍與迭代次數
    current_settings.max_correspondence_distance = 15.0;
    current_settings.max_iterations = gicp_settings_.max_iterations * 2;
  }

  // 2. Source Cloud Conversion
  auto source_gicp = toGicpCloud(current_cloud);
  auto source_tree =
      std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(source_gicp);
  small_gicp::estimate_covariances_omp(*source_gicp, *source_tree, 20,
                                       current_settings.num_threads);
  auto t2 = std::chrono::steady_clock::now();

  /*
  // --- DEBUG: Save clouds to check overlap ---
  {
    PointCloudType::Ptr source_pcl_debug(new PointCloudType());
    // Transform current cloud by initial guess to see if it's close to map
    pcl::transformPointCloud(*current_cloud, *source_pcl_debug, out_transform);

    pcl::io::savePCDFileASCII(
        "/root/catkin_ws/src/lidar_scan_match_c/scan_to_map_source.pcd",
        *source_pcl_debug);

    // Convert small_gicp target back to PCL for saving
    PointCloudType::Ptr target_pcl_debug(new PointCloudType());

    // 1. 改用 resize 預先劃分好確定的記憶體空間，徹底避開 push_back的長度檢查
    target_pcl_debug->points.resize(target_gicp->points.size());

// 2. 啟動 OpenMP 讓所有 CPU 核心平行轉換這數十萬個點
#pragma omp parallel for num_threads(10) schedule(dynamic)
    for (size_t i = 0; i < target_gicp->points.size(); ++i) {
      const auto &pt = target_gicp->points[i];
      auto &p = target_pcl_debug->points[i];
      p.x = pt.x();
      p.y = pt.y();
      p.z = pt.z();
      p.intensity = 1.0f;
    }

    // 手動補上 PCL 必要的 metadata
    target_pcl_debug->width = target_pcl_debug->points.size();
    target_pcl_debug->height = 1;
    target_pcl_debug->is_dense = true;
    pcl::io::savePCDFileBinary(
        "/root/catkin_ws/src/lidar_scan_match_c/scan_to_map_target.pcd",
        *target_pcl_debug);
  }
  // -------------------------------------------
  */

  // Pass the prebuilt target KdTree to GICP alignment
  Eigen::Isometry3d init_guess(out_transform.cast<double>());
  auto result = small_gicp::align(*target_gicp, *source_gicp, *target_tree,
                                  init_guess, current_settings);
  auto t3 = std::chrono::steady_clock::now();

  /*
  std::cout << "--- T_target_source ---" << std::endl
            << result.T_target_source.matrix() << std::endl;
  std::cout << "converged:" << result.converged << std::endl;
  std::cout << "error:" << result.error << std::endl;
  std::cout << "iterations:" << result.iterations << std::endl;
  std::cout << "num_inliers:" << result.num_inliers << std::endl;
  std::cout << "--- H ---" << std::endl << result.H << std::endl;
  std::cout << "--- b ---" << std::endl << result.b.transpose() << std::endl;
  */

  // == 無論成功與否，都強制提取最新的轉換矩陣 ==
  Eigen::Matrix4d T_res = result.T_target_source.matrix();
  out_transform = T_res.cast<float>();
  out_fitness_score =
      (result.num_inliers > 0) ? (result.error / result.num_inliers) : 1e6;

  /*
// === DEBUG: 永遠存下 光達對地圖(HD Map) 的匹配結果 ===
{
  PointCloudType::Ptr matched_pcl_debug(new PointCloudType());
  pcl::transformPointCloud(*current_cloud, *matched_pcl_debug, out_transform);
  pcl::io::savePCDFileBinary(
      "/root/catkin_ws/src/lidar_scan_match_c/scan_to_map_source_matched.pcd",
      *matched_pcl_debug);
}
// -----------------------------------
*/

  double d_lock = std::chrono::duration<double, std::milli>(t1 - t0).count();
  double d_conv = std::chrono::duration<double, std::milli>(t2 - t1).count();
  double d_align = std::chrono::duration<double, std::milli>(t3 - t2).count();

  ROS_INFO_THROTTLE(
      1.0, "[HD Profiler] Lock:%.1fms, ConvSource:%.1fms, Align:%.1fms", d_lock,
      d_conv, d_align);

  // == 最後才回報真假值給外部流程 ==
  return result.converged;
}

void FrontEndICP::addKeyframeCloud(const PointCloudType::Ptr &cloud,
                                   const Eigen::Matrix4f &pose) {
  PointCloudType::Ptr cloned_cloud(new PointCloudType(*cloud));
  keyframe_clouds_.push_back(cloned_cloud);
  keyframe_poses_.push_back(pose);
}

void FrontEndICP::clearLocalMap() {
  keyframe_clouds_.clear();
  keyframe_poses_.clear();
}

void FrontEndICP::shiftLocalMap(const Eigen::Matrix4f &delta_transform) {
  for (auto &pose : keyframe_poses_) {
    pose = delta_transform * pose;
  }
}

void FrontEndICP::getLocalMap(PointCloudType::Ptr &out_local_map) {
  out_local_map->clear();
  int num_keyframes = keyframe_clouds_.size();
  if (num_keyframes == 0)
    return;

  // Use a sliding window to build a FAST local map
  int window_size = 30;
  int start_idx = std::max(0, num_keyframes - window_size);

  for (int i = start_idx; i < num_keyframes; ++i) {
    PointCloudType::Ptr transformed_kf(new PointCloudType());
    pcl::transformPointCloud(*keyframe_clouds_[i], *transformed_kf,
                             keyframe_poses_[i]);
    *out_local_map += *transformed_kf;
  }

  // Optional: Downsample the local map if it's too large
  if (out_local_map->size() > 50000) {
    pcl::VoxelGrid<PointType> vg;
    vg.setLeafSize(0.5, 0.5, 0.5);
    vg.setInputCloud(out_local_map);
    vg.filter(*out_local_map);
  }
}

bool FrontEndICP::detectLoopClosure(const Eigen::Matrix4f &current_pose,
                                    double search_radius, int &out_loop_index,
                                    Eigen::Matrix4f &out_relative_pose,
                                    double &out_fitness_score) {
  if (keyframe_poses_.size() < 50)
    return false; // Need some history

  // Create a temporary cloud of poses to search using KDTree
  pcl::PointCloud<pcl::PointXYZ>::Ptr pose_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  for (const auto &pose : keyframe_poses_) {
    pose_cloud->push_back(pcl::PointXYZ(pose(0, 3), pose(1, 3), pose(2, 3)));
  }

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(pose_cloud);

  pcl::PointXYZ searchPoint;
  searchPoint.x = current_pose(0, 3);
  searchPoint.y = current_pose(1, 3);
  searchPoint.z = current_pose(2, 3);

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  // Search for historical poses within radius, but ignore recent ones (e.g.,
  // last 30 frames)
  if (kdtree.radiusSearch(searchPoint, search_radius, pointIdxRadiusSearch,
                          pointRadiusSquaredDistance) > 0) {
    for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
      int candidate_idx = pointIdxRadiusSearch[i];
      // Prevent false positives by ensuring a minimum travel distance (both in
      // index and time) Increasing from 30 to 100 keyframes to avoid matching
      // back to start too early
      if ((int)keyframe_poses_.size() - candidate_idx > 100) {
        // Found a valid loop closure candidate!
        out_loop_index = candidate_idx;
        return scanMatch(keyframe_clouds_.back(),
                         keyframe_clouds_[candidate_idx], out_relative_pose,
                         out_fitness_score);
      }
    }
  }
  return false;
}

} // namespace lidar_scan_match_c
