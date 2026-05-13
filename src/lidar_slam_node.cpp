#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <tf2_ros/transform_broadcaster.h>

#include "back_end_optimization.hpp"
#include "front_end_icp.hpp"
#include "sensor_preprocess.hpp"

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <algorithm>
#include <atomic>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <queue>
#include <thread>

using namespace lidar_scan_match_c;

class LidarSlamSystem {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  LidarSlamSystem(ros::NodeHandle &nh) : nh_(nh) {
    preprocess_ = std::make_shared<SensorPreprocess>(nh_);
    frontend_ = std::make_shared<FrontEndICP>(nh_);
    backend_ = std::make_shared<BackEndOptimization>(nh_);

    std::string lidar_topic, gps_topic, imu_topic;
    nh_.param<std::string>("lidar_topic", lidar_topic, "/ouster/points");
    nh_.param<std::string>("gps_topic", gps_topic, "/gps/fix");
    nh_.param<std::string>("imu_topic", imu_topic, "/imu/data");
    nh_.param<int>("lidar_rps", lidar_rps_, 10);
    nh_.param<int>("hd_map_match_interval", hd_map_match_interval_, 5);
    nh_.param<double>("gps_std_thres", gps_std_thres_, 0.3);
    nh_.param<double>("gps_cov_multiplier", gps_cov_multiplier_, 1.0);
    nh_.param<std::string>(
        "out_trajectory_path", out_trajectory_path_,
        "/root/catkin_ws/src/lidar_scan_match_c/output/lidar_trajectory.csv");
    nh_.param<bool>("out_save", out_save_, false);
    nh_.param<double>("out_hz", out_hz_, 10.0);

    // Loop Closure
    nh_.param<bool>("loop_closure_enabled", loop_closure_enabled_, true);
    nh_.param<double>("loop_closure_search_radius", loop_closure_search_radius_,
                      15.0);
    nh_.param<double>("loop_closure_fitness_score", loop_closure_fitness_score_,
                      0.3);

    last_reg_time_ = -1.0;
    last_record_time_ = -1.0;

    sub_lidar_ = nh_.subscribe(lidar_topic, 10000,
                               &LidarSlamSystem::lidarCallback, this);
    sub_gps_ =
        nh_.subscribe(gps_topic, 10000, &LidarSlamSystem::gpsCallback, this);
    sub_imu_ =
        nh_.subscribe(imu_topic, 10000, &LidarSlamSystem::imuCallback, this);

    pub_odom_ =
        nh_.advertise<nav_msgs::Odometry>("/lidar_scan_match_c/odometry", 10);
    pub_path_ = nh_.advertise<nav_msgs::Path>("/lidar_scan_match_c/path", 10);
    pub_current_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/lidar_scan_match_c/current_cloud", 1);
    pub_hd_map_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/lidar_scan_match_c/hd_map", 1, true);
    pub_local_map_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/lidar_scan_match_c/local_map", 1);
    pub_fitness_ = nh_.advertise<std_msgs::Float32>(
        "/lidar_scan_match_c/fitness_score", 1);
    pub_gps_path_ =
        nh_.advertise<nav_msgs::Path>("/lidar_scan_match_c/gps_path", 10);

    global_path_.header.frame_id = "map";
    gps_path_.header.frame_id = "map";
    latest_pose_ = Eigen::Matrix4f::Identity();

    process_thread_ = std::thread(&LidarSlamSystem::processLoop, this);
    map_loader_thread_ = std::thread(&LidarSlamSystem::mapLoaderLoop, this);
  }

  ~LidarSlamSystem() {
    if (out_save_) {
      saveTrajectory();
    }
    if (process_thread_.joinable())
      process_thread_.join();
    if (map_loader_thread_.joinable())
      map_loader_thread_.join();
  }

private:
  struct TrajectoryPoint {
    double time;
    double lat, lon, h;
    double twdx, twdy, twdz;
    double roll, pitch, yaw;
  };
  std::vector<TrajectoryPoint> recorded_trajectory_;

  void saveTrajectory() {
    std::string filename = out_trajectory_path_;
    nh_.getParam("out_trajectory_path", filename);
    std::ofstream f(filename);
    if (!f.is_open()) {
      ROS_ERROR("[Output] Cannot open %s for writing trajectory",
                filename.c_str());
      return;
    }
    f << "time,lat,lon,h,twd97x,twd97y,twd97z,roll_deg,pitch_deg,yaw_deg\n";
    f << std::fixed << std::setprecision(8);
    for (const auto &p : recorded_trajectory_) {
      f << p.time << "," << p.lat << "," << p.lon << "," << p.h << "," << p.twdx
        << "," << p.twdy << "," << p.twdz << "," << p.roll << "," << p.pitch
        << "," << p.yaw << "\n";
    }
    f.close();
    ROS_INFO("[Output] Successfully saved %zu trajectory points to %s",
             recorded_trajectory_.size(), filename.c_str());
  }

  void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(lidar_buf_mutex_);
    lidar_buf_.push(msg);
  }

  void gpsCallback(const sensor_msgs::NavSatFixConstPtr &msg) {
    if (msg->status.status < sensor_msgs::NavSatStatus::STATUS_FIX)
      return;
    std::lock_guard<std::mutex> lock(gps_buf_mutex_);
    gps_buf_.push(msg);
  }

  void imuCallback(const sensor_msgs::ImuConstPtr &msg) {
    std::lock_guard<std::mutex> lock(imu_buf_mutex_);
    imu_buf_.push(msg);
  }

  struct MapTileInfo {
    std::string filepath;
    double offset_x, offset_y, offset_z;
  };
  std::vector<MapTileInfo> available_map_tiles_;

  void mapLoaderLoop() {
    std::string hd_map_dir;
    if (!nh_.getParam("hd_map_directory", hd_map_dir) ||
        !nh_.getParam("map_tile_size", tile_size))
      return;

    ros::Rate rate(1);
    Eigen::Vector3f last_checked_pos(-999999.0f, -999999.0f, -999999.0f);

    while (ros::ok()) {
      if (!has_map_origin_) {
        rate.sleep();
        continue;
      }

      // Use direct access to pose (slight race condition is better than total
      // stall for local map reference)
      Eigen::Vector3f current_pos = latest_pose_.block<3, 1>(0, 3);

      if ((current_pos - last_checked_pos).norm() > 10.0) {
        last_checked_pos = current_pos;
        double p_x = current_pos.x() + map_origin_twd97_.x();
        double p_y = current_pos.y() + map_origin_twd97_.y();
        double p_z = current_pos.z() + map_origin_twd97_.z();

        PointCloudType::Ptr merged_tiles(new PointCloudType());
        if (available_map_tiles_.empty()) {
          for (const auto &entry :
               std::filesystem::directory_iterator(hd_map_dir)) {
            if (entry.path().extension() == ".pcd") {
              double ox, oy, oz;
              if (sscanf(entry.path().filename().string().c_str(),
                         "map_%lf_%lf_%lf.pcd", &ox, &oy, &oz) == 3) {
                available_map_tiles_.push_back(
                    {entry.path().string(), ox, oy, oz});
              }
            }
          }
          std::sort(available_map_tiles_.begin(), available_map_tiles_.end(),
                    [](const MapTileInfo &a, const MapTileInfo &b) {
                      return a.offset_x < b.offset_x;
                    });
        }

        for (const auto &tile_info : available_map_tiles_) {
          if (std::abs(tile_info.offset_x + tile_size / 2.0 - p_x) <=
                  tile_size * 1.5 &&
              std::abs(tile_info.offset_y + tile_size / 2.0 - p_y) <=
                  tile_size * 1.5 &&
              std::abs(tile_info.offset_z - p_z) <= 35.0) {
            PointCloudType::Ptr tile = loadPcdCloud(tile_info.filepath);
            if (tile && !tile->empty()) {
              Eigen::Affine3f T = Eigen::Affine3f::Identity();
              T.translation() << (tile_info.offset_x - map_origin_twd97_.x()),
                  (tile_info.offset_y - map_origin_twd97_.y()),
                  (tile_info.offset_z - map_origin_twd97_.z());
              pcl::transformPointCloud(*tile, *tile, T);
              *merged_tiles += *tile;
              ROS_INFO("map loaded.\n");
            }
          }
        }
        if (!merged_tiles->empty()) {
          // Optimized for real-time: Downsample the HD map to 0.5m density
          PointCloudType::Ptr optimized_map(new PointCloudType());
          pcl::VoxelGrid<PointType> vg;
          vg.setLeafSize(0.5f, 0.5f, 0.5f);
          vg.setInputCloud(merged_tiles);
          vg.filter(*optimized_map);

          pcl::KdTreeFLANN<PointType>::Ptr new_kdtree(
              new pcl::KdTreeFLANN<PointType>());
          new_kdtree->setInputCloud(optimized_map);

          {
            std::lock_guard<std::mutex> lock(hd_map_mutex_);
            current_global_map_ = optimized_map;
            hd_map_kdtree_ = new_kdtree;
          }

          frontend_->updateHDMapCloud(optimized_map);
          // Only publish once when the map successfully changes!
          publishHDMap();
        } else {
          bool need_clear = false;
          {
            std::lock_guard<std::mutex> lock(hd_map_mutex_);
            if (current_global_map_ && !current_global_map_->empty()) {
              current_global_map_->clear();
              hd_map_kdtree_.reset();
              need_clear = true;
            }
          }
          if (need_clear) {
            PointCloudType::Ptr empty_map(new PointCloudType());
            frontend_->updateHDMapCloud(empty_map);
            ROS_INFO("Car left HD map area. Map cleared.");
            publishHDMap();
          }
        }
      }
      // Increase sleep to prevent checking distance 50 times per second
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  }

  void processLoop() {
    ros::Rate rate(50);
    bool system_initialized = false;
    Eigen::Matrix4f current_pose = Eigen::Matrix4f::Identity();
    PointCloudType::Ptr local_map(new PointCloudType());

    while (ros::ok()) {
      sensor_msgs::PointCloud2ConstPtr lidar_msg = nullptr;
      size_t buf_size = 0;
      {
        std::lock_guard<std::mutex> lock(lidar_buf_mutex_);
        buf_size = lidar_buf_.size();
        if (!lidar_buf_.empty()) {
          lidar_msg = lidar_buf_.front();
          lidar_buf_.pop();
        }
      }

      if (!lidar_msg) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      if (buf_size > 2) {
        ROS_WARN_THROTTLE(
            1.0, "[Offline] Catching up... Lidar Buffer Size: %zu", buf_size);
      }
      auto t_loop_start = std::chrono::steady_clock::now();

      double lidar_time = lidar_msg->header.stamp.toSec();

      std::vector<sensor_msgs::ImuConstPtr> imu_msgs;
      {
        std::lock_guard<std::mutex> lock(imu_buf_mutex_);
        while (!imu_buf_.empty() &&
               imu_buf_.front()->header.stamp.toSec() <= lidar_time) {
          imu_msgs.push_back(imu_buf_.front());
          imu_buf_.pop();
        }
      }

      for (const auto &imu : imu_msgs) {
        if (!system_initialized) {
          Eigen::Quaternionf q(imu->orientation.w, imu->orientation.x,
                               imu->orientation.y, imu->orientation.z);
          if (q.norm() > 0.1) {
            double yaw_imu = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                                   1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
            // Align North-Up IMU (0) to ENU Y-axis (90)
            // double yaw_enu = yaw_imu + (M_PI / 2.0);
            double yaw_enu = -yaw_imu + (M_PI / 2.0);

            Eigen::Matrix4f T_world = Eigen::Matrix4f::Identity();
            T_world.block<3, 3>(0, 0) =
                Eigen::AngleAxisf(yaw_enu, Eigen::Vector3f::UnitZ())
                    .toRotationMatrix();
            current_pose =
                T_world * backend_->getExtrinsic().matrix().cast<float>();

            system_initialized = true;
            first_frame = true;
          }
        }
        double imu_time = imu->header.stamp.toSec();
        if (last_imu_time_internal_ > 0) {
          double dt = std::max(0.0001, imu_time - last_imu_time_internal_);
          ImuMeasurement m;
          m.linear_acceleration << imu->linear_acceleration.x,
              imu->linear_acceleration.y, imu->linear_acceleration.z;
          m.angular_velocity << imu->angular_velocity.x,
              imu->angular_velocity.y, imu->angular_velocity.z;
          backend_->integrateImuMeasurement(m, dt);
        }
        last_imu_time_internal_ = imu_time;
      }

      // CRITICAL FIX: If no IMU message has arrived yet, we CANNOT process the
      // Lidar frame. Doing so would initialize the graph origin to Identity (0
      // yaw), ignoring the true ENU heading!
      if (!system_initialized) {
        ROS_WARN_THROTTLE(1.0, "[Init] Waiting for internal IMU messages to "
                               "establish true world heading...");
        continue;
      }

      std::vector<sensor_msgs::NavSatFixConstPtr> gps_msgs;
      {
        std::lock_guard<std::mutex> lock(gps_buf_mutex_);
        while (!gps_buf_.empty() &&
               gps_buf_.front()->header.stamp.toSec() <= lidar_time) {
          gps_msgs.push_back(gps_buf_.front());
          gps_buf_.pop();
        }
      }
      for (const auto &gps : gps_msgs) {
        if (!gps_translator_internal_.initialized) {
          gps_translator_internal_.Reset(gps->latitude, gps->longitude,
                                         gps->altitude);
          gps_translator_internal_.GetTWD97(
              gps->latitude, gps->longitude, gps->altitude,
              map_origin_twd97_.x(), map_origin_twd97_.y(),
              map_origin_twd97_.z());
          has_map_origin_ = true;
        }
        if (gps_translator_internal_.initialized) {
          double tx, ty, tz;
          gps_translator_internal_.GetTWD97(gps->latitude, gps->longitude,
                                            gps->altitude, tx, ty, tz);
          GpsMeasurement gm;
          gm.timestamp = gps->header.stamp.toSec();
          gm.latitude = tx - map_origin_twd97_.x();
          gm.longitude = ty - map_origin_twd97_.y();
          gm.altitude = tz - map_origin_twd97_.z();

          // 讀取 GPS 雜訊 (Covariance)
          double cov_x =
              gps->position_covariance[0]; // 東向變異數 (Variance in East)
          double cov_y =
              gps->position_covariance[4]; // 北向變異數 (Variance in North)
          double cov_z =
              gps->position_covariance[8]; // 垂直變異數 (Variance in Up)
          double cov_h = std::sqrt(cov_x + cov_y) * gps_cov_multiplier_;

          // 判斷是否為有效且高精度的 RTK GPS (標準差 < 0.3m -> 變異數 <
          // 0.09)
          bool has_cov = (gps->position_covariance_type !=
                          sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN) &&
                         (cov_x > 0.0001);

          if (has_cov && cov_h < gps_std_thres_) {
            // 將高精度的變異數真實反映給 GTSAM，並乘上人工權重係數 (Covariance
            // 越大代表越不信任)
            gm.covariance_diag << cov_x * gps_cov_multiplier_,
                cov_y * gps_cov_multiplier_, cov_z * gps_cov_multiplier_;
            backend_->addGpsFactor(gm);
            ROS_INFO_THROTTLE(1.0,
                              "[GPS] RTK High-Precision GPS injected! "
                              "2DStdDev: %.2fm (Weight Multiplier: %.1f)",
                              cov_h, gps_cov_multiplier_);
          } else {
            // 如果是一般 GPS (誤差數公尺)，則不丟入優化器，避免撕裂 HD Map
            // 的精準軌跡
            gm.covariance_diag << 0.1, 0.1, 0.5;
          }

          // Visualization: Record and Publish GPS Path
          geometry_msgs::PoseStamped gps_pose;
          gps_pose.header.stamp = gps->header.stamp;
          gps_pose.header.frame_id = "map";
          gps_pose.pose.position.x = gm.latitude;
          gps_pose.pose.position.y = gm.longitude;
          gps_pose.pose.position.z = gm.altitude;
          gps_pose.pose.orientation.w = 1.0;
          gps_path_.header.stamp = gps->header.stamp;
          gps_path_.poses.push_back(gps_pose);
          pub_gps_path_.publish(gps_path_);
        }
      }

      PointCloudType::Ptr cloud(new PointCloudType());
      preprocess_->processCloud(lidar_msg, cloud);

      try {
        // Frequency control: Only perform Scan Matching at the specified rate
        bool run_matching =
            (last_reg_time_ < 0) ||
            (lidar_time - last_reg_time_ >= (0.95 / lidar_rps_));

        if (run_matching) {
          last_reg_time_ = lidar_time;
          // 0. Handle first frame initialization
          if (!frontend_->hasKeyframes()) {
            frontend_->addKeyframeCloud(cloud, current_pose);
            backend_->addOdomFactor(lidar_time, current_pose, false, false);
            backend_->optimize();
            publishData(lidar_time, current_pose, cloud, local_map);
            ROS_INFO("First Lidar frame initialized.");
            continue;
          }

          // 1. Prediction from Backend (IMU integration)
          if (!first_frame) {
            Eigen::Matrix4f predicted_pose =
                backend_->getPredictedPose().matrix().cast<float>();
            double predict_jump = (predicted_pose.block<3, 1>(0, 3) -
                                   current_pose.block<3, 1>(0, 3))
                                      .norm();
            if (predict_jump > 3.0) {
              ROS_ERROR_THROTTLE(1.0,
                                 "[FATAL] IMU Prediction Exploded! Jump: "
                                 "%.2fm. Failsafe activated.",
                                 predict_jump);
              // Fallback: Use the previous valid pose (constant position
              // fallback) to survive the frame without detonating ICP.
            } else {
              current_pose = predicted_pose;
            }
          } else {
            first_frame = false;
          }
          Eigen::Matrix4f local_pose = current_pose;
          double fitness;

          // --- Turn Detection ---
          bool is_turning = false;
          {
            static Eigen::Matrix4f prev_pose_for_turn =
                Eigen::Matrix4f::Identity();
            static bool first_turn_check = true;
            if (first_turn_check) {
              prev_pose_for_turn = current_pose;
              first_turn_check = false;
            }
            Eigen::Matrix3f R_curr = current_pose.block<3, 3>(0, 0);
            Eigen::Matrix3f R_prev = prev_pose_for_turn.block<3, 3>(0, 0);
            double turn_angle = std::abs(
                Eigen::AngleAxisf(R_curr * R_prev.transpose()).angle());

            // If turn_angle > 0.02 rad per frame (~11 deg/s at 10Hz), consider
            // it as turning
            ROS_INFO_THROTTLE(1.0, "[Turn Angle] %.2f.", turn_angle);
            if (turn_angle > 0.02) {
              is_turning = true;
              ROS_INFO_THROTTLE(1.0, "[Turn Detect] Turning detected! "
                                     "Increasing ICP search range.");
            }
            prev_pose_for_turn = current_pose;
          }

          bool scan_match_failed = false;
          // 2. Step A: Scan-to-LocalMap Matching
          frontend_->getLocalMap(local_map);
          if (frontend_->scanMatch(cloud, local_map, local_pose, fitness,
                                   is_turning)) {
            double jump =
                (local_pose.block<3, 1>(0, 3) - current_pose.block<3, 1>(0, 3))
                    .norm();
            if (jump < 3.0) {
              current_pose = local_pose;
            } else {
              ROS_WARN("[Match] Huge jump (%.2fm), ignoring scan-to-local "
                       "result. Relying on IMU prediction.",
                       jump);
              scan_match_failed = true;
            }
          } else {
            ROS_WARN("[Match] Scan-to-Local failed. Failsafe activated: "
                     "maintaining safe prediction.");
            scan_match_failed = true;
          }

          // 3. Step B: Scan-to-HDMap Refinement BEFORE adding Odom Factor
          Eigen::Matrix4f hd_pose = current_pose;
          double hd_fitness;
          static int hd_match_count = 0;
          bool hd_map_matched_this_frame = false;
          Eigen::Matrix4f delta_transform = Eigen::Matrix4f::Identity();

          bool has_hd_map = false;
          pcl::KdTreeFLANN<PointType>::Ptr local_kdtree;
          {
            std::lock_guard<std::mutex> lock(hd_map_mutex_);
            if (current_global_map_ != nullptr &&
                !current_global_map_->empty() && hd_map_kdtree_ != nullptr) {
              has_hd_map = true;
              local_kdtree = hd_map_kdtree_;
            }
          }

          if (has_hd_map) {
            PointType posPoint;
            posPoint.x = current_pose(0, 3);
            posPoint.y = current_pose(1, 3);
            posPoint.z = current_pose(2, 3);

            // Predict a point 15 meters directly ahead of the vehicle
            Eigen::Vector3f forward_dir =
                current_pose.block<3, 3>(0, 0) * Eigen::Vector3f::UnitX();
            Eigen::Vector3f lookahead_pos =
                current_pose.block<3, 1>(0, 3) + forward_dir * 15.0f;
            PointType lookaheadPoint;
            lookaheadPoint.x = lookahead_pos.x();
            lookaheadPoint.y = lookahead_pos.y();
            lookaheadPoint.z = lookahead_pos.z();

            std::vector<int> pointIdx(1);
            std::vector<float> pointDistSq(1);

            // 1. Check if vehicle is physically near the map
            if (local_kdtree->nearestKSearch(posPoint, 1, pointIdx,
                                             pointDistSq) > 0) {
              if (std::sqrt(pointDistSq[0]) >
                  8.0) { // Car is > 8m away from the closest map point
                has_hd_map = false;
              }
            }

            // 2. Check if the map continues ahead (Lookahead boundary
            // detection)
            if (has_hd_map &&
                local_kdtree->nearestKSearch(lookaheadPoint, 1, pointIdx,
                                             pointDistSq) > 0) {
              if (std::sqrt(pointDistSq[0]) >
                  15.0) { // No map points within 15m of the lookahead position
                has_hd_map = false;
              }
            }
          }

          if (has_hd_map) {
            if (++hd_match_count >= (lidar_rps_ / hd_map_match_interval_)) {
              hd_match_count = 0;
              if (frontend_->scanToHDMapMatch(cloud, hd_pose, hd_fitness,
                                              is_turning)) {
                if (hd_fitness < frontend_->getICPThreshold().max_fitness) {
                  // Compute the coordinate jump delta
                  double hd_jump = (hd_pose.block<3, 1>(0, 3) -
                                    current_pose.block<3, 1>(0, 3))
                                       .norm();
                  if (hd_jump > 3.0 * (lidar_rps_ / hd_map_match_interval_)) {
                    ROS_ERROR("[HD Map] Hallucinated match detected! Jump was "
                              "%.2fm. Discarding falsely good match.",
                              hd_jump);
                  } else {
                    hd_map_matched_this_frame = true;
                    has_hd_map_matched_ =
                        true; // Global localization established
                  }
                }
              }
            }
          } else {
            ROS_INFO_THROTTLE(
                2.0,
                "[Odometry Only] No HD Map loaded. Running Lidar Odometry.");
          }

          // --- ZUPT (Zero Velocity Update) Detection ---
          static Eigen::Matrix4f last_process_pose =
              Eigen::Matrix4f::Identity();
          static int static_frame_count = 0;
          if (first_frame) {
            last_process_pose = current_pose;
          }

          double frame_dist = (current_pose.block<3, 1>(0, 3) -
                               last_process_pose.block<3, 1>(0, 3))
                                  .norm();
          Eigen::Matrix3f R_curr_proc = current_pose.block<3, 3>(0, 0);
          Eigen::Matrix3f R_last_proc = last_process_pose.block<3, 3>(0, 0);
          double frame_angle = std::abs(
              Eigen::AngleAxisf(R_curr_proc * R_last_proc.transpose()).angle());

          // 如果一幀內位移小於 2公分 且 旋轉小於 0.005弧度 (約 0.28度)
          if (frame_dist < 0.02 && frame_angle < 0.005) {
            static_frame_count++;
          } else {
            static_frame_count = 0;
          }

          // 連續 3 幀 (0.3秒) 都符合靜止條件，則觸發 ZUPT
          if (static_frame_count >= 3) {
            current_pose = last_process_pose; // 凍結當前位姿，防止點雲抖動
            backend_->addZUPTFactor();        // 告訴後端優化器目前速度為 0
            ROS_INFO_THROTTLE(
                2.0,
                "[ZUPT] Vehicle is stationary. Applying Zero Velocity Update.");
          }
          last_process_pose = current_pose;
          // ---------------------------------------------

          // 4. Step A: Update Backend Odometry Factor
          // We pass hd_map_matched_this_frame to loosen the Odom Factor to
          // allow instant snapping, and scan_match_failed to prevent bad odom
          // from dragging
          backend_->addOdomFactor(lidar_time, current_pose,
                                  hd_map_matched_this_frame, scan_match_failed,
                                  is_turning);

          if (hd_map_matched_this_frame) {
            // Add absolute prior to the newly created graph node
            backend_->addHDMapFactor(hd_pose, hd_fitness);
          }

          // 5. Update Backend and Optimize EVERY Frame
          // DO NOT skip optimization. ISAM2 is very fast.
          // Skipping optimization causes getCurrentPose to return the LAST
          // optimized frame, snapping the Lidar point cloud backwards in time
          // and destroying final_publish.pcd!
          int opt_iters = hd_map_matched_this_frame ? 10 : 1;
          backend_->optimize(opt_iters);

          // 6. Add Keyframe
          static Eigen::Matrix4f last_keyframe_pose =
              Eigen::Matrix4f::Identity();
          double dist = (current_pose.block<3, 1>(0, 3) -
                         last_keyframe_pose.block<3, 1>(0, 3))
                            .norm();

          Eigen::Matrix3f R_curr = current_pose.block<3, 3>(0, 0);
          Eigen::Matrix3f R_last = last_keyframe_pose.block<3, 3>(0, 0);
          Eigen::AngleAxisf aa(R_curr * R_last.transpose());
          double angle = std::abs(aa.angle());

          if (dist > 0.3 || angle > 0.1) {
            int current_idx = frontend_->getKeyframeCount();
            frontend_->addKeyframeCloud(cloud, current_pose);
            last_keyframe_pose = current_pose;

            // 7. Optional Loop Closure Detection
            if (loop_closure_enabled_) {
              int loop_idx;
              Eigen::Matrix4f rel_pose;
              double loop_fitness;
              if (frontend_->detectLoopClosure(
                      current_pose, loop_closure_search_radius_, loop_idx,
                      rel_pose, loop_fitness)) {
                if (loop_fitness < loop_closure_fitness_score_) {
                  backend_->addLoopFactor(loop_idx, rel_pose, loop_fitness);
                  // Trigger an extra optimization for the loop
                  backend_->optimize(10);
                }
              }
            }
          }

          // Fetch the smoothed pose from ISAM2 for this newly added frame
          gtsam::Pose3 optimized_pose = backend_->getCurrentPose();

          // CRITICAL FIX: Tightly couple the frontend and backend!
          // We shift the ENTIRE frontend coordinate system to align with the
          // optimized pose every frame. This elegantly handles GPS corrections,
          // Loop Closures, and HD Map snaps automatically without needing
          // duplicated logic.
          Eigen::Matrix4f opt_pose_mat = optimized_pose.matrix().cast<float>();
          delta_transform = opt_pose_mat * current_pose.inverse();

          // 1. Shift the frontend map history so the next ICP matches in the
          // correct global frame
          frontend_->shiftLocalMap(delta_transform);

          // 2. Update the current pose
          current_pose = opt_pose_mat;

          // 3. Shift the last keyframe pose tracker to prevent instant false
          // keyframe drops
          last_keyframe_pose = delta_transform * last_keyframe_pose;

          // 4. Update the backend's internal baseline (prev_lidar_pose_) so
          // the next relative_odom is correct
          backend_->setCurrentPose(optimized_pose);
        } else {
          // For skipped frames, perform safely bounded prediction from IMU
          Eigen::Matrix4f predicted_pose =
              backend_->getPredictedPose().matrix().cast<float>();
          double predict_jump = (predicted_pose.block<3, 1>(0, 3) -
                                 current_pose.block<3, 1>(0, 3))
                                    .norm();
          if (predict_jump < 3.0) {
            current_pose = predicted_pose;
          }
          ROS_INFO_THROTTLE(1.0, "[INFO] Skip Scan Matching.");
        }

        // 7. Final Visualization (Always publish at lidar rate)
        publishData(lidar_time, current_pose, cloud, local_map);
        latest_pose_ = current_pose;

        /*
        // === DEBUG: 永遠存下 光達對地圖(HD Map) 的匹配結果 ===
        {
          PointCloudType::Ptr matched_pcl_debug(new PointCloudType());
          pcl::transformPointCloud(*cloud, *matched_pcl_debug, current_pose);
          pcl::io::savePCDFileBinary("/root/catkin_ws/src/lidar_scan_match_c/"
                                     "final_publish.pcd",
                                     *matched_pcl_debug);
        }
        // -----------------------------------
        */

        auto t_loop_end = std::chrono::steady_clock::now();
        double total_ms =
            std::chrono::duration<double, std::milli>(t_loop_end - t_loop_start)
                .count();
        ROS_INFO_THROTTLE(2.0,
                          "[Realtime] Total Latency: %.1f ms (Buffer: %zu)",
                          total_ms, buf_size);
      } catch (std::exception &e) {
        ROS_ERROR("Exception: %s", e.what());
      }
    }
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_lidar_, sub_gps_, sub_imu_;
  ros::Publisher pub_odom_, pub_path_, pub_current_cloud_, pub_hd_map_,
      pub_local_map_, pub_fitness_, pub_gps_path_;
  nav_msgs::Path global_path_, gps_path_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  std::shared_ptr<SensorPreprocess> preprocess_;
  std::shared_ptr<FrontEndICP> frontend_;
  std::shared_ptr<BackEndOptimization> backend_;
  std::mutex lidar_buf_mutex_, gps_buf_mutex_, imu_buf_mutex_, hd_map_mutex_;
  std::queue<sensor_msgs::PointCloud2ConstPtr> lidar_buf_;
  std::queue<sensor_msgs::NavSatFixConstPtr> gps_buf_;
  std::queue<sensor_msgs::ImuConstPtr> imu_buf_;
  Eigen::Matrix4f latest_pose_;
  std::thread process_thread_, map_loader_thread_;
  bool first_frame{false};
  Eigen::Vector3d map_origin_twd97_{0, 0, 0};
  std::atomic<bool> has_map_origin_{false};
  std::atomic<bool> has_hd_map_matched_{false};
  LocalCartesian gps_translator_internal_;
  double last_reg_time_;
  int lidar_rps_;
  int hd_map_match_interval_;
  double gps_std_thres_;
  double gps_cov_multiplier_;
  std::string out_trajectory_path_;
  bool out_save_;
  double out_hz_;
  double tile_size;

  // Loop Closure Parameters
  bool loop_closure_enabled_;

  pcl::KdTreeFLANN<PointType>::Ptr hd_map_kdtree_;
  double loop_closure_search_radius_;
  double loop_closure_fitness_score_;

  PointCloudType::Ptr current_global_map_;
  double last_imu_time_internal_ = -1.0;
  double last_record_time_ = -1.0;

  PointCloudType::Ptr loadPcdCloud(const std::string &path) {
    PointCloudType::Ptr cloud(new PointCloudType);
    if (pcl::io::loadPCDFile<PointType>(path, *cloud) == -1) {
      ROS_ERROR_THROTTLE(5.0, "Couldn't read PCD file: %s", path.c_str());
    }
    return cloud;
  }

  PointCloudType::Ptr loadBinCloud(const std::string &path) {
    PointCloudType::Ptr cloud(new PointCloudType);
    std::ifstream f(path, std::ios::binary);
    if (!f)
      return cloud;
    f.seekg(0, std::ios::end);
    size_t size = f.tellg();
    f.seekg(0, std::ios::beg);
    size_t n = size / (4 * sizeof(float));
    cloud->resize(n);
    std::vector<float> v(4 * n);
    f.read((char *)v.data(), size);
    for (size_t i = 0; i < n; ++i) {
      cloud->points[i].x = v[4 * i];
      cloud->points[i].y = v[4 * i + 1];
      cloud->points[i].z = v[4 * i + 2];
      cloud->points[i].intensity = v[4 * i + 3];
    }
    return cloud;
  }

  void publishHDMap() {
    sensor_msgs::PointCloud2 msg;
    if (!current_global_map_ || current_global_map_->empty()) {
      // 為了強迫 RViz 清除畫面上舊的地圖，發佈一個藏在地底下的隱形假點
      PointCloudType dummy_map;
      PointType p;
      p.x = 0;
      p.y = 0;
      p.z = -10000.0; // 藏在地底一萬公尺
      dummy_map.push_back(p);
      pcl::toROSMsg(dummy_map, msg);
    } else {
      pcl::toROSMsg(*current_global_map_, msg);
    }
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    pub_hd_map_.publish(msg);
  }

  void publishData(double ts, const Eigen::Matrix4f &pose,
                   const PointCloudType::Ptr &cloud,
                   const PointCloudType::Ptr &local_map) {
    ros::Time rts(ts);

    // 1. Publish Path & Pose
    geometry_msgs::PoseStamped ps;
    ps.header.stamp = rts;
    ps.header.frame_id = "map";
    ps.pose.position.x = pose(0, 3);
    ps.pose.position.y = pose(1, 3);
    ps.pose.position.z = pose(2, 3);
    Eigen::Quaternionf q(pose.block<3, 3>(0, 0));
    ps.pose.orientation.w = q.w();
    ps.pose.orientation.x = q.x();
    ps.pose.orientation.y = q.y();
    ps.pose.orientation.z = q.z();

    global_path_.header.stamp = rts;
    global_path_.poses.push_back(ps);
    pub_path_.publish(global_path_);

    // 2. Broadcast TF (map -> lidar)
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = rts;
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "lidar";
    tf_msg.transform.translation.x = ps.pose.position.x;
    tf_msg.transform.translation.y = ps.pose.position.y;
    tf_msg.transform.translation.z = ps.pose.position.z;
    tf_msg.transform.rotation.w = q.w();
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_broadcaster_.sendTransform(tf_msg);

    // 3. Publish Matched Cloud in Map Frame
    PointCloudType::Ptr world_cloud(new PointCloudType());
    pcl::transformPointCloud(*cloud, *world_cloud, pose);
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*world_cloud, cloud_msg);
    cloud_msg.header.stamp = rts;
    cloud_msg.header.frame_id = "map";
    pub_current_cloud_.publish(cloud_msg);

    // 4. Publish Odometry
    nav_msgs::Odometry odom;
    odom.header.stamp = rts;
    odom.header.frame_id = "map";
    odom.child_frame_id = "lidar";
    odom.pose.pose = ps.pose;
    pub_odom_.publish(odom);

    // 4.5 Publish Local Map
    if (local_map && !local_map->empty()) {
      sensor_msgs::PointCloud2 local_map_msg;
      pcl::toROSMsg(*local_map, local_map_msg);
      local_map_msg.header.stamp = rts;
      local_map_msg.header.frame_id = "map";
      pub_local_map_.publish(local_map_msg);
    }

    // 5. Record Trajectory Point at specified out_hz_
    if (has_map_origin_) {
      if (last_record_time_ < 0 || ts - last_record_time_ >= (0.95 / out_hz_)) {
        last_record_time_ = ts;
        TrajectoryPoint tp;

        Eigen::Matrix4f T_imu_lidar =
            backend_->getExtrinsic().matrix().cast<float>();
        Eigen::Matrix4f T_map_imu = pose * T_imu_lidar.inverse();
        Eigen::Quaternionf q_veh(T_map_imu.block<3, 3>(0, 0));

        tp.time = ts;
        tp.twdx = T_map_imu(0, 3) + map_origin_twd97_.x();
        tp.twdy = T_map_imu(1, 3) + map_origin_twd97_.y();
        tp.twdz = T_map_imu(2, 3) + map_origin_twd97_.z();

        // Convert Map Frame (ENU) to Lat/Lon/H
        gps_translator_internal_.GetWGS84(tp.twdx, tp.twdy, tp.twdz, tp.lat,
                                          tp.lon, tp.h);

        // Convert Vehicle Quaternion (q_veh) to Roll, Pitch, Yaw in Degrees
        double sinr_cosp = 2 * (q_veh.w() * q_veh.x() + q_veh.y() * q_veh.z());
        double cosr_cosp =
            1 - 2 * (q_veh.x() * q_veh.x() + q_veh.y() * q_veh.y());
        tp.roll = std::atan2(sinr_cosp, cosr_cosp) * 180.0 / M_PI;

        double sinp = 2 * (q_veh.w() * q_veh.y() - q_veh.z() * q_veh.x());
        if (std::abs(sinp) >= 1)
          tp.pitch = std::copysign(M_PI / 2, sinp) * 180.0 / M_PI;
        else
          tp.pitch = std::asin(sinp) * 180.0 / M_PI;

        // Calculate Heading (Yaw) using the VEHICLE'S forward vector
        Eigen::Vector3f veh_forward_in_map =
            T_map_imu.block<3, 3>(0, 0) * Eigen::Vector3f::UnitX();
        tp.yaw = std::atan2(veh_forward_in_map.x(), veh_forward_in_map.y()) *
                 180.0 / M_PI;

        recorded_trajectory_.push_back(tp);
      }
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_scan_match_c_node");
  ros::NodeHandle nh("~");
  LidarSlamSystem slam(nh);
  ros::spin();
  return 0;
}
