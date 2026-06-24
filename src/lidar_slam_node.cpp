#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include "lidar_scan_match_c/back_end_optimization.hpp"
#include "lidar_scan_match_c/front_end_icp.hpp"
#include "lidar_scan_match_c/sensor_preprocess.hpp"

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
    int hd_map_match_freq;
    nh_.param<int>("hd_map_match_interval", hd_map_match_freq, 5);
    // 計算跳幀間隔：例如 10Hz / 5Hz = 每 2 幀匹配一次
    hd_map_match_interval_ = std::max(1, lidar_rps_ / hd_map_match_freq);
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

    nh_.param<double>("max_single_frame_translation", max_trans_jump_, 1.5);
    nh_.param<double>("max_single_frame_rotation", max_rot_jump_, 0.5);

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
    pub_range_rings_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/lidar_scan_match_c/range_rings", 1);

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
    f << "time,lat,lon,h,twd97x,twd97y,twd97z,roll_rad,pitch_rad,yaw_rad\n";
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

    if (!gps_translator_internal_.initialized) {
      gps_translator_internal_.Reset(msg->latitude, msg->longitude,
                                     msg->altitude);
      gps_translator_internal_.GetTWD97(
          msg->latitude, msg->longitude, msg->altitude, map_origin_twd97_.x(),
          map_origin_twd97_.y(), map_origin_twd97_.z());
      has_map_origin_ = true;
      ROS_INFO("[Init] GPS Map origin established: Lat: %.6f, Lon: %.6f",
               msg->latitude, msg->longitude);
    }

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
  std::atomic<bool> first_map_check_done_{false};

  void mapLoaderLoop() {
    std::string hd_map_dir;
    if (!nh_.getParam("hd_map_directory", hd_map_dir) ||
        !nh_.getParam("map_tile_size", tile_size)) {
      first_map_check_done_ = true;
      return;
    }

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

        CloudType::Ptr merged_tiles(new CloudType());
        if (available_map_tiles_.empty()) {
          try {
            if (std::filesystem::exists(hd_map_dir) &&
                std::filesystem::is_directory(hd_map_dir)) {
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
              std::sort(available_map_tiles_.begin(),
                        available_map_tiles_.end(),
                        [](const MapTileInfo &a, const MapTileInfo &b) {
                          return a.offset_x < b.offset_x;
                        });
            } else {
              ROS_WARN("[MapLoader] HD map directory does not exist: %s",
                       hd_map_dir.c_str());
            }
          } catch (const std::exception &e) {
            ROS_ERROR("[MapLoader] Error reading map directory: %s", e.what());
          }
        }

        for (const auto &tile_info : available_map_tiles_) {
          if (std::abs(tile_info.offset_x + tile_size / 2.0 - p_x) <=
                  tile_size * 1.5 &&
              std::abs(tile_info.offset_y + tile_size / 2.0 - p_y) <=
                  tile_size * 1.5 &&
              std::abs(tile_info.offset_z - p_z) <= 35.0) {
            CloudType::Ptr tile = loadPcdCloud(tile_info.filepath);
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

        first_map_check_done_ = true;
        last_checked_pos = current_pos;

        if (!merged_tiles->empty()) {
          // Optimized for real-time: Downsample the HD map to 0.5m density
          CloudType::Ptr optimized_map(new CloudType());
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
            CloudType::Ptr empty_map(new CloudType());
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
    CloudType::Ptr local_map(new CloudType());

    while (ros::ok()) {
      sensor_msgs::PointCloud2ConstPtr lidar_msg = nullptr;
      size_t buf_size = 0;
      bool wait_for_lidar = false;
      bool wait_for_map = false;

      {
        std::lock_guard<std::mutex> lock(lidar_buf_mutex_);
        if (lidar_buf_.empty()) {
          wait_for_lidar = true;
        } else if (!first_map_check_done_) {
          wait_for_map = true;
        } else {
          buf_size = lidar_buf_.size();
          lidar_msg = lidar_buf_.front();
          lidar_buf_.pop();
        }
      }

      if (wait_for_lidar) {
        rate.sleep();
        continue;
      }
      if (wait_for_map) {
        ROS_INFO_THROTTLE(1.0, "[Init] Waiting for initial HD Map check...");
        rate.sleep();
        continue;
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

      static bool currently_within_hd_bounds = false;
      static int consecutive_hd_map_failures = 0;
      static double current_gps_speed = 0.0; // Global to processLoop for ZUPT
      static double last_gps_time_for_speed = -1.0;
      static Eigen::Vector2d last_gps_pos_2d(0, 0);
      static double gps_displacement =
          0.0; // Added for precise ZUPT displacement tracking
      static int static_frame_count = 0; // Global to processLoop for ZUPT

      for (const auto &gps : gps_msgs) {
        if (gps_translator_internal_.initialized) {
          double tx, ty, tz;
          gps_translator_internal_.GetTWD97(gps->latitude, gps->longitude,
                                            gps->altitude, tx, ty, tz);
          GpsMeasurement gm;
          gm.timestamp = gps->header.stamp.toSec();
          gm.latitude = tx - map_origin_twd97_.x();
          gm.longitude = ty - map_origin_twd97_.y();
          gm.altitude = tz - map_origin_twd97_.z();

          Eigen::Vector2d curr_gps_pos_2d(gm.latitude, gm.longitude);
          if (last_gps_time_for_speed > 0) {
            double dt = gm.timestamp - last_gps_time_for_speed;
            if (dt > 0.05) { // 避免時間差過小導致除以零或雜訊放大
              current_gps_speed =
                  (curr_gps_pos_2d - last_gps_pos_2d).norm() / dt;
              gps_displacement = (curr_gps_pos_2d - last_gps_pos_2d).norm();
            }
          }
          last_gps_pos_2d = curr_gps_pos_2d;
          last_gps_time_for_speed = gm.timestamp;

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
            if (!currently_within_hd_bounds || !has_hd_map_matched_ ||
                consecutive_hd_map_failures >= 3) {
              gm.covariance_diag << cov_x * gps_cov_multiplier_,
                  cov_y * gps_cov_multiplier_, cov_z * gps_cov_multiplier_;
              backend_->addGpsFactor(gm);

              if (currently_within_hd_bounds &&
                  consecutive_hd_map_failures >= 3) {
                ROS_WARN_THROTTLE(1.0, "[GPS] HD Map failed >= 3 times! "
                                       "Fallback to RTK GNSS injected!");
              } else if (!has_hd_map_matched_) {
                ROS_INFO_THROTTLE(2.0,
                                  "[GPS] Pre-Match: Trusting RTK GPS tightly "
                                  "for initialization! "
                                  "2DStdDev: %.2fm",
                                  cov_h);
              } else {
                ROS_INFO_THROTTLE(1.0,
                                  "[GPS] RTK High-Precision GPS injected! "
                                  "2DStdDev: %.2fm (Weight Multiplier: %.1f)",
                                  cov_h, gps_cov_multiplier_);
              }
            } else {
              // IN HD MAP: Inject GNSS with a massive covariance (10.0m std dev
              // -> 100.0 variance). This is too weak to cause lateral wobbling
              // fighting the HD map, but it acts as a final longitudinal anchor
              // to prevent infinite corridor sliding!
              double loose_var = 100.0;
              gm.covariance_diag << loose_var, loose_var, loose_var;
              backend_->addGpsFactor(gm);
            }
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

      CloudType::Ptr cloud(new CloudType());
      std::vector<double> timestamps;
      preprocess_->processCloud(lidar_msg, cloud, timestamps);

      try {
        // Frequency control: Only perform Scan Matching at the specified rate
        bool run_matching =
            (last_reg_time_ < 0) ||
            (lidar_time - last_reg_time_ >= (0.95 / lidar_rps_));

        double prev_reg_time_ = last_reg_time_;
        if (run_matching) {
          last_reg_time_ = lidar_time;

          bool is_first_lidar_frame = first_frame;

          // 1. Prediction from Backend (IMU integration)
          Eigen::Matrix4f predicted_pose = current_pose;
          double predict_jump = 0.0;
          double predict_angle = 0.0;

          if (!is_first_lidar_frame) {
            if (static_frame_count >= 3) {
              // 如果車輛處於 ZUPT 靜止鎖定狀態，我們強制預測位姿為當前位姿，
              // 徹底阻斷任何由 IMU 偏差積分產生的前後滑動與預測漂移！
              predicted_pose = current_pose;
              predict_jump = 0.0;
              predict_angle = 0.0;
            } else {
              predicted_pose =
                  backend_->getPredictedPose().matrix().cast<float>();
              predict_jump = (predicted_pose.block<3, 1>(0, 3) -
                              current_pose.block<3, 1>(0, 3))
                                 .norm();
              Eigen::Matrix3f R_curr = current_pose.block<3, 3>(0, 0);
              Eigen::Matrix3f R_pred = predicted_pose.block<3, 3>(0, 0);
              predict_angle = std::abs(
                  Eigen::AngleAxisf(R_pred * R_curr.transpose()).angle());
            }
          } else {
            first_frame = false;
            ROS_INFO("[Init] Processing first Lidar frame...");
          }

          // ==== MOTION COMPENSATION ====
          if (!is_first_lidar_frame && !timestamps.empty() &&
              prev_reg_time_ > 0) {
            auto t_mc_start = std::chrono::high_resolution_clock::now();

            auto poseToVector = [](double time, const Eigen::Matrix4f &pose) {
              Eigen::VectorXd vec(7);
              vec(0) = time;
              vec(1) = pose(0, 3);
              vec(2) = pose(1, 3);
              vec(3) = pose(2, 3);
              Eigen::Matrix3f R = pose.block<3, 3>(0, 0);
              Eigen::Vector3f euler =
                  R.eulerAngles(2, 1, 0);       // Yaw, Pitch, Roll
              vec(4) = euler[2] * 180.0 / M_PI; // Roll
              vec(5) = euler[1] * 180.0 / M_PI; // Pitch
              vec(6) = euler[0] * 180.0 / M_PI; // Yaw
              return vec;
            };

            Eigen::VectorXd posPrev =
                poseToVector(prev_reg_time_, latest_pose_);
            Eigen::VectorXd posCurr = poseToVector(lidar_time, predicted_pose);

            lidar_utils::CloudUtils::motionCompensateAndDG(
                cloud, timestamps, posPrev, posCurr, true);

            // Revert the cloud from global frame back to lidar frame, as
            // scanToHDmap will input the init_guess pose itself
            pcl::transformPointCloud(*cloud, *cloud, predicted_pose.inverse());

            auto t_mc_end = std::chrono::high_resolution_clock::now();
            ROS_INFO_THROTTLE(
                1.0, "[BackEnd] Time - Motion Compensation: %.2f ms",
                std::chrono::duration<double, std::milli>(t_mc_end - t_mc_start)
                    .count());
          }

          bool scan_match_failed = false;
          bool hd_map_matched_this_frame = false;
          bool is_turning = false;
          double fitness = 1e6;
          double hd_fitness = 1e6;
          Eigen::Matrix4f hd_pose = current_pose;
          Eigen::Matrix4f delta_transform = Eigen::Matrix4f::Identity();
          bool should_skip_frame = false;
          currently_within_hd_bounds = false;
          bool local_match_success = false;
          bool use_temp_local_map = false;

          // --- Update current_pose with prediction ---
          if (predict_jump < max_trans_jump_ && predict_angle < max_rot_jump_) {
            current_pose = predicted_pose;
          } else if (!is_first_lidar_frame) {
            ROS_WARN_THROTTLE(
                2.0,
                "[Prediction] Rejected IMU prediction due to large jump "
                "(trans: %.2fm/%.2fm, rot: %.2frad/%.2frad)",
                predict_jump, max_trans_jump_, predict_angle, max_rot_jump_);
          }

          Eigen::Matrix4f local_pose = current_pose;
          // --- Turn Detection (YAW ONLY & Filtered to prevent false triggers)
          // ---
          {
            static Eigen::Matrix4f prev_pose_for_turn =
                Eigen::Matrix4f::Identity();
            static bool first_turn_check = true;
            static int turn_frame_count = 0;

            if (first_turn_check) {
              prev_pose_for_turn = current_pose;
              first_turn_check = false;
            }

            Eigen::Vector3f fwd_prev =
                prev_pose_for_turn.block<3, 3>(0, 0) * Eigen::Vector3f::UnitX();
            Eigen::Vector3f fwd_curr =
                current_pose.block<3, 3>(0, 0) * Eigen::Vector3f::UnitX();
            fwd_prev.z() = 0;
            fwd_curr.z() = 0; // Project to horizontal plane

            if (fwd_prev.norm() > 1e-6 && fwd_curr.norm() > 1e-6) {
              fwd_prev.normalize();
              fwd_curr.normalize();
              double turn_angle = std::acos(
                  std::max(-1.0f, std::min(1.0f, fwd_prev.dot(fwd_curr))));

              if (turn_angle > 0.015) { // 0.015 rad = ~0.85 deg/frame
                turn_frame_count++;
              } else {
                turn_frame_count = 0;
              }
            }

            if (turn_frame_count >=
                2) { // Require 2 consecutive frames to trigger
              is_turning = true;
            }
            prev_pose_for_turn = current_pose;
          }

          // --- Extended Local Map tracking (Keep for 20m after turn) ---
          static bool was_turning_extended = false;
          static Eigen::Vector3f turn_end_pos = Eigen::Vector3f::Zero();
          use_temp_local_map = is_turning;

          if (is_turning) {
            was_turning_extended = true;
            turn_end_pos = current_pose.block<3, 1>(0, 3);
          } else if (was_turning_extended) {
            double dist_since_turn =
                (current_pose.block<3, 1>(0, 3) - turn_end_pos).norm();
            if (dist_since_turn < 20.0) {
              use_temp_local_map = true;
            } else {
              was_turning_extended = false;
              ROS_INFO("[Local Map] 20 meters reached since turn ended. Ready "
                       "to clear temporary local map.");
            }
          }

          // 2. 判斷是否在 HD Map 範圍內 (Overlap Check)
          pcl::KdTreeFLANN<PointType>::Ptr local_kdtree;
          {
            std::lock_guard<std::mutex> lock(hd_map_mutex_);
            if (current_global_map_ && !current_global_map_->empty() &&
                hd_map_kdtree_) {
              local_kdtree = hd_map_kdtree_;
              PointType posPoint;
              posPoint.x = current_pose(0, 3);
              posPoint.y = current_pose(1, 3);
              posPoint.z = current_pose(2, 3);
              std::vector<int> pIdx(1);
              std::vector<float> pDistSq(1);
              if (local_kdtree->nearestKSearch(posPoint, 1, pIdx, pDistSq) >
                  0) {
                // "和hdmap重合的部份小於一定距離或範圍" - 超過 20m 視為沒有
                // HD Map (Increased to 30.0m to handle maps without ground
                // points)
                if (std::sqrt(pDistSq[0]) < 30.0) {
                  currently_within_hd_bounds = true;
                }
              }
            }
          }

          static bool was_in_hd_map = false;

          if (currently_within_hd_bounds) {
            // --- 狀態 A：有 HD Map ---
            // 預設進行 Scan-to-HDMap (信賴，糾正 GPS/IMU)
            static int hd_match_count = 0;
            bool periodic_update = (++hd_match_count >= hd_map_match_interval_);

            // 確保在車輛即將靜止的前一兩幀，強制執行高精度的 HD Map 匹配，
            // 這樣可以保證 ZUPT 凍結時，車輛是完美停在正確的 HD Map
            // 位置上，而不是停在漂移的 IMU 預測位置。
            if (static_frame_count > 0 && static_frame_count < 3) {
              periodic_update = true;
            }

            if (!was_in_hd_map) {
              periodic_update = true;
              consecutive_hd_map_failures = 0;
            }
            was_in_hd_map = true;

            // == 1. 局部匹配與建圖 (Local Map) ==
            // 使用者邏輯 3: "只有在轉彎以及過轉彎後20公尺
            // 會依據點雲對地圖匹配好的結果 來製作LOCALMAP ，並且會進行SCANMATCH
            // (LO)"
            if (use_temp_local_map) {
              if (has_last_keyframe_ && !local_map->empty()) {
                if (frontend_->scanMatch(cloud, local_map, local_pose, fitness,
                                         true)) {
                  double jump = (local_pose.block<3, 1>(0, 3) -
                                 current_pose.block<3, 1>(0, 3))
                                    .norm();
                  if (jump < 1.0) {
                    current_pose =
                        local_pose; // 更新目前姿態為局部匹配結果，提供更準確的初始猜測給
                                    // HD Map
                    local_match_success = true;
                  }
                }
              } else {
                // 初始化 Local Map
                frontend_->clearLocalMap();
                local_map->clear();
                local_match_success = true;
                has_last_keyframe_ = true;
                last_keyframe_pose_ = current_pose;
              }
            } else {
              // 非轉彎/緩衝期，嚴格清空 Local Map (不該點雲匹配的時候絕不出現
              // localmap)
              if (has_last_keyframe_) {
                frontend_->clearLocalMap();
                local_map->clear();
                has_last_keyframe_ = false;
              }
            }

            // == 2. 全域匹配 (HD Map) ==
            if (periodic_update) {
              hd_match_count = 0;
              Eigen::Matrix4f rescue_pose =
                  current_pose; // 此為經過 Local Match (LO) 優化過的姿態
              if (frontend_->scanToHDMapMatch(cloud, rescue_pose, hd_fitness,
                                              is_turning)) {
                ROS_INFO_THROTTLE(1.0, "[DEBUG] hd_fitness: %.2f.", hd_fitness);

                // 放寬轉彎時的接受標準，因為運動模糊可能會讓 fitness 稍微增加
                double threshold = frontend_->getICPThreshold().max_fitness;
                if (is_turning)
                  threshold *= 5.0;

                Eigen::Matrix3f R_pred = predicted_pose.block<3, 3>(0, 0);
                Eigen::Vector3f delta_t_body =
                    R_pred.transpose() * (rescue_pose.block<3, 1>(0, 3) -
                                          predicted_pose.block<3, 1>(0, 3));
                double lateral_jump = std::abs(delta_t_body.y());

                if (hd_fitness < threshold && lateral_jump < 0.3) {
                  current_pose = rescue_pose; // 以 HD Map 全域結果為最終姿態
                  hd_map_matched_this_frame = true;
                  hd_pose = rescue_pose;
                  has_hd_map_matched_ = true;
                  consecutive_hd_map_failures = 0;
                } else {
                  consecutive_hd_map_failures++;
                  if (hd_fitness < threshold && lateral_jump >= 0.3) {
                    ROS_WARN_THROTTLE(1.0,
                                      "[HD Map] Match rejected due to large "
                                      "lateral jump: %.2fm >= 0.3m",
                                      lateral_jump);
                  }
                }
              } else {
                consecutive_hd_map_failures++;
              }
            }

            should_skip_frame =
                !hd_map_matched_this_frame && !local_match_success;

          } else {
            // --- 狀態 B：沒有 HD Map ---
            // "當沒有hdmap的時候 才會誘發scanmatch"
            was_in_hd_map = false;

            if (has_last_keyframe_ && !local_map->empty()) {
              if (frontend_->scanMatch(cloud, local_map, local_pose, fitness,
                                       is_turning)) {
                double jump = (local_pose.block<3, 1>(0, 3) -
                               current_pose.block<3, 1>(0, 3))
                                  .norm();
                double local_threshold =
                    frontend_->getICPThreshold().max_fitness * 0.5;
                if (is_turning) {
                  local_threshold *= 1.5; // 轉彎時放寬至 1.5 倍
                }
                if (jump < 1.0) {
                  current_pose =
                      local_pose; // 永遠採信 Lidar Odometry 避免 IMU 預測失控
                  if (fitness < local_threshold) {
                    local_match_success = true;
                  } else {
                    ROS_WARN_THROTTLE(
                        1.0,
                        "[Local Map] High fitness %.2f > %.2f. Pose updated, "
                        "but cloud NOT added to map.",
                        fitness, local_threshold);
                  }
                } else {
                  ROS_WARN_THROTTLE(
                      1.0, "[Local Map] Rejected match: jump=%.2fm > 1.0m",
                      jump);
                }
              }
            } else {
              // 剛離開 HD Map (或一開始就沒有)，強制信任 GPS/IMU 預測並做為
              // Local Map 的起點
              frontend_
                  ->clearLocalMap(); // 重要：清空上一個無地圖區域留下的舊點雲
              local_map->clear();    // 同步清空當前幀使用的 local_map 變數
              local_match_success = true;
              has_last_keyframe_ = true;
              last_keyframe_pose_ = current_pose;
              ROS_INFO("[Local Map] Creating new local map anchor outside HD "
                       "Map.");
            }

            if (!local_match_success) {
              should_skip_frame = true;
              ROS_WARN_THROTTLE(
                  1.0, "[Failsafe] Local match failed. Using IMU prediction.");
            } else {
              should_skip_frame = false;
            }
          }

          // --- ZUPT (Zero Velocity Update) Detection ---
          static Eigen::Matrix4f last_process_pose =
              Eigen::Matrix4f::Identity();
          static Eigen::Matrix4f last_hd_pose = Eigen::Matrix4f::Identity();
          static bool has_last_hd_pose = false;

          if (is_first_lidar_frame) {
            last_process_pose = current_pose;
          }

          // 1. 計算 GICP 匹配位姿的前後位置差 (若本幀完成 HD Map 匹配)
          double match_dist = 999.0;
          double match_angle = 999.0;
          if (hd_map_matched_this_frame) {
            if (has_last_hd_pose) {
              match_dist =
                  (hd_pose.block<3, 1>(0, 3) - last_hd_pose.block<3, 1>(0, 3))
                      .norm();
              match_angle = std::abs(
                  Eigen::AngleAxisf(hd_pose.block<3, 3>(0, 0) *
                                    last_hd_pose.block<3, 3>(0, 0).transpose())
                      .angle());
            }
            last_hd_pose = hd_pose;
            has_last_hd_pose = true;
          }

          // 2. 判斷車輛是否處於物理靜止狀態 (使用地圖匹配差，或高精度 GPS
          // 位移與速度)
          bool is_static_by_match =
              (hd_map_matched_this_frame && has_last_hd_pose &&
               match_dist < 0.10 && match_angle < 0.01);
          bool is_static_by_gps =
              (current_gps_speed < 0.15 && gps_displacement < 0.08);

          static bool was_zupt_locked = false;

          if (is_static_by_match || is_static_by_gps) {
            static_frame_count++;
          } else {
            static_frame_count = 0;
            if (was_zupt_locked) {
              ROS_INFO("[ZUPT] Vehicle started moving (GNSS Speed: %.2f). "
                       "Breaking lock!",
                       current_gps_speed);
              was_zupt_locked = false;
            }
          }

          // 連續 3 幀 (0.3秒) 都符合靜止條件，則觸發 ZUPT
          if (static_frame_count >= 3) {
            current_pose = last_process_pose; // 凍結當前位姿，防止點雲抖動
            backend_->addZUPTFactor();        // 告訴後端優化器目前速度為 0
            was_zupt_locked = true;
            ROS_INFO_THROTTLE(
                2.0,
                "[ZUPT] Vehicle is stationary. Applying Zero Velocity Update.");
          }
          last_process_pose = current_pose;
          // ---------------------------------------------

          // 4. Step A: Update Backend Odometry Factor
          // 即使匹配不佳，我們也必須讓後端知道 IMU
          // 的預測位移，否則預測起點會永遠卡在過去
          scan_match_failed = should_skip_frame;
          backend_->addOdomFactor(lidar_time, current_pose,
                                  hd_map_matched_this_frame, scan_match_failed,
                                  is_turning);

          if (hd_map_matched_this_frame) {
            // Add absolute prior to the newly created graph node
            backend_->addHDMapFactor(hd_pose, hd_fitness);
          }

          // 5. Update Backend and Optimize
          int opt_iters = hd_map_matched_this_frame ? 10 : 1;
          backend_->optimize(opt_iters);

          // 獲取優化後的位姿 (即使匹配失敗，這裡也會得到 IMU 修正後的結果)
          gtsam::Pose3 optimized_pose = backend_->getCurrentPose();
          Eigen::Matrix4f opt_pose_mat = optimized_pose.matrix().cast<float>();

          if (static_frame_count >= 3) {
            // 車輛處於 ZUPT 靜止狀態。
            // 為了防止 GNSS 雜訊導致優化器產生的 opt_pose_mat
            // 抖動，我們拋棄 it。
            delta_transform = Eigen::Matrix4f::Identity();
            gtsam::Pose3 static_pose3(current_pose.cast<double>());
            backend_->setCurrentPose(static_pose3);
          } else {
            delta_transform = opt_pose_mat * current_pose.inverse();
            // 同步座標系
            frontend_->shiftLocalMap(delta_transform);
            current_pose = opt_pose_mat;
            backend_->setCurrentPose(optimized_pose);
          }

          // 6. 新增 Keyframe 與發布資料
          // "批配成功的點雲才能製作localmap"
          // 當我們在 HD Map 外，或在 HD Map
          // 內但處於轉彎及20m緩衝期時，將點雲加入 Local Map 進行延伸
          if ((!currently_within_hd_bounds || use_temp_local_map) &&
              local_match_success) {
            if (!has_last_keyframe_) {
              last_keyframe_pose_ = current_pose;
              has_last_keyframe_ = true;
            }

            last_keyframe_pose_ = delta_transform * last_keyframe_pose_;

            double dist = (current_pose.block<3, 1>(0, 3) -
                           last_keyframe_pose_.block<3, 1>(0, 3))
                              .norm();
            Eigen::Matrix3f R_curr = current_pose.block<3, 3>(0, 0);
            Eigen::Matrix3f R_last = last_keyframe_pose_.block<3, 3>(0, 0);
            double angle = std::abs(
                Eigen::AngleAxisf(R_curr * R_last.transpose()).angle());

            if (dist > 1.0 || angle > 0.2 || !frontend_->hasKeyframes()) {
              frontend_->addKeyframeCloud(cloud, current_pose);
              last_keyframe_pose_ = current_pose;

              // 7. Loop Closure (只有在完全沒有 HD Map 時才進行全域迴圈偵測)
              if (!currently_within_hd_bounds && loop_closure_enabled_ &&
                  !should_skip_frame) {
                int loop_idx;
                Eigen::Matrix4f rel_pose;
                double loop_fitness;
                if (frontend_->detectLoopClosure(
                        current_pose, loop_closure_search_radius_, loop_idx,
                        rel_pose, loop_fitness)) {
                  if (loop_fitness < loop_closure_fitness_score_) {
                    backend_->addLoopFactor(loop_idx, rel_pose, loop_fitness);
                    backend_->optimize(10);
                  }
                }
              }
            }

            // 7. 更新 Local Map 用於下一幀匹配與發布
            frontend_->getLocalMap(local_map);
          }
          latest_pose_ = current_pose;
        } else {
          // 對於跳過的幀（頻率限制），我們僅使用 IMU 預測位姿進行補間發布
          Eigen::Matrix4f predicted_pose =
              backend_->getPredictedPose().matrix().cast<float>();
          double predict_jump = (predicted_pose.block<3, 1>(0, 3) -
                                 current_pose.block<3, 1>(0, 3))
                                    .norm();
          Eigen::Matrix3f R_curr = current_pose.block<3, 3>(0, 0);
          Eigen::Matrix3f R_pred = predicted_pose.block<3, 3>(0, 0);
          double predict_angle =
              std::abs(Eigen::AngleAxisf(R_pred * R_curr.transpose()).angle());
          if (predict_jump < max_trans_jump_ && predict_angle < max_rot_jump_) {
            current_pose = predicted_pose;
          }
          ROS_INFO_THROTTLE(5.0,
                            "[INFO] Skip Scan Matching (Frequency limit).");
        }

        // 8. Final Visualization (Always publish at lidar rate, but avoid
        // redundant stamps)
        static double last_published_lidar_time = -1.0;
        if (lidar_time > last_published_lidar_time) {
          // 在發布前重新獲取一次 Local Map，確保最新加入的 Keyframe
          // 也能立即顯示
          frontend_->getLocalMap(local_map);
          publishData(lidar_time, current_pose, cloud, local_map);
          last_published_lidar_time = lidar_time;
        }
        latest_pose_ = current_pose;

        /*
        // === DEBUG: 永遠存下 光達對地圖(HD Map) 的匹配結果 ===
        {
          CloudType::Ptr matched_pcl_debug(new CloudType());
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
      pub_local_map_, pub_fitness_, pub_gps_path_, pub_range_rings_;
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
  double max_trans_jump_;
  double max_rot_jump_;

  // Loop Closure Parameters
  bool loop_closure_enabled_;

  pcl::KdTreeFLANN<PointType>::Ptr hd_map_kdtree_;
  double loop_closure_search_radius_;
  double loop_closure_fitness_score_;

  Eigen::Matrix4f last_keyframe_pose_ = Eigen::Matrix4f::Identity();
  bool has_last_keyframe_ = false;

  CloudType::Ptr current_global_map_;
  double last_imu_time_internal_ = -1.0;
  double last_record_time_ = -1.0;

  CloudType::Ptr loadPcdCloud(const std::string &path) {
    CloudType::Ptr cloud(new CloudType);
    if (pcl::io::loadPCDFile<PointType>(path, *cloud) == -1) {
      ROS_ERROR_THROTTLE(5.0, "Couldn't read PCD file: %s", path.c_str());
    }
    return cloud;
  }

  CloudType::Ptr loadBinCloud(const std::string &path) {
    CloudType::Ptr cloud(new CloudType);
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
      CloudType dummy_map;
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
                   const CloudType::Ptr &cloud,
                   const CloudType::Ptr &local_map) {
    // 只有在 HD Map
    // 已經加載且點雲成功對地圖完成第一次匹配後，才允許發布任何資料 (包含
    // TF、點雲與里程計) 這能完全避免在匹配成功前，RViz
    // 畫面上先顯示錯誤坐標的點雲與軌跡，消除瞬間巨大跳變！
    if (!first_map_check_done_ || !has_hd_map_matched_) {
      return;
    }

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

    // 只有在 HD Map
    // 已經加載且點雲成功對地圖完成第一次匹配後，才允許開始繪製軌跡 (Path)
    // 這能確保第一個 Path
    // 的位置就是精準匹配完的位置，完全消除軌跡起點的側向跳變！
    if (first_map_check_done_ && has_hd_map_matched_) {
      global_path_.header.stamp = rts;
      global_path_.poses.push_back(ps);
      pub_path_.publish(global_path_);
    }

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
    CloudType::Ptr world_cloud(new CloudType());
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
    sensor_msgs::PointCloud2 local_map_msg;
    if (local_map && !local_map->empty()) {
      pcl::toROSMsg(*local_map, local_map_msg);
    } else {
      // 發布隱形假點強迫 RViz 清除畫面上的舊 Local Map
      CloudType dummy_map;
      PointType p;
      p.x = 0;
      p.y = 0;
      p.z = -10000.0;
      dummy_map.push_back(p);
      pcl::toROSMsg(dummy_map, local_map_msg);
    }
    local_map_msg.header.stamp = rts;
    local_map_msg.header.frame_id = "map";
    pub_local_map_.publish(local_map_msg);

    // 4.8 Publish Range Rings (5m & 10m)
    publishRangeRings(rts);

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
        tp.roll = std::atan2(sinr_cosp, cosr_cosp);

        double sinp = 2 * (q_veh.w() * q_veh.y() - q_veh.z() * q_veh.x());
        if (std::abs(sinp) >= 1)
          tp.pitch = std::copysign(M_PI / 2, sinp);
        else
          tp.pitch = std::asin(sinp);

        // Calculate Heading (Yaw) using the VEHICLE'S forward vector
        Eigen::Vector3f veh_forward_in_map =
            T_map_imu.block<3, 3>(0, 0) * Eigen::Vector3f::UnitX();
        tp.yaw = std::atan2(veh_forward_in_map.x(), veh_forward_in_map.y());

        recorded_trajectory_.push_back(tp);
      }
    }
  }

  void publishRangeRings(const ros::Time &stamp) {
    visualization_msgs::MarkerArray marker_array;

    auto createCircle = [&](int id, double radius, double r, double g,
                            double b) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "lidar"; // Follow the lidar sensor
      marker.header.stamp = stamp;
      marker.ns = "range_rings";
      marker.id = id;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.action = visualization_msgs::Marker::ADD;
      marker.scale.x = 0.5; // Line width (changed from 1.0m to 5cm)

      // Explicitly initialize position and orientation
      marker.pose.position.x = 0.0;
      marker.pose.position.y = 0.0;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.color.a = 1.0; // Changed to 1.0 for better visibility in RViz

      int num_segments = 100;
      for (int i = 0; i <= num_segments; ++i) {
        double angle = 2.0 * M_PI * i / num_segments;
        geometry_msgs::Point p;
        p.x = radius * std::cos(angle);
        p.y = radius * std::sin(angle);
        p.z = 0.0;
        marker.points.push_back(p);
      }
      return marker;
    };

    marker_array.markers.push_back(
        createCircle(0, 5.0, 0.0, 1.0, 1.0)); // Cyan 5m
    marker_array.markers.push_back(
        createCircle(1, 10.0, 0.0, 1.0, 1.0)); // Cyan 10m

    pub_range_rings_.publish(marker_array);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_scan_match_c_node");
  ros::NodeHandle nh("~");
  LidarSlamSystem slam(nh);
  ros::spin();
  return 0;
}
