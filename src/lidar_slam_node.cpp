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
#include <cmath>
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
    start_time_ = std::chrono::steady_clock::now();
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
    nh_.param<double>("gnss_diff_min_speed", gnss_diff_min_speed_, 2.0);

    // Loop Closure
    nh_.param<bool>("loop_closure_enabled", loop_closure_enabled_, true);
    nh_.param<double>("loop_closure_search_radius", loop_closure_search_radius_,
                      15.0);
    nh_.param<double>("loop_closure_fitness_score", loop_closure_fitness_score_,
                      0.3);

    nh_.param<double>("max_single_frame_translation", max_trans_jump_, 1.5);
    nh_.param<double>("max_single_frame_rotation", max_rot_jump_, 0.5);

    // Local Map failure recovery: gate relaxes with each consecutive
    // rejection so a merely-drifted (but otherwise valid) match isn't stuck
    // behind a fixed threshold; past the cap we give up and re-anchor.
    nh_.param<double>("local_match_jump_gate", local_match_jump_gate_, 1.0);
    nh_.param<double>("local_match_jump_relax_step",
                      local_match_jump_relax_step_, 0.5);
    nh_.param<int>("local_match_max_consecutive_failures",
                   local_match_max_failures_, 5);
    // Outside the HD Map, once lidar odometry has been failing for a while,
    // low-precision GPS is no longer worse than unconstrained IMU dead
    // reckoning - inject it loosely as a floor against unbounded drift.
    nh_.param<int>("gps_fallback_min_failures", gps_fallback_min_failures_, 3);
    // Outside the HD Map, scan-to-local-map is much more prone to
    // degenerate/no-map drift than a live (even non-RTK) GPS fix - trust it
    // continuously there instead of only as a post-failure fallback.
    nh_.param<double>("gps_std_thres_no_map", gps_std_thres_no_map_, 5.0);

    // Manual override: outside the HD Map, trust GPS/IMU prediction directly
    // and skip lidar scan-to-local-map correction entirely. Use this when
    // the covariance-based trust logic above isn't enough - e.g. the GPS
    // source doesn't report usable covariance, so it can never win the
    // has_cov-gated comparisons no matter how good it actually is.
    nh_.param<bool>("no_map_use_gps_only", no_map_use_gps_only_, false);
    nh_.param<double>("no_map_gps_fixed_std", no_map_gps_fixed_std_, 2.0);

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
    std::cout << "[INFO] LidarSlamSystem shutting down..." << std::endl;
    // Join first so total_pipeline_cost_ms_ (written by process_thread_) is
    // done updating before we read it below.
    if (process_thread_.joinable())
      process_thread_.join();
    if (map_loader_thread_.joinable())
      map_loader_thread_.join();

    double elapsed_s = std::chrono::duration<double>(
                           std::chrono::steady_clock::now() - start_time_)
                           .count();
    int elapsed_min = static_cast<int>(elapsed_s) / 60;
    double elapsed_sec_rem = elapsed_s - elapsed_min * 60;
    std::cout << "[INFO] Total runtime: " << elapsed_min << "m "
              << std::fixed << std::setprecision(1) << elapsed_sec_rem
              << "s (" << elapsed_s << "s) | Total computation time: "
              << (total_pipeline_cost_ms_ / 1000.0) << "s ("
              << (elapsed_s > 0
                      ? (total_pipeline_cost_ms_ / 1000.0 / elapsed_s * 100.0)
                      : 0.0)
              << "% of runtime)" << std::endl;
    if (out_save_) {
      std::cout << "[INFO] Saving trajectory to " << out_trajectory_path_
                << std::endl;
      saveTrajectory();
      std::cout << "[INFO] Trajectory saved successfully." << std::endl;
    }
  }

private:
  struct TrajectoryPoint {
    double time;
    double lat, lon, h;
    double twdx, twdy, twdz;
    double roll, pitch, yaw;
    std::string source;
  };
  std::vector<TrajectoryPoint> recorded_trajectory_;

  // Interpolate an SE(3) pose at fraction t in [0,1] between a and b
  // (SLERP for rotation, LERP for translation).
  static Eigen::Matrix4f interpolatePose(const Eigen::Matrix4f &a,
                                         const Eigen::Matrix4f &b, float t) {
    t = std::max(0.0f, std::min(1.0f, t));
    Eigen::Quaternionf qa(a.block<3, 3>(0, 0));
    Eigen::Quaternionf qb(b.block<3, 3>(0, 0));
    Eigen::Quaternionf q = qa.slerp(t, qb);
    Eigen::Vector3f p =
        (1.0f - t) * a.block<3, 1>(0, 3) + t * b.block<3, 1>(0, 3);
    Eigen::Matrix4f out = Eigen::Matrix4f::Identity();
    out.block<3, 3>(0, 0) = q.toRotationMatrix();
    out.block<3, 1>(0, 3) = p;
    return out;
  }

  // Convert a T_map_lidar pose into a TrajectoryPoint stamped at `t`.
  TrajectoryPoint poseToTrajectoryPoint(const Eigen::Matrix4f &pose, double t) {
    TrajectoryPoint tp;
    Eigen::Matrix4f T_imu_lidar = backend_->getExtrinsic().matrix().cast<float>();
    Eigen::Matrix4f T_map_imu = pose * T_imu_lidar.inverse();
    Eigen::Quaternionf q_veh(T_map_imu.block<3, 3>(0, 0));

    tp.time = t;
    tp.twdx = T_map_imu(0, 3) + map_origin_twd97_.x();
    tp.twdy = T_map_imu(1, 3) + map_origin_twd97_.y();
    tp.twdz = T_map_imu(2, 3) + map_origin_twd97_.z();

    // Convert Map Frame (ENU) to Lat/Lon/H
    gps_translator_internal_.GetWGS84(tp.twdx, tp.twdy, tp.twdz, tp.lat, tp.lon,
                                      tp.h);

    // Convert Vehicle Quaternion (q_veh) to Roll, Pitch, Yaw (rad)
    double sinr_cosp = 2 * (q_veh.w() * q_veh.x() + q_veh.y() * q_veh.z());
    double cosr_cosp = 1 - 2 * (q_veh.x() * q_veh.x() + q_veh.y() * q_veh.y());
    tp.roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2 * (q_veh.w() * q_veh.y() - q_veh.z() * q_veh.x());
    if (std::abs(sinp) >= 1)
      tp.pitch = std::copysign(M_PI / 2, sinp);
    else
      tp.pitch = std::asin(sinp);

    // Heading (Yaw) using the vehicle's forward vector
    Eigen::Vector3f veh_forward_in_map =
        T_map_imu.block<3, 3>(0, 0) * Eigen::Vector3f::UnitX();
    tp.yaw = std::atan2(veh_forward_in_map.x(), veh_forward_in_map.y());
    tp.source = "MAP";
    return tp;
  }

  void saveTrajectory() {
    std::string filename = out_trajectory_path_;
    nh_.getParam("out_trajectory_path", filename);
    std::ofstream f(filename);
    if (!f.is_open()) {
      ROS_ERROR("[Output] Cannot open %s for writing trajectory",
                filename.c_str());
      return;
    }
    f << "time,lat,lon,h,twd97x,twd97y,twd97z,roll_rad,pitch_rad,yaw_rad,source\n";
    for (const auto &p : recorded_trajectory_) {
      // Timestamp in integer milliseconds (sub-ms digits truncated).
      long long time_ms = static_cast<long long>(std::round(p.time * 1000.0));
      f << time_ms << std::fixed << std::setprecision(8) << "," << p.lat << ","
        << p.lon << "," << p.h << "," << p.twdx << "," << p.twdy << ","
        << p.twdz << "," << p.roll << "," << p.pitch << "," << p.yaw << ","
        << p.source << "\n";
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
              ROS_WARN("[WARN] MapLoader: HD map directory does not exist: %s",
                       hd_map_dir.c_str());
            }
          } catch (const std::exception &e) {
            ROS_ERROR("[ERROR] MapLoader: Error reading map directory: %s",
                      e.what());
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
              // ROS_INFO("map loaded.\n");
            }
          }
        }

        first_map_check_done_ = true;
        last_checked_pos = current_pos;

        if (!merged_tiles->empty()) {
          // Optimized for real-time: Downsample the HD map to 0.5m density
          CloudType::Ptr optimized_map(new CloudType());
          pcl::VoxelGrid<PointType> vg;
          vg.setLeafSize(0.2f, 0.2f, 0.2f);
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
            Eigen::Matrix3f R_imu = q.toRotationMatrix();
            Eigen::Vector3f euler =
                R_imu.eulerAngles(2, 1, 0); // Yaw, Pitch, Roll
            double yaw_imu = euler[0];
            double pitch_imu = euler[1];
            double roll_imu = euler[2];

            // Align North-Up IMU (0) to ENU Y-axis (90)
            double yaw_enu = -yaw_imu + (M_PI / 2.0);

            Eigen::Matrix4f T_world = Eigen::Matrix4f::Identity();
            T_world.block<3, 3>(0, 0) =
                (Eigen::AngleAxisf(yaw_enu, Eigen::Vector3f::UnitZ()) *
                 Eigen::AngleAxisf(pitch_imu, Eigen::Vector3f::UnitY()) *
                 Eigen::AngleAxisf(roll_imu, Eigen::Vector3f::UnitX()))
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
      static int consecutive_local_match_failures = 0;
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

          // Collect the GPS fix so the trajectory can be recorded at the exact
          // GPS instant (pose interpolated in publishData), not the lidar frame
          // time. The position and speed ride along so publishData can also
          // report the along-track GNSS/SLAM difference at that same instant.
          pending_gps_.push_back(
              {gm.timestamp,
               Eigen::Vector3d(gm.latitude, gm.longitude, gm.altitude),
               current_gps_speed});

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

          if (!currently_within_hd_bounds && no_map_use_gps_only_) {
            // 手動切換：地圖外一律直接信任 GPS，不管 covariance
            // 是否可用 - 用來因應 GPS 來源根本沒有回報有效
            // covariance，導致上面以 covariance 為準的判斷永遠無法
            // 讓 GPS 勝出的情況
            double fixed_var = no_map_gps_fixed_std_ * no_map_gps_fixed_std_;
            gm.covariance_diag << fixed_var, fixed_var, fixed_var;
            backend_->addGpsFactor(gm);
            ROS_INFO_THROTTLE(
                2.0,
                "[INFO] GPS: no_map_use_gps_only active - trusting GPS "
                "directly outside HD Map (fixed 2DStdDev: %.2fm).",
                no_map_gps_fixed_std_);
          } else if (has_cov && cov_h < gps_std_thres_) {
            // 將高精度的變異數真實反映給 GTSAM，並乘上人工權重係數 (Covariance
            // 越大代表越不信任)
            if (!currently_within_hd_bounds || !has_hd_map_matched_ ||
                consecutive_hd_map_failures >= 3) {
              gm.covariance_diag << cov_x * gps_cov_multiplier_,
                  cov_y * gps_cov_multiplier_, cov_z * gps_cov_multiplier_;
              backend_->addGpsFactor(gm);

              if (currently_within_hd_bounds &&
                  consecutive_hd_map_failures >= 3) {
                ROS_WARN_THROTTLE(1.0, "[INFO] GPS: HD Map failed >= 3 times! "
                                       "Fallback to RTK GNSS injected!");
              } else if (!has_hd_map_matched_) {
                ROS_INFO_THROTTLE(2.0,
                                  "[INFO] GPS: Trusting RTK GPS tightly "
                                  "for initialization! "
                                  "2DStdDev: %.2fm",
                                  cov_h);
              } else {
                ROS_INFO_THROTTLE(
                    1.0,
                    "[INFO] GPS: RTK High-Precision GPS injected! "
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
          } else if (!currently_within_hd_bounds && has_cov &&
                    cov_h < gps_std_thres_no_map_) {
            // 沒有 HD Map 時，Lidar 只能靠 scan-to-local-map，比起有 HD Map
            // 校正時更容易因幾何退化或長期漂移而失準。因此在地圖範圍外，
            // 就算不是 RTK 等級，也直接以真實 covariance 信任 GPS，而非
            // 等到連續失敗才注入 - 讓 GTSAM 依照真實精度自然權衡兩者
            gm.covariance_diag << cov_x * gps_cov_multiplier_,
                cov_y * gps_cov_multiplier_, cov_z * gps_cov_multiplier_;
            backend_->addGpsFactor(gm);
            ROS_INFO_THROTTLE(
                1.0,
                "[INFO] GPS: Standard-precision GPS trusted over local-map "
                "ICP outside HD Map (2DStdDev: %.2fm).",
                cov_h);
          } else if ((!currently_within_hd_bounds &&
                     consecutive_local_match_failures >=
                         gps_fallback_min_failures_) ||
                    (currently_within_hd_bounds &&
                     consecutive_hd_map_failures >=
                         gps_fallback_min_failures_)) {
            // Lidar 校正 (Local Map 或 HD Map) 已連續失敗多次：純 IMU
            // 航位推算會無限漂移，此時就算是低精度 GPS 也比完全沒有校正好，
            // 以寬鬆的 covariance 注入做為漂移的下限保護，讓 predicted_pose
            // 不至於無界漂移，而不是放寬會影響車道正確性的匹配門檻
            double loose_var = std::max(cov_h * cov_h, 25.0);
            gm.covariance_diag << loose_var, loose_var, loose_var;
            backend_->addGpsFactor(gm);
            ROS_WARN_THROTTLE(
                1.0,
                "[INFO] GPS: Lidar correction failed repeatedly (local:%d, "
                "hd:%d). Injecting low-precision GPS as drift floor "
                "(2DStdDev: %.2fm).",
                consecutive_local_match_failures, consecutive_hd_map_failures,
                cov_h);
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
      cloud->header.stamp = lidar_time * 1e6; // Store timestamp in microseconds

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

          // --- Extended Turn Tracking (Keep continuous HD Map for 10m after
          // turn) ---
          static bool was_turning_extended = false;
          static Eigen::Vector3f turn_end_pos = Eigen::Vector3f::Zero();

          if (is_turning) {
            was_turning_extended = true;
            turn_end_pos = current_pose.block<3, 1>(0, 3);
          } else if (was_turning_extended) {
            double dist_since_turn =
                (current_pose.block<3, 1>(0, 3) - turn_end_pos).norm();
            if (dist_since_turn >= 10.0) {
              was_turning_extended = false;
              ROS_INFO("[HD Map] Post-turn 10m threshold reached. Returning to "
                       "periodic updates.");
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

            if (is_turning || was_turning_extended) {
              periodic_update = true;
            }

            // 清空 Local Map：一旦進入 HD Map 範圍，就不再需要 Local
            // Map，應立即清空以避免在 RViz 殘留
            if (has_last_keyframe_) {
              frontend_->clearLocalMap();
              local_map->clear();
              has_last_keyframe_ = false;
            }

            // == 1. 全域匹配 (HD Map) ==
            if (periodic_update) {
              hd_match_count = 0;
              // CRITICAL FIX: Always use pure IMU predicted_pose as the initial
              // guess for HD Map! If we use current_pose here, any slip/drift
              // from the Local Map will corrupt the HD Map guess!
              Eigen::Matrix4f rescue_pose = predicted_pose;
              bool is_init = !has_hd_map_matched_;
              if (frontend_->scanToHDMapMatch(cloud, rescue_pose, hd_fitness,
                                              is_turning, is_init)) {
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

                if (hd_fitness < threshold && lateral_jump < 10.0) {
                  // If HD Map pulled the pose by more than 0.5m, the Local Map
                  // is internally distorted (ghosted). Clear it!
                  double correction_jump = (rescue_pose.block<3, 1>(0, 3) -
                                            current_pose.block<3, 1>(0, 3))
                                               .norm();
                  if (correction_jump > 0.5) {
                    ROS_WARN_THROTTLE(
                        1.0,
                        "[WARN] Local Map: HD Map correction %.2fm is "
                        "large. Clearing distorted Local Map!",
                        correction_jump);
                    frontend_->clearLocalMap();
                    local_map->clear();
                  }

                  current_pose = rescue_pose; // 以 HD Map 全域結果為最終姿態
                  hd_map_matched_this_frame = true;
                  hd_pose = rescue_pose;
                  has_hd_map_matched_ = true;
                  consecutive_hd_map_failures = 0;
                } else {
                  consecutive_hd_map_failures++;
                  if (hd_fitness < threshold && lateral_jump >= 10.0) {
                    ROS_WARN_THROTTLE(1.0,
                                      "[HD Map] Match rejected due to large "
                                      "lateral jump: %.2fm >= 10.0m",
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

            if (no_map_use_gps_only_) {
              // 手動切換：地圖外不做 lidar 修正，直接信任 GPS/IMU
              // 融合後的 current_pose (predicted_pose)。local map/keyframe
              // 仍然照常累積，供視覺化與日後重新進入 HD Map 使用
              local_match_success = true;
              consecutive_local_match_failures = 0;
              if (!has_last_keyframe_) {
                has_last_keyframe_ = true;
                last_keyframe_pose_ = current_pose;
              }
            } else if (has_last_keyframe_ && !local_map->empty()) {
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
                // 每次連續失敗就放寬跳動門檻：先前的失敗會讓 current_pose
                // 持續以純 IMU 漂移，導致本來合理的匹配也被固定門檻擋下
                double jump_gate =
                    local_match_jump_gate_ +
                    consecutive_local_match_failures * local_match_jump_relax_step_;
                if (jump < jump_gate) {
                  current_pose =
                      local_pose; // 永遠採信 Lidar Odometry 避免 IMU 預測失控
                  local_match_success = true;
                  consecutive_local_match_failures = 0;

                  if (fitness >= local_threshold) {
                    ROS_WARN_THROTTLE(
                        1.0,
                        "[WARN] Local Map: High fitness %.2f > %.2f, "
                        "but map growth is allowed.",
                        fitness, local_threshold);
                  }
                } else {
                  consecutive_local_match_failures++;
                  ROS_WARN_THROTTLE(
                      1.0,
                      "[WARN] Local Map: Rejected match: jump=%.2fm > "
                      "%.2fm (fail #%d)",
                      jump, jump_gate, consecutive_local_match_failures);
                }
              } else {
                consecutive_local_match_failures++;
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
              consecutive_local_match_failures = 0;
              ROS_INFO(
                  "[INFO] Local Map: Creating new local map anchor outside HD "
                  "Map.");
            }

            // 連續失敗次數過多：local map 可能已與實際場景脫節（幾何退化
            // 或長期漂移），與其繼續卡死，不如直接以目前 IMU 預測位姿
            // 重建 local map，讓系統有機會重新對齊
            if (!local_match_success &&
                consecutive_local_match_failures >= local_match_max_failures_) {
              ROS_WARN("[WARN] Local Map: %d consecutive failures. Forcing "
                       "local map refresh at current pose.",
                       consecutive_local_match_failures);
              frontend_->clearLocalMap();
              local_map->clear();
              local_match_success = true;
              has_last_keyframe_ = true;
              last_keyframe_pose_ = current_pose;
              consecutive_local_match_failures = 0;
            }

            if (!local_match_success) {
              should_skip_frame = true;
              ROS_WARN_THROTTLE(
                  1.0,
                  "[WARN] Failsafe: Local match failed. Using IMU prediction.");
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
              ROS_INFO(
                  "[INFO] ZUPT: Vehicle started moving (GNSS Speed: %.2f). "
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
            ROS_INFO_THROTTLE(2.0, "[INFO] ZUPT: Vehicle is stationary. "
                                   "Applying Zero Velocity Update.");
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
          if (!currently_within_hd_bounds && local_match_success) {
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
              ROS_INFO("[DEBUG] Adding keyframe! dist: %.2f, angle: %.2f", dist, angle);
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
            static double last_local_map_build_time = -1.0;
            // Force map generation every frame by removing the 0.1s check to avoid float precision bugs
            frontend_->getLocalMap(local_map);
            ROS_INFO_THROTTLE(1.0, "[DEBUG] getLocalMap built! keyframes: %ld, local_map points: %ld", 
                              frontend_->getKeyframesSize(), local_map->size());
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
          // Local map is now built at a specific timegap (0.5s) earlier in the
          // loop. We directly use the cached local_map here instead of
          publishData(lidar_time, current_pose, cloud, local_map,
                      currently_within_hd_bounds);
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

        static double first_pipeline_data_time = -1.0;
        static int pipeline_frame_count = 0;

        total_pipeline_cost_ms_ += total_ms;
        pipeline_frame_count++;
        if (first_pipeline_data_time < 0) {
          first_pipeline_data_time = lidar_time;
        }
        double elapsed_data_time = lidar_time - first_pipeline_data_time;

        ROS_INFO("[Pipeline] Data Time: %.2f s | Cost: %.2f ms | Avg: %.2f ms | Total CPU: %.2f s",
                 elapsed_data_time, total_ms, total_pipeline_cost_ms_ / pipeline_frame_count, total_pipeline_cost_ms_ / 1000.0);

        if (buf_size > 2) {
          ROS_WARN_THROTTLE(
              1.0, "[WARN] Offline: catching up... Lidar Buffer Size: %zu",
              buf_size);
        }
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
  std::chrono::steady_clock::time_point start_time_;
  double total_pipeline_cost_ms_ = 0.0;
  // Trajectory recording: interpolation bracket + pending GPS timestamps.
  Eigen::Matrix4f traj_prev_pose_ = Eigen::Matrix4f::Identity();
  double traj_prev_time_ = -1.0;
  // One entry per GPS fix drained this cycle: the fix instant, its ENU position
  // (relative to map_origin_twd97_) and the GPS-derived speed at that instant.
  // Kept as a single struct so time and position can never drift out of step.
  struct PendingGps {
    double time;
    Eigen::Vector3d enu;
    double speed;
  };
  std::vector<PendingGps> pending_gps_;
  double gnss_diff_min_speed_;
  double gnss_lag_sum_ = 0.0;
  long gnss_lag_count_ = 0;
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
  double local_match_jump_gate_;
  double local_match_jump_relax_step_;
  int local_match_max_failures_;
  int gps_fallback_min_failures_;
  double gps_std_thres_no_map_;
  bool no_map_use_gps_only_;
  double no_map_gps_fixed_std_;

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
                   const CloudType::Ptr &cloud, const CloudType::Ptr &local_map,
                   bool currently_within_hd_bounds) {
    // 只有在 HD Map
    // 已經加載且點雲成功對地圖完成第一次匹配後，才允許發布任何資料 (包含
    // TF、點雲與里程計) 這能完全避免在匹配成功前，RViz
    // 畫面上先顯示錯誤坐標的點雲與軌跡，消除瞬間巨大跳變！
    if (!first_map_check_done_) {
      pending_gps_.clear();
      return;
    }

    // 若尚未匹配 HD Map，但車輛根本不在 HD Map 範圍內，則允許發布 Local Map
    // 與軌跡 (純粹依賴 GPS/IMU) 這樣在無地圖區域起步時，使用者依然能看到 Local
    // Map 不斷生成與更新
    static bool allow_publishing = false;
    if (has_hd_map_matched_ || !currently_within_hd_bounds) {
      allow_publishing = true;
    }

    if (!allow_publishing) {
      pending_gps_.clear();
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

    // 5. Record Trajectory Points at each GPS timestamp (pose interpolated to
    // the exact GPS instant so time and pose stay consistent), rate-limited
    // to out_hz_. GPS timestamps this cycle fall in (traj_prev_time_, ts].
    if (has_map_origin_ && traj_prev_time_ >= 0.0 && ts > traj_prev_time_) {
      double bracket = ts - traj_prev_time_;
      for (const auto &pg : pending_gps_) {
        // Clamp any GPS time to the bracket (should already be inside it).
        if (pg.time <= traj_prev_time_ || pg.time > ts)
          continue;

        float frac = static_cast<float>((pg.time - traj_prev_time_) / bracket);
        Eigen::Matrix4f interp_pose =
            interpolatePose(traj_prev_pose_, pose, frac);

        bool record = !(last_record_time_ >= 0 &&
                        pg.time - last_record_time_ < (0.95 / out_hz_));

        // Measure the GNSS/SLAM discrepancy on every fix, but only print it for
        // the fixes that get recorded, so the log lines correspond 1:1 with the
        // rows of the output trajectory and both run at out_hz_.
        logGnssTrackDiff(pg, interp_pose, record);

        if (!record)
          continue;

        recorded_trajectory_.push_back(
            poseToTrajectoryPoint(interp_pose, pg.time));
        last_record_time_ = pg.time;
      }
    }

    // Advance the interpolation bracket to this frame.
    traj_prev_time_ = ts;
    traj_prev_pose_ = pose;
    pending_gps_.clear();
  }

  // Decompose the GNSS-minus-SLAM offset into the vehicle's along-track and
  // cross-track axes at a single GPS instant. Both poses refer to the same
  // instant (the SLAM pose is interpolated), so a residual along-track term
  // that scales with speed is a time offset, not a position error - hence the
  // implied lag, which is the offset expressed in seconds.
  void logGnssTrackDiff(const PendingGps &pg, const Eigen::Matrix4f &pose,
                        bool do_print) {
    // Compare in the vehicle (IMU) frame, matching how the trajectory is
    // recorded and how GNSS enters the graph. The GNSS antenna lever arm is
    // not compensated, so cross-track carries a small constant bias.
    Eigen::Matrix4f T_imu_lidar =
        backend_->getExtrinsic().matrix().cast<float>();
    Eigen::Matrix4f T_map_imu = pose * T_imu_lidar.inverse();

    double dx = pg.enu.x() - static_cast<double>(T_map_imu(0, 3));
    double dy = pg.enu.y() - static_cast<double>(T_map_imu(1, 3));
    double dz = pg.enu.z() - static_cast<double>(T_map_imu(2, 3));
    double yaw = std::atan2(static_cast<double>(T_map_imu(1, 0)),
                            static_cast<double>(T_map_imu(0, 0)));

    // Positive along = GNSS ahead of SLAM; positive cross = GNSS to the left.
    double along = dx * std::cos(yaw) + dy * std::sin(yaw);
    double cross = -dx * std::sin(yaw) + dy * std::cos(yaw);

    // Accumulate the running lag estimate over EVERY fix, independent of how
    // often we print, so the mean is not thinned by the log throttle.
    bool have_lag = pg.speed >= gnss_diff_min_speed_;
    double implied_lag = 0.0;
    if (have_lag) {
      // GNSS trailing the SLAM pose (along < 0) means positive lag.
      implied_lag = -along / pg.speed;
      gnss_lag_sum_ += implied_lag;
      ++gnss_lag_count_;
    }

    if (!do_print)
      return;

    if (have_lag) {
      ROS_INFO("[GNSS-DIFF] t:%.3f along:%+7.2fm cross:%+6.2fm up:%+6.2fm "
               "|2D|:%5.2fm v:%5.2fm/s implied_lag:%+6.3fs "
               "(mean %+6.3fs over %ld)",
               pg.time, along, cross, dz, std::hypot(dx, dy), pg.speed,
               implied_lag, gnss_lag_sum_ / gnss_lag_count_, gnss_lag_count_);
    } else {
      ROS_INFO("[GNSS-DIFF] t:%.3f along:%+7.2fm cross:%+6.2fm up:%+6.2fm "
               "|2D|:%5.2fm v:%5.2fm/s implied_lag:n/a (below %.1fm/s; "
               "a lag collapses to 0 at standstill)",
               pg.time, along, cross, dz, std::hypot(dx, dy), pg.speed,
               gnss_diff_min_speed_);
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
