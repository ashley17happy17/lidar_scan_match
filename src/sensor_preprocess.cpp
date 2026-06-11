#include "sensor_preprocess.hpp"
#include <chrono>
#include <lidar_utils/types.hpp>

// Define a struct that EXACTLY matches your bag file's fields!
struct OusterPoint {
  PCL_ADD_POINT4D; // x, y, z
  float intensity;
  uint32_t t;    // Datatype 6 is uint32
  uint16_t ring; // Datatype 4 is uint16
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    OusterPoint,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                            intensity)(uint32_t, t,
                                                       t)(uint16_t, ring, ring))

namespace lidar_scan_match_c {

SensorPreprocess::SensorPreprocess(ros::NodeHandle &nh) {
  nh.param<double>("leaf_size", leaf_size_, 0.2);
  std::vector<double> min_b, max_b;
  if (nh.getParam("crop_min_bound", min_b) && min_b.size() >= 3) {
    crop_min_bound_ = Eigen::Vector3f(min_b[0], min_b[1], min_b[2]);
  } else {
    crop_min_bound_ = Eigen::Vector3f(10.0, 3.0, 1.8);
  }

  if (nh.getParam("crop_max_bound", max_b) && max_b.size() >= 3) {
    crop_max_bound_ = Eigen::Vector3f(max_b[0], max_b[1], max_b[2]);
  } else {
    crop_max_bound_ = Eigen::Vector3f(60.0, 60.0, 30.0);
  }
}

void SensorPreprocess::processCloud(const sensor_msgs::PointCloud2ConstPtr &msg,
                                    CloudType::Ptr &out_cloud,
                                    std::vector<double> &out_timestamps) {
  CloudType::Ptr raw_cloud(new CloudType());

  pcl::PointCloud<OusterPoint> cloud_with_time;
  pcl::fromROSMsg(*msg, cloud_with_time);

  std::vector<double> timestamps;
  raw_cloud->reserve(cloud_with_time.size());
  timestamps.reserve(cloud_with_time.size());

  // Get the base unix time of the entire scan from the ROS message header
  double base_unix_time = msg->header.stamp.toSec();

  for (const auto &pt : cloud_with_time.points) {
    pcl::PointXYZI p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = pt.z;
    p.intensity = pt.intensity;
    raw_cloud->push_back(p);

    // Ouster's 't' field is the RELATIVE offset in nanoseconds from the start
    // of the scan! To get the absolute Unix time, we must add the
    // base_unix_time.
    double point_unix_time = base_unix_time + (static_cast<double>(pt.t) / 1e9);
    timestamps.push_back(point_unix_time);
  }

  if (!timestamps.empty()) {
    // Just print the first one! Printing 130,000 points will crash your
    // terminal!
    ROS_INFO_THROTTLE(1.0, "[BackEnd] Base time: %.6f, Second point_time: %.6f",
                      base_unix_time, timestamps[1]);
  }

  auto t1 = std::chrono::high_resolution_clock::now();

  // === Crop Cloud ===
  Eigen::Vector3f minBound = crop_min_bound_;
  Eigen::Vector3f maxBound = crop_max_bound_;
  lidar_utils::CloudUtils::cropCloud(raw_cloud, timestamps, minBound, maxBound);

  auto t2 = std::chrono::high_resolution_clock::now();

  // === Denoise Cloud ===
  lidar_utils::CloudUtils::denoiseCloud(raw_cloud, 0.01f, 0.1f);

  auto t3 = std::chrono::high_resolution_clock::now();

  // // === Remove Artifact Cloud ===
  // lidar_utils::CloudUtils::removeArtifactCloud(raw_cloud, timestamps);

  auto t4 = std::chrono::high_resolution_clock::now();

  // === Downsample Cloud ===
  lidar_utils::CloudUtils::downsampleCloud(raw_cloud, timestamps, leaf_size_);

  auto t5 = std::chrono::high_resolution_clock::now();

  ROS_INFO_THROTTLE(1.0,
                    "[BackEnd] Time - Crop: %.2f ms, Denoise: %.2f ms, "
                    "Artifact: %.2f ms, Downsample: %.2f ms",
                    std::chrono::duration<double, std::milli>(t2 - t1).count(),
                    std::chrono::duration<double, std::milli>(t3 - t2).count(),
                    std::chrono::duration<double, std::milli>(t4 - t3).count(),
                    std::chrono::duration<double, std::milli>(t5 - t4).count());

  // CRITICAL: You forgot to assign the result to out_cloud!
  *out_cloud = *raw_cloud;
  out_timestamps = std::move(timestamps);
}
} // namespace lidar_scan_match_c
