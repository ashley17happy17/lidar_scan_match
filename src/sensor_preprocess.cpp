#include "lidar_scan_match_c/sensor_preprocess.hpp"
#include <chrono>
#include <lidar_utils/types.hpp>

#include <sensor_msgs/point_cloud2_iterator.h>

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
  std::vector<double> timestamps;
  size_t num_points = msg->width * msg->height;
  raw_cloud->reserve(num_points);
  timestamps.reserve(num_points);

  double base_unix_time = msg->header.stamp.toSec();

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  // 1. Extract X, Y, Z
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    lidar_utils::PointType p;
    p.x = *iter_x;
    p.y = *iter_y;
    p.z = *iter_z;
    p.intensity = 0.0f; // Default intensity
    raw_cloud->push_back(p);
  }

  // 2. Extract Intensity or Reflectivity
  bool has_intensity = false, has_reflectivity = false;
  int intensity_type = 0, reflectivity_type = 0;
  for (const auto &f : msg->fields) {
    if (f.name == "intensity") {
      has_intensity = true;
      intensity_type = f.datatype;
    }
    if (f.name == "reflectivity") {
      has_reflectivity = true;
      reflectivity_type = f.datatype;
    }
  }

  if (has_intensity) {
    if (intensity_type == sensor_msgs::PointField::FLOAT32) {
      sensor_msgs::PointCloud2ConstIterator<float> iter_i(*msg, "intensity");
      for (size_t i = 0; i < num_points && iter_i != iter_i.end();
           ++i, ++iter_i)
        raw_cloud->points[i].intensity = *iter_i;
    } else if (intensity_type == sensor_msgs::PointField::UINT16) {
      sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_i(*msg, "intensity");
      for (size_t i = 0; i < num_points && iter_i != iter_i.end();
           ++i, ++iter_i)
        raw_cloud->points[i].intensity = static_cast<float>(*iter_i);
    } else if (intensity_type == sensor_msgs::PointField::UINT8) {
      sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_i(*msg, "intensity");
      for (size_t i = 0; i < num_points && iter_i != iter_i.end();
           ++i, ++iter_i)
        raw_cloud->points[i].intensity = static_cast<float>(*iter_i);
    }
  } else if (has_reflectivity) {
    if (reflectivity_type == sensor_msgs::PointField::FLOAT32) {
      sensor_msgs::PointCloud2ConstIterator<float> iter_r(*msg, "reflectivity");
      for (size_t i = 0; i < num_points && iter_r != iter_r.end();
           ++i, ++iter_r)
        raw_cloud->points[i].intensity = *iter_r;
    } else if (reflectivity_type == sensor_msgs::PointField::UINT16) {
      sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_r(*msg,
                                                             "reflectivity");
      for (size_t i = 0; i < num_points && iter_r != iter_r.end();
           ++i, ++iter_r)
        raw_cloud->points[i].intensity = static_cast<float>(*iter_r);
    } else if (reflectivity_type == sensor_msgs::PointField::UINT8) {
      sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_r(*msg,
                                                            "reflectivity");
      for (size_t i = 0; i < num_points && iter_r != iter_r.end();
           ++i, ++iter_r)
        raw_cloud->points[i].intensity = static_cast<float>(*iter_r);
    }
  }

  // 3. Extract Timestamps
  bool has_time = false, has_t = false;
  int time_type = 0, t_type = 0;
  for (const auto &f : msg->fields) {
    if (f.name == "time") {
      has_time = true;
      time_type = f.datatype;
    }
    if (f.name == "t") {
      has_t = true;
      t_type = f.datatype;
    }
  }

  if (has_time) {
    // Velodyne typically uses 'time' (float or double, in seconds relative to
    // scan)
    if (time_type == sensor_msgs::PointField::FLOAT64) {
      sensor_msgs::PointCloud2ConstIterator<double> iter_t(*msg, "time");
      for (size_t i = 0; i < num_points && iter_t != iter_t.end();
           ++i, ++iter_t)
        timestamps.push_back(base_unix_time + *iter_t);
    } else if (time_type == sensor_msgs::PointField::FLOAT32) {
      sensor_msgs::PointCloud2ConstIterator<float> iter_t(*msg, "time");
      for (size_t i = 0; i < num_points && iter_t != iter_t.end();
           ++i, ++iter_t)
        timestamps.push_back(base_unix_time + static_cast<double>(*iter_t));
    }
  } else if (has_t) {
    // Ouster typically uses 't' (uint32, in nanoseconds)
    if (t_type == sensor_msgs::PointField::UINT32) {
      sensor_msgs::PointCloud2ConstIterator<uint32_t> iter_t(*msg, "t");
      for (size_t i = 0; i < num_points && iter_t != iter_t.end();
           ++i, ++iter_t)
        timestamps.push_back(base_unix_time +
                             static_cast<double>(*iter_t) / 1e9);
    } else if (t_type == sensor_msgs::PointField::FLOAT64) {
      sensor_msgs::PointCloud2ConstIterator<double> iter_t(*msg, "t");
      for (size_t i = 0; i < num_points && iter_t != iter_t.end();
           ++i, ++iter_t)
        timestamps.push_back(base_unix_time + *iter_t);
    } else if (t_type == sensor_msgs::PointField::FLOAT32) {
      sensor_msgs::PointCloud2ConstIterator<float> iter_t(*msg, "t");
      for (size_t i = 0; i < num_points && iter_t != iter_t.end();
           ++i, ++iter_t)
        timestamps.push_back(base_unix_time + static_cast<double>(*iter_t));
    }
  } else {
    // Fallback if no valid time field exists
    for (size_t i = 0; i < num_points; ++i)
      timestamps.push_back(base_unix_time);
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

  // ROS_INFO_THROTTLE(1.0,
  //                   "[BackEnd] Time - Crop: %.2f ms, Denoise: %.2f ms, "
  //                   "Artifact: %.2f ms, Downsample: %.2f ms",
  //                   std::chrono::duration<double, std::milli>(t2 -
  //                   t1).count(), std::chrono::duration<double, std::milli>(t3
  //                   - t2).count(), std::chrono::duration<double,
  //                   std::milli>(t4 - t3).count(),
  //                   std::chrono::duration<double, std::milli>(t5 -
  //                   t4).count());

  // CRITICAL: You forgot to assign the result to out_cloud!
  *out_cloud = *raw_cloud;
  out_timestamps = std::move(timestamps);
}
} // namespace lidar_scan_match_c
