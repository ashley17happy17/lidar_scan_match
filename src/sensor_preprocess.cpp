#include "sensor_preprocess.hpp"

namespace lidar_scan_match_c {

SensorPreprocess::SensorPreprocess(ros::NodeHandle &nh) {
  nh.param<double>("leaf_size", leaf_size_, 0.2);
  nh.param<double>("crop_vehicle_x", crop_vehicle_x_, 2.0);
  nh.param<double>("crop_vehicle_y", crop_vehicle_y_, 1.0);
  nh.param<double>("crop_vehicle_z", crop_vehicle_z_, -1.5);
  downSizeFilter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
}

void SensorPreprocess::processCloud(const sensor_msgs::PointCloud2ConstPtr &msg,
                                    PointCloudType::Ptr &out_cloud) {
  PointCloudType::Ptr raw_cloud(new PointCloudType());

  pcl::fromROSMsg(*msg, *raw_cloud);

  PointCloudType::Ptr denoised_cloud(new PointCloudType());
  denoiseCloud(raw_cloud, denoised_cloud);

  motionCompensate(denoised_cloud, out_cloud);
}

void SensorPreprocess::denoiseCloud(const PointCloudType::Ptr &in_cloud,
                                    PointCloudType::Ptr &out_cloud) {
  // Crop points around the vehicle to avoid matching dynamic objects (e.g. ego
  // body, adjacent cars) but keep the ground points (where p.z <=
  // crop_vehicle_z_)
  PointCloudType::Ptr cropped_cloud(new PointCloudType());
  cropped_cloud->points.reserve(in_cloud->size());

  for (const auto &p : in_cloud->points) {
    if (std::abs(p.x) < crop_vehicle_x_ && std::abs(p.y) < crop_vehicle_y_ &&
        p.z > crop_vehicle_z_) {
      continue;
    }
    cropped_cloud->points.push_back(p);
  }
  cropped_cloud->width = cropped_cloud->points.size();
  cropped_cloud->height = 1;
  cropped_cloud->is_dense = true;

  // Voxel grid filtering to downsample and remove noise
  downSizeFilter_.setInputCloud(cropped_cloud);
  downSizeFilter_.filter(*out_cloud);
}

void SensorPreprocess::motionCompensate(const PointCloudType::Ptr &in_cloud,
                                        PointCloudType::Ptr &out_cloud) {
  // Implement deskew logic using IMU preintegration or constant velocity model
  // For now, pass through
  *out_cloud = *in_cloud;
}

} // namespace lidar_scan_match_c
