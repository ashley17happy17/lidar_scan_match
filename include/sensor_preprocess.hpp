#pragma once

#include "common.h"
#include <pcl/filters/voxel_grid.h>

namespace lidar_scan_match_c {

class SensorPreprocess {
public:
  SensorPreprocess(ros::NodeHandle &nh);

  // Processes an incoming raw point cloud
  void processCloud(const sensor_msgs::PointCloud2ConstPtr &msg,
                    CloudType::Ptr &out_cloud);

  // Matches the MATLAB lidarPreprocess: denoising
  void denoiseCloud(const CloudType::Ptr &in_cloud, CloudType::Ptr &out_cloud);

  // Motion compensation (deskew)
  void motionCompensate(const CloudType::Ptr &in_cloud,
                        CloudType::Ptr &out_cloud);

private:
  double leaf_size_;
  double crop_vehicle_x_;
  double crop_vehicle_y_;
  double crop_vehicle_z_;
  pcl::VoxelGrid<PointType> downSizeFilter_;
};

} // namespace lidar_scan_match_c
