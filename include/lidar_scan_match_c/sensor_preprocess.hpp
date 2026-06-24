#pragma once

#include "common.h"
#include <lidar_utils/cloud_utils.hpp>
#include <pcl/filters/voxel_grid.h>
#include <vector>

namespace lidar_scan_match_c {

class SensorPreprocess {
public:
  SensorPreprocess(ros::NodeHandle &nh);

  // Processes an incoming raw point cloud
  void processCloud(const sensor_msgs::PointCloud2ConstPtr &msg,
                    CloudType::Ptr &out_cloud,
                    std::vector<double> &out_timestamps);

private:
  double leaf_size_;
  Eigen::Vector3f crop_min_bound_;
  Eigen::Vector3f crop_max_bound_;
};

} // namespace lidar_scan_match_c
