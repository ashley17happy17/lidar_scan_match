#pragma once
#include <Eigen/Dense>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>

namespace lidar_scan_match_c {

// Define common point types
using PointType = pcl::PointXYZI;
using CloudType = pcl::PointCloud<PointType>;

// Define structures for sensor measurements
struct ImuMeasurement {
  double timestamp;
  Eigen::Vector3d linear_acceleration;
  Eigen::Vector3d angular_velocity;
};

struct GpsMeasurement {
  double timestamp;
  double latitude;
  double longitude;
  double altitude;
  Eigen::Vector3d covariance_diag;
};

struct LoopClosureThreshold {
  bool enable;
  double search_radius;
  double fitness_score;
  double travel_distance;
};

struct ICPThreshold {
  double max_fitness;
  double max_trans;
  double max_rot;
};

// Utilities for LatLon translation
class LocalCartesian {
public:
  LocalCartesian();

  void Reset(double lat0, double lon0, double alt0);

  void GetTWD97(double lat, double lon, double alt, double &twdx, double &twdy,
                double &twdz);

  void GetWGS84(double twdx, double twdy, double twdz, double &lat, double &lon,
                double &alt);

  void Forward(double lat, double lon, double alt, double &x, double &y,
               double &z) const;

  bool initialized;

private:
  double lat0_, lon0_, alt0_;
};

} // namespace lidar_scan_match_c
