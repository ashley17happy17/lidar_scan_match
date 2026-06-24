#include "lidar_scan_match_c/common.h"
#include <cmath>

namespace lidar_scan_match_c {

LocalCartesian::LocalCartesian()
    : initialized(false), lat0_(0.0), lon0_(0.0), alt0_(0.0) {}

void LocalCartesian::Reset(double lat0, double lon0, double alt0) {
  lat0_ = lat0 * M_PI / 180.0;
  lon0_ = lon0 * M_PI / 180.0;
  alt0_ = alt0;
  initialized = true;
}

void LocalCartesian::GetTWD97(double lat, double lon, double alt, double &twdx,
                              double &twdy, double &twdz) {
  const double a = 6378137.0;
  const double b = 6356752.314245;
  const double lon0 = 121.0 * M_PI / 180.0;
  const double k0 = 0.9999;
  const double dx = 250000.0;

  double e2 = 1.0 - (b * b) / (a * a);
  double e4 = e2 * e2;
  double e6 = e4 * e2;

  double e_prime_sq = (a * a - b * b) / (b * b);

  double lat_rad = lat * M_PI / 180.0;
  double lon_rad = lon * M_PI / 180.0;

  double A = (lon_rad - lon0) * std::cos(lat_rad);
  double A2 = A * A;
  double A3 = A2 * A;
  double A4 = A3 * A;
  double A5 = A4 * A;
  double A6 = A5 * A;

  double N = a / std::sqrt(1.0 - e2 * std::sin(lat_rad) * std::sin(lat_rad));
  double T = std::tan(lat_rad) * std::tan(lat_rad);
  double C = e_prime_sq * std::cos(lat_rad) * std::cos(lat_rad);

  double M =
      a * ((1.0 - e2 / 4.0 - 3.0 * e4 / 64.0 - 5.0 * e6 / 256.0) * lat_rad -
           (3.0 * e2 / 8.0 + 3.0 * e4 / 32.0 + 45.0 * e6 / 1024.0) *
               std::sin(2.0 * lat_rad) +
           (15.0 * e4 / 256.0 + 45.0 * e6 / 1024.0) * std::sin(4.0 * lat_rad) -
           (35.0 * e6 / 3072.0) * std::sin(6.0 * lat_rad));

  twdx = dx + k0 * N *
                  (A + (1.0 - T + C) * A3 / 6.0 +
                   (5.0 - 18.0 * T + T * T + 72.0 * C - 58.0 * e_prime_sq) *
                       A5 / 120.0);

  twdy = k0 *
         (M + N * std::tan(lat_rad) *
                  (A2 / 2.0 + (5.0 - T + 9.0 * C + 4.0 * C * C) * A4 / 24.0 +
                   (61.0 - 58.0 * T + T * T + 600.0 * C - 330.0 * e_prime_sq) *
                       A6 / 720.0));

  twdz = alt;
}

void LocalCartesian::GetWGS84(double twdx, double twdy, double twdz,
                              double &lat, double &lon, double &alt) {
  const double a = 6378137.0;
  const double b = 6356752.314245;
  const double lon0 = 121.0 * M_PI / 180.0;
  const double k0 = 0.9999;
  const double dx = 250000.0;

  double e2 = 1.0 - (b * b) / (a * a);
  double e_prime_sq = (a * a - b * b) / (b * b);

  double x = twdx - dx;
  double y = twdy;

  double M = y / k0;
  double mu = M / (a * (1.0 - e2 / 4.0 - 3.0 * e2 * e2 / 64.0 -
                        5.0 * std::pow(e2, 3) / 256.0));

  double e1 = (1.0 - std::sqrt(1.0 - e2)) / (1.0 + std::sqrt(1.0 - e2));

  double phi1 =
      mu +
      (3.0 * e1 / 2.0 - 27.0 * std::pow(e1, 3) / 32.0) * std::sin(2.0 * mu) +
      (21.0 * e1 * e1 / 16.0 - 55.0 * std::pow(e1, 4) / 32.0) *
          std::sin(4.0 * mu) +
      (151.0 * std::pow(e1, 3) / 96.0) * std::sin(6.0 * mu) +
      (1097.0 * std::pow(e1, 4) / 512.0) * std::sin(8.0 * mu);

  double C1 = e_prime_sq * std::pow(std::cos(phi1), 2);
  double T1 = std::pow(std::tan(phi1), 2);
  double N1 = a / std::sqrt(1.0 - e2 * std::pow(std::sin(phi1), 2));
  double R1 =
      a * (1.0 - e2) / std::pow(1.0 - e2 * std::pow(std::sin(phi1), 2), 1.5);
  double D = x / (N1 * k0);

  lat = phi1 -
        (N1 * std::tan(phi1) / R1) *
            (D * D / 2.0 -
             (5.0 + 3.0 * T1 + 10.0 * C1 - 4.0 * C1 * C1 - 9.0 * e_prime_sq) *
                 std::pow(D, 4) / 24.0 +
             (61.0 + 90.0 * T1 + 298.0 * C1 + 45.0 * T1 * T1 -
              252.0 * e_prime_sq - 3.0 * C1 * C1) *
                 std::pow(D, 6) / 720.0);

  lon = lon0 + (D - (1.0 + 2.0 * T1 + C1) * std::pow(D, 3) / 6.0 +
                (5.0 - 2.0 * C1 + 28.0 * T1 - 3.0 * C1 * C1 + 8.0 * e_prime_sq +
                 24.0 * T1 * T1) *
                    std::pow(D, 5) / 120.0) /
                   std::cos(phi1);

  lat = lat * 180.0 / M_PI;
  lon = lon * 180.0 / M_PI;
  alt = twdz;
}

void LocalCartesian::Forward(double lat, double lon, double alt, double &x,
                             double &y, double &z) const {
  if (!initialized)
    return; // Should assert
  double lat_rad = lat * M_PI / 180.0;
  double lon_rad = lon * M_PI / 180.0;

  // Simple spherical approximation for small regions
  static const double R = 6378137.0; // WGS84 equatorial radius
  double dLat = lat_rad - lat0_;
  double dLon = lon_rad - lon0_;

  x = R * dLon * std::cos(lat0_);
  y = R * dLat;
  z = alt - alt0_;
}

} // namespace lidar_scan_match_c
