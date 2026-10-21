# LiDAR Scan Match C++

## Docker

```bash
# Build Docker
bash ./docker/build_docker.sh
# Run Docker
bash ./docker/run_docker.sh
```

## Third Party
- Create Third Party
```bash
cd /root/catkin_ws/src
bash create_third_party.sh
```
- Update Third Party
```bash
cd /root/catkin_ws/src
bash update_third_party.sh
```

## Build
```bash
bash build.sh
```

---

## How to Run

All operating modes and options are configured inside **`config/params.yaml`**.  
To run the node:
```bash
# Default (with RViz GUI)
bash run.sh
```
*(Or inside Docker: `source /root/catkin_ws/devel/setup.bash && roslaunch lidar_scan_match_c run.launch`)*

---

### Mode 1: Online Mode (Live Sensors or `rosbag play`)

In this mode, the node runs as a standard ROS subscriber listening to live sensor topics (`/points_raw`, `/gps/fix`, `/imu_combined`).

1. In **`config/params.yaml`**, leave `bag_file` empty:
```yaml
# Bag Playback Configuration
bag_file: ""                            # Leave empty "" for online live topic mode
```

2. Start the SLAM node:
```bash
bash run.sh
```

3. In a separate terminal, stream your sensor topics or play your bag:
```bash
rosbag play your_dataset.bag
```

---

### Mode 2: Offline Mode (Direct Bag File Reader)

In this mode, the node reads the `.bag` file directly from disk using `rosbag::Bag`.  
**Advantages:**
- Runs faster than real-time (no wall-clock delay).
- No message drops or subscriber buffer overflows.
- Computes exact SLAM processing time and speedup factor.
- Automatically saves trajectory to CSV and terminates when finished (if `exit_on_finish: true`).

1. In **`config/params.yaml`**, specify the bag file path and settings:
```yaml
# Bag Playback Configuration
bag_file: "/root/catkin_ws/data/data_collection/20260918_rec01/merged_sep.bag" # Path to .bag file
bag_start_time: 0.0                     # Start offset in seconds (e.g., 30.0), 0.0 for start of bag
bag_duration: -1.0                      # Processing duration in seconds (-1.0 for the whole bag)
exit_on_finish: true                    # Set true to auto-shutdown when bag finishes (great for headless runs)
```

2. Run the node:
```bash
bash run.sh
```

3. When the bag completes, a summary report is printed:
```text
=================================================
[Bag] Finished processing 4980 LiDAR frames!
[Bag] Bag duration: 497.80s | Wall time: 145.89s | Speedup: 1.22x real-time
[Bag] Total computation time: 138.40s (94.9% CPU)
=================================================
[Bag] Saving trajectory to /root/catkin_ws/data/data_collection/output/tmp/ego_traj_map.csv...
[Output] Successfully saved trajectory points to CSV.
```

---

## Configuration Parameter Reference (`config/params.yaml`)

```yaml
# Bag Playback Configuration
bag_file: ""                            # string: path to .bag file (leave "" for live subscriber mode)
bag_start_time: 0.0                     # unit: sec (start offset from bag beginning, e.g. 30.0)
bag_duration: -1.0                      # unit: sec (duration to process, -1.0 for entire bag)
exit_on_finish: false                   # bool: automatically terminate ROS node when bag finishes

# ROS Topics
lidar_topic: "/points_raw"              # sensor_msgs/PointCloud2
gps_topic: "/gps/fix"                   # sensor_msgs/NavSatFix
imu_topic: "/imu_combined"              # sensor_msgs/Imu

# HD Map Settings
hd_map_directory: "/root/catkin_ws/data/BaoGao_HDmap_pcd/"
map_tile_size: 50.0                    # unit: meter

# Extrinsics
extrinsicTrans: [0.627273, 0.372772, 0.986067]
extrinsicRot: [ 0.999879, -0.010550, -0.011447,
                0.010116, 0.999253, -0.037311,
                0.011832, 0.037191, 0.999238]
extrinsicGpsTrans: [0.16, 0.64, 0.59]

# Small-GICP
use_vgicp: true                         # true for high-speed voxel GICP, false for standard GICP
icp_max_iterations: 100                 # unit: count
hd_map_match_interval: 2                # unit: Hz (target frequency for Scan-to-HDMap match)

# Output
out_trajectory_path: "/root/catkin_ws/src/lidar_scan_match_c/output/2026092302/ego_traj.csv"
out_save: true                          # bool: whether to save CSV on exit
out_hz: 10                              # unit: Hz (Trajectory recording frequency, locked to LiDAR timestamps)

# Trajectory Output Reference Center (Lever Arm & Boresight relative to IMU)
# Note: The output trajectory center is currently at the IMU center by default.
# Users can shift/rotate the trajectory reference center to another point (e.g. rear axle center, base_link) using:
#   out_la: Lever arm translation [x, y, z] in meters (relative to IMU body frame: X-Front, Y-Left, Z-Up)
#   out_bs: Boresight angular offset [roll, pitch, yaw] in radians (relative to IMU body frame)
out_la: [0.0, 0.0, 0.0]                 # unit: meter [x, y, z] (Lever arm offset from IMU center)
out_bs: [0.0, 0.0, 0.0]                 # unit: rad [roll, pitch, yaw] (Boresight rotation offset from IMU center)
```

---

## Trajectory Output Format (`ego_traj.csv`)

The trajectory CSV contains the following columns:
```csv
time,lat,lon,h,twd97x,twd97y,twd97z,roll_rad,pitch_rad,yaw_rad,source
```

- **`time`**: Integer milliseconds (`ms`) based directly on the **LiDAR data timestamp** (`header.stamp`). Each row corresponds to a LiDAR scan match with zero interpolation delay.
- **`lat, lon, h`**: WGS-84 geographic coordinates.
- **`twd97x, twd97y, twd97z`**: Taiwan TWD97 projected coordinates in meters.
- **`roll_rad, pitch_rad, yaw_rad`**: Vehicle body orientation in radians ($X=\text{Front}, Y=\text{Left}, Z=\text{Up}$).
- **Reference Center**: By default, the trajectory position and attitude correspond to the **IMU center**. Adjust `out_la` and `out_bs` in `config/params.yaml` to shift the reference point to any vehicle frame (e.g., rear axle center or vehicle center of gravity).
