cmake_minimum_required(VERSION 3.14)
project(lidar_scan_match_c)

# Compile options
add_compile_options(-std=c++17 -O3 -Wall)

# Find ROS packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  pcl_ros
  pcl_conversions
  cv_bridge
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Find PCL
find_package(PCL REQUIRED)
# Find Ceres
find_package(Ceres REQUIRED)
# Find GTSAM
find_package(GTSAM REQUIRED)
# Find OpenCV
find_package(OpenCV REQUIRED)
# Find Eigen3
find_package(Eigen3 REQUIRED)
# Find PDAL
find_package(PDAL REQUIRED)

# small_gicp 由 third_party/lidar_utils_lib 提供（見下方 add_subdirectory），
# 已 vendor 進該函式庫內，建置不需要網路，版本也不會隨 upstream master 漂移。
# 注意：原本這裡的 set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE) 會強制整個
# 專案改成靜態連結，一併移除；CMake 未設定此變數時預設就是靜態。

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} 
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs nav_msgs geometry_msgs pcl_ros pcl_conversions cv_bridge tf2 tf2_ros tf2_geometry_msgs
  DEPENDS PCL Ceres GTSAM OpenCV Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_subdirectory(third_party/lidar_utils_lib/src)

# Library containing the main logic
add_library(${PROJECT_NAME}_lib
  src/common.cpp
  src/sensor_preprocess.cpp
  src/front_end_icp.cpp
  src/back_end_optimization.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
  gtsam
  ${OpenCV_LIBRARIES}
  ${PDAL_LIBRARIES} 
  small_gicp
  lidar_utils
)

# Node Executable
add_executable(${PROJECT_NAME}_node src/lidar_slam_node.cpp)
target_link_libraries(
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
  gtsam
  ${OpenCV_LIBRARIES}
  ${PDAL_LIBRARIES} 
  small_gicp
  lidar_utils
)

# Install rules (optional but good practice)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_lib ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
