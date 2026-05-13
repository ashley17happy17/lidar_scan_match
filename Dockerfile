FROM osrf/ros:noetic-desktop-full

# Avoid interactive prompts during apt installations
ENV DEBIAN_FRONTEND=noninteractive

# Switch to Taiwan mirror for faster and more reliable downloads
RUN sed -i 's/archive.ubuntu.com/tw.archive.ubuntu.com/g' /etc/apt/sources.list && \
    sed -i 's/security.ubuntu.com/tw.archive.ubuntu.com/g' /etc/apt/sources.list

# Update apt and install essential build tools and common Lidar SLAM dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common && \
    add-apt-repository universe && \
    apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    unzip \
    vim \
    htop \
    nano \
    clang \
    libc++-dev \
    libc++abi-dev \
    libpdal-dev \
    pdal \
    libpcl-dev \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
    ros-noetic-foxglove-bridge \
    libopencv-dev \
    ros-noetic-cv-bridge \
    libceres-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libsuitesparse-dev \
    libtbb-dev \
    libboost-all-dev \
    python3-catkin-tools \
    python3-osrf-pycommon \
    python3-pip \
    mesa-utils \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    && rm -rf /var/lib/apt/lists/*

# Note: Eigen3 is usually installed along with PCL, but we can make sure it's linked correctly.
# If you need a more recent version of Eigen, you can build it from source here.

# Install Nanoflann from source (header-only, but cmake install puts it in default include paths)
RUN cd /tmp && \
    git clone https://github.com/jlblancoc/nanoflann.git && \
    cd nanoflann && \
    mkdir build && cd build && \
    cmake -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
    make install && \
    cd / && rm -rf /tmp/nanoflann

# Install GTSAM from source (often required for Lidar SLAM like LIO-SAM)
RUN cd /tmp && \
    git clone https://github.com/borglab/gtsam.git && \
    cd gtsam && \
    git checkout 4.2.0 && \
    mkdir build && cd build && \
    cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    cd / && rm -rf /tmp/gtsam

# Install Open3D (Python API and C++ Pre-compiled Development Library)
RUN pip3 install --no-cache-dir open3d && \
    cd /tmp && \
    wget https://github.com/isl-org/Open3D/releases/download/v0.19.0/open3d-devel-linux-x86_64-cxx11-abi-0.19.0.tar.xz && \
    tar -xf open3d-devel-linux-x86_64-cxx11-abi-0.19.0.tar.xz && \
    cp -r open3d-devel-linux-x86_64-cxx11-abi-0.19.0/* /usr/local/ && \
    rm -rf /tmp/open3d-devel*

# Create a workspace
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws

# Setup environment variables
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
ENV LIBGL_ALWAYS_SOFTWARE=1

RUN echo "umask 000" >> /root/.bashrc

# Default entry CMD
CMD ["bash"]

