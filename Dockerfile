FROM osrf/ros:jazzy-desktop-full

# Suppress interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# System dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    libgtk-3-dev \
    libeigen3-dev \
    libboost-all-dev \
    python3 \
    python3-numpy \
    wget \
    unzip \
    libtbb-dev \
    libgl1-mesa-dev \
    libglew-dev \
    ffmpeg \
    libssl-dev \
    libyaml-cpp-dev \
    libusb-1.0-0-dev \
    libx11-dev \
    libtbb-dev \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    && rm -rf /var/lib/apt/lists/*

# Pangolin dependencies
RUN apt-get update && apt-get install -y \
    libglfw3-dev \
    libglew-dev \
    libpython3-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt
RUN git clone https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    mkdir build && cd build && \
    cmake .. && make -j$(nproc) && make install

# Install OpenCV 4.8.0 from source
# TODO- replace with apt install later
RUN apt-get update && apt-get install -y \
    libjpeg-dev libpng-dev libtiff-dev \
    libavcodec-dev libavformat-dev libswscale-dev \
    libv4l-dev libxvidcore-dev libx264-dev \
    libgtk-3-dev libatlas-base-dev gfortran \
    python3-dev

# Download OpenCV and contrib
WORKDIR /opt
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/4.8.0.zip && unzip opencv.zip && \
    wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.8.0.zip && unzip opencv_contrib.zip

# Build
WORKDIR /opt/opencv-4.8.0
RUN mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release \
          -D CMAKE_INSTALL_PREFIX=/usr/local \
          -D OPENCV_EXTRA_MODULES_PATH=/opt/opencv_contrib-4.8.0/modules \
          -D BUILD_EXAMPLES=OFF \
          -D BUILD_TESTS=OFF \
          -D WITH_TBB=OFF \
          .. && \
    make -j$(nproc) && make install && ldconfig

# Doing this because I am having display isses on ububtu
RUN apt-get update && apt-get install -y \
      libgl1-mesa-dri \
      libglx-mesa0 \
      mesa-utils \
      # if you do need EGL for offscreen or Vulkan paths:
      libegl-mesa0 \
    && rm -rf /var/lib/apt/lists/*

# Set workspace
WORKDIR /ORB_SLAM3_ROS2

# Copy source code (optional if you're bind mounting)
COPY . .

CMD ["/bin/bash"]