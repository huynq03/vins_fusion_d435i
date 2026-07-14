FROM ros:noetic-perception

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Không cài ros-noetic-librealsense2
# Không cài ros-noetic-realsense2-camera
# Hai thằng đó sẽ build source riêng để tránh lỗi IMU.

RUN apt-get update && apt-get install -y \
    git curl ca-certificates wget \
    build-essential cmake pkg-config \
    python3-catkin-tools python3-rosdep python3-rosinstall python3-vcstool \
    python3-pip \
    libssl-dev libusb-1.0-0-dev libudev-dev \
    libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
    libatlas-base-dev libeigen3-dev libgoogle-glog-dev libsuitesparse-dev libceres-dev \
    libopencv-dev libboost-all-dev libyaml-cpp-dev \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-message-filters \
    ros-noetic-tf \
    ros-noetic-rviz \
    ros-noetic-rqt-image-view \
    ros-noetic-rqt-reconfigure \
    ros-noetic-ddynamic-reconfigure \
    ros-noetic-diagnostic-updater \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true

RUN mkdir -p /opt/librealsense
RUN mkdir -p /opt/rs_ros_ws

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "if [ -f /opt/rs_ros_ws/devel/setup.bash ]; then source /opt/rs_ros_ws/devel/setup.bash; fi" >> /root/.bashrc
RUN echo "if [ -f /work/catkin_ws/devel/setup.bash ]; then source /work/catkin_ws/devel/setup.bash; fi" >> /root/.bashrc
RUN echo "export PATH=/opt/librealsense/bin:\$PATH" >> /root/.bashrc
RUN echo "export LD_LIBRARY_PATH=/opt/librealsense/lib:\$LD_LIBRARY_PATH" >> /root/.bashrc

WORKDIR /work

CMD ["/bin/bash"]
