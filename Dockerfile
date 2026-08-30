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
    python3-pip python3-matplotlib \
    geographiclib-tools \
    libssl-dev libusb-1.0-0-dev libudev-dev \
    libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
    libatlas-base-dev libeigen3-dev libgoogle-glog-dev libsuitesparse-dev libceres-dev \
    libopencv-dev libboost-all-dev libyaml-cpp-dev \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-message-filters \
    ros-noetic-mavros \
    ros-noetic-mavros-extras \
    ros-noetic-mavros-msgs \
    ros-noetic-tf \
    ros-noetic-rviz \
    ros-noetic-rqt-image-view \
    ros-noetic-rqt-reconfigure \
    ros-noetic-ddynamic-reconfigure \
    ros-noetic-diagnostic-updater \
    && rm -rf /var/lib/apt/lists/*

RUN /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh

RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi

RUN mkdir -p /opt/librealsense
RUN mkdir -p /opt/rs_ros_ws

ARG HOST_UID=1000
ARG HOST_GID=1000

RUN groupadd --gid "${HOST_GID}" hann \
    && useradd --uid "${HOST_UID}" --gid "${HOST_GID}" --create-home --shell /bin/bash hann \
    && mkdir -p /home/hann/vins_fusion_d435i_local \
    && chown -R hann:hann /home/hann /opt/librealsense /opt/rs_ros_ws \
    && ln -s /home/hann/vins_fusion_d435i_local /work

RUN echo "source /opt/ros/noetic/setup.bash" >> /home/hann/.bashrc
RUN echo "if [ -f /opt/rs_ros_ws/devel/setup.bash ]; then source /opt/rs_ros_ws/devel/setup.bash; fi" >> /home/hann/.bashrc
RUN echo "if [ -f /home/hann/vins_fusion_d435i_local/catkin_ws/devel/setup.bash ]; then source /home/hann/vins_fusion_d435i_local/catkin_ws/devel/setup.bash; fi" >> /home/hann/.bashrc
RUN echo "export PATH=/opt/librealsense/bin:\$PATH" >> /home/hann/.bashrc
RUN echo "export LD_LIBRARY_PATH=/opt/librealsense/lib:\$LD_LIBRARY_PATH" >> /home/hann/.bashrc

USER hann
WORKDIR /home/hann/vins_fusion_d435i_local

ENV LD_LIBRARY_PATH=/home/hann/vins_fusion_d435i_local/third_party/librealsense/build/Release:/home/hann/vins_fusion_d435i_local/rs_ros_ws/devel/lib:/opt/librealsense/lib:/opt/ros/noetic/lib

CMD ["/bin/bash"]
