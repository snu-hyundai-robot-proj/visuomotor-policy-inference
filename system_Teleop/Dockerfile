# Teleop System Dockerfile
# ROS 2 Humble based image with all dependencies

FROM osrf/ros:humble-desktop

# Set working directory
WORKDIR /workspace

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV TELEOP_IMAGE_REV=vision-no-zivid-20260602

SHELL ["/bin/bash", "-c"]

# Install system dependencies
RUN apt-get update && apt-get install -y \
    # Build tools
    build-essential \
    cmake \
    ninja-build \
    git \
    wget \
    curl \
    vim \
    # ROS tools
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-rosinstall-generator \
    python3-pip \
    python3-dev \
    python3-setuptools \
    python3-wheel \
    # ROS message/control packages used by the teleop nodes + our drive scripts
    ros-humble-control-msgs \
    # ros2_control stack — DG5F hand runs controller_manager + joint_state_broadcaster +
    # pid_controller + force_torque_sensor_broadcaster (was a manual install, wiped on rebuild)
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    # Development tools
    clang-format \
    # Vision and image processing
    libopencv-dev \
    python3-opencv \
    libopenexr-dev \
    libopenvdb-dev \
    # 3D processing
    libboost-all-dev \
    # Threading and parallel
    libtbb-dev \
    # Additional dependencies
    libglvnd0 \
    libgl1-mesa-glx \
    libx11-6 \
    libxext6 \
    libxrender-dev \
    clinfo \
    ocl-icd-libopencl1 \
    ocl-icd-opencl-dev \
    intel-opencl-icd \
    && rm -rf /var/lib/apt/lists/*

# NVIDIA Container Toolkit mounts the host driver library, but some base images
# do not provide an OpenCL ICD vendor file. This file lets ocl-icd discover the
# mounted NVIDIA OpenCL implementation when the container is started with GPUs.
RUN mkdir -p /etc/OpenCL/vendors && \
    echo "libnvidia-opencl.so.1" > /etc/OpenCL/vendors/nvidia.icd

# Install Python runtime dependencies used by the Vision node.
RUN python3 -m pip install --no-cache-dir --ignore-installed \
    torch torchvision torchaudio \
    --index-url https://download.pytorch.org/whl/cpu && \
    python3 -m pip install --no-cache-dir \
    "numpy<2" \
    scipy \
    scikit-image \
    pillow \
    open3d \
    pyyaml \
    tqdm \
    rfdetr \
    pyrealsense2 \
    pyserial \
    requests

# ROS 2 Humble's ament_python/colcon --symlink-install flow expects the older
# setuptools behavior. Keep this after runtime pip installs, since packages such
# as torch may pull a newer setuptools as a build dependency.
RUN python3 -m pip install --no-cache-dir "numpy<2" "setuptools==58.2.0"

# Zivid SDK + Python binding — the Vision node imports `zivid`. The zivid python wheel
# is built from source via scikit-build and needs `conan` to resolve its C++ deps.
# (Previously installed manually in the container; baked in here so it survives rebuilds.)
RUN cd /tmp && \
    wget -q https://downloads.zivid.com/sdk/releases/2.17.2+440b2367-1/u22/amd64/zivid_2.17.2+440b2367-1_amd64.deb -O zivid.deb && \
    apt-get update && apt-get install -y --no-install-recommends ./zivid.deb && \
    rm -f zivid.deb && rm -rf /var/lib/apt/lists/* && \
    python3 -m pip install --no-cache-dir conan && \
    python3 -m pip install --no-cache-dir zivid==2.17.2 && \
    python3 -c "import zivid; print('zivid', zivid.__version__)"

# MoveIt 2 — move_group, OMPL/Pilz planners, moveit_msgs, kinematics. Used ONLY by the
# standalone (non-VLA) perception->plan->execute path; the teleop/VLA flow does not need it.
# Added as a late layer so the torch/zivid layers above stay cached on rebuild.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-moveit \
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-planners-ompl \
    ros-humble-moveit-kinematics \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep if the base image has not done it already.
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi && \
    rosdep update

# Setup environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash" >> ~/.bashrc

# Expose ports (adjust as needed for your application)
EXPOSE 8080 5900

# Default command
CMD ["/bin/bash"]
