# Use the official ROS 2 Humble base image
FROM ros:humble

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# --- Core tools, build deps, and GUI (X11/Qt) libs so rviz2/gz sim can be
# forwarded to an XLaunch server on the host ---
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    lsb-release \
    gnupg \
    pkg-config \
    nano \
    gdb \
    libxml2-utils \
    ros-humble-rviz2 \
    ros-humble-cv-bridge \
    ros-humble-message-filters \
    libopencv-dev \
    python3-opencv \
    libx11-xcb1 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render0 \
    libxcb-shape0 \
    libxcb-shm0 \
    libxcb-sync1 \
    libxcb-util1 \
    libxcb-xfixes0 \
    libxkbcommon-x11-0 \
    qtbase5-dev \
    qtchooser \
    qt5-qmake \
    qtbase5-dev-tools \
    x11-apps \
    x11-utils \
    && rm -rf /var/lib/apt/lists/*

# --- VO/VIO estimator deps: Eigen3, OpenCV (above), Ceres ---
RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    libceres-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libsuitesparse-dev \
    && rm -rf /var/lib/apt/lists/*

# --- DBoW3: not packaged for apt, build + install from source.
# Produces the DBoW3Config.cmake that sensor_pkg's
# find_package(DBoW3 REQUIRED) expects (DBoW3_INCLUDE_DIRS / DBoW3_LIBS). ---
RUN git clone --depth 1 https://github.com/rmsalinas/DBow3.git /opt/DBow3 \
    && cmake -S /opt/DBow3 -B /opt/DBow3/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /opt/DBow3/build -j"$(nproc)" \
    && cmake --install /opt/DBow3/build \
    && ldconfig \
    && rm -rf /opt/DBow3/build

# --- Gazebo Harmonic (gz sim) for sensor_pkg/sim, via ROS 2's non-official
# Humble+Harmonic packages. NOTE: this conflicts with the officially
# supported ros-humble-ros-gz (Fortress) -- do not install both. ---
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update && apt-get install -y \
        gz-harmonic \
        ros-humble-ros-gzharmonic \
    && rm -rf /var/lib/apt/lists/*

# Set up the ROS workspace (src is bind-mounted here at `docker run` time,
# e.g. -v ".../euroc_mav:/root/ros2_ws/src")
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws

# Create and activate Python virtual environment + install Python deps
# (pinned versions for colcon/ROS 2 Humble tooling compatibility)
RUN python3 -m venv /root/ros2_venv && \
    /root/ros2_venv/bin/pip install --upgrade pip && \
    /root/ros2_venv/bin/pip install \
        setuptools==58.2.0 \
        pyyaml \
        empy==3.3.4 \
        catkin_pkg \
        lark-parser \
        numpy \
        matplotlib

# Install rosbags/bagpy for bag post-processing (use venv pip)
RUN /root/ros2_venv/bin/pip install rosbags bagpy

# Auto-source ROS and venv for interactive sessions (gz-harmonic itself needs
# no sourcing -- its "gz" CLI is installed directly on PATH by apt)
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /root/ros2_venv/bin/activate" >> ~/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash 2>/dev/null || true" >> ~/.bashrc

# Default shell
CMD ["/bin/bash"]
