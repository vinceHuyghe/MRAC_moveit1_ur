ARG ROSDISTRO=noetic

FROM ros:$ROSDISTRO-robot as base

ARG ROSDISTRO

# Set environment variables
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive

ENV TERM xterm-256color

# Install tools
RUN apt-get update && apt-get install -y --no-install-recommends\
    ssh \
    git \
    curl \
    terminator \
    python3-pip \
    python3-flake8 \
    build-essential \
    cmake \
    iputils-ping \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install -U --no-cache-dir\
    argcomplete \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-return \
    flake8-length \
    flake8-todo \
    flake8-quotes \
    pycodestyle \
    autopep8 \
    numpy \
    open3d \
    pyquaternion \
    python-dateutil

# Intall ROS pkgs and dependencies
RUN apt-get update && apt-get install -y --no-install-recommends\
    pkg-config \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall-generator \
    python3-vcstool \
    python3-pykdl \
    ros-$ROSDISTRO-rqt \
    ros-$ROSDISTRO-rqt-action \
    ros-$ROSDISTRO-rqt-console \
    ros-$ROSDISTRO-rqt-service-caller \
    ros-$ROSDISTRO-rqt-graph \
    ros-$ROSDISTRO-rqt-topic \
    ros-$ROSDISTRO-rqt-top \
    ros-$ROSDISTRO-rqt-reconfigure \
    ros-$ROSDISTRO-rqt-joint-trajectory-controller \
    ros-$ROSDISTRO-rqt-tf-tree \
    ros-$ROSDISTRO-rqt-image-view \
    ros-$ROSDISTRO-rqt-robot-plugins \
    ros-$ROSDISTRO-moveit \
    ros-$ROSDISTRO-trac-ik-kinematics-plugin \
    ros-$ROSDISTRO-ros-industrial-cmake-boilerplate \ 
    ros-$ROSDISTRO-urdf-parser-plugin \
    ros-$ROSDISTRO-urdfdom-py \
    ros-$ROSDISTRO-kdl-parser-py \
    # UR dependencies
    ros-$ROSDISTRO-ur-client-library \
    ros-$ROSDISTRO-ur-msgs \
    ros-$ROSDISTRO-controller-manager \
    ros-$ROSDISTRO-pass-through-controllers \
    ros-$ROSDISTRO-force-torque-sensor-controller \
    ros-$ROSDISTRO-industrial-robot-status-interface \
    ros-$ROSDISTRO-industrial-robot-status-controller \
    ros-$ROSDISTRO-joint-state-controller \
    ros-$ROSDISTRO-joint-trajectory-controller \
    ros-$ROSDISTRO-cartesian-trajectory-controller \
    ros-$ROSDISTRO-scaled-joint-trajectory-controller \
    ros-$ROSDISTRO-speed-scaling-interface \
    ros-$ROSDISTRO-speed-scaling-state-controller \
    ros-$ROSDISTRO-velocity-controllers \
    ros-$ROSDISTRO-effort-controllers \
    ros-$ROSDISTRO-kdl-parser \
    # RQT joint trajectory plot dependencies
    ros-$ROSDISTRO-roslint \
    ros-$ROSDISTRO-rqt-gui \
    ros-$ROSDISTRO-rqt-gui-py \
    ros-$ROSDISTRO-rqt-py-common \
    python3-matplotlib \
    # roslibpy
    ros-$ROS_DISTRO-rosbridge-suite \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Add nice permision to container
RUN echo "* - nice -15" >> /etc/security/limits.conf

# Create workspace
RUN mkdir -p dev_ws/src
WORKDIR /dev_ws/src

RUN ["/bin/bash", "-c", "git clone https://github.com/tork-a/rqt_joint_trajectory_plot.git "]

# Stomp motion planning
# ----------------------------------------
FROM base as stomp

RUN ["/bin/bash", "-c", "git clone https://github.com/stevengj/nlopt.git &&\
    cd nlopt &&\
    mkdir build &&\
    cd build &&\
    cmake .. &&\
    make &&\
    make install"]

WORKDIR /dev_ws/src
RUN ["/bin/bash", "-c", "git clone https://github.com/ros-industrial/stomp.git &&\
    git clone https://github.com/ros-industrial/stomp_ros.git"]

# Realsense camera
# ----------------------------------------
FROM base as realsense
# FROM stomp as realsense

RUN apt-get update && apt-get install -y --no-install-recommends\
    software-properties-common \
    ros-$ROS_DISTRO-ddynamic-reconfigure \
    ros-$ROS_DISTRO-rgbd-launch \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN apt-key adv \
    --keyserver keyserver.ubuntu.com \
    --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
    apt-key adv \
    --keyserver hkp://keyserver.ubuntu.com:80 \
    --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
    && add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u \
    && apt-get install -y --install-recommends \
    librealsense2-dkms librealsense2-utils \
    librealsense2-dev librealsense2-dbg

WORKDIR /dev_ws/src
RUN git clone https://github.com/rjwb1/realsense-ros.git


# Astra camera
# ----------------------------------------
FROM base as astra
# FROM realsense as astra
# FROM stomp as astra

RUN apt-get update && apt-get install -y --no-install-recommends\
    libgoogle-glog-dev \
    libusb-1.0-0-dev \
    libeigen3-dev \   
    libgflags-dev \
    ros-$ROS_DISTRO-ddynamic-reconfigure \
    ros-$ROS_DISTRO-image-geometry \
    ros-$ROS_DISTRO-camera-info-manager \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-image-publisher \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN ["/bin/bash", "-c", "git clone https://github.com/libuvc/libuvc.git &&\
    cd libuvc &&\
    mkdir build && cd build &&\
    cmake .. && make -j4 &&\
    sudo make install &&\
    sudo ldconfig"]

WORKDIR /dev_ws/src
RUN ["/bin/bash", "-c", "git clone https://github.com/orbbec/ros_astra_camera.git"]


# Bash final stage
# ----------------------------------------
FROM realsense as bash
ARG ROSDISTRO

# Copy local pkg's from host to container
COPY . /dev_ws/src/

# Build catkin workspace
WORKDIR /dev_ws
RUN ["/bin/bash", "-c", "source /opt/ros/$ROSDISTRO/setup.bash &&\
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release &&\
    catkin build \
    "]

# Copy and make setup script executable
COPY .docker/setup.bash /dev_ws/setup.bash
RUN ["/bin/bash", "-c", " chmod 777 /dev_ws/setup.bash"]

# Copy and make entrypoint script executable
RUN ["/bin/bash", "-c", "cp /dev_ws/src/.docker/entrypoint.sh / && chmod 777 /entrypoint.sh"]
ENTRYPOINT [ "/entrypoint.sh" ]

# Start bash shell
CMD ["bash"]


# Zsh final stage
# ----------------------------------------
FROM realsense as zsh
ARG ROSDISTRO

# install zsh and remove apt cache
RUN apt-get update && apt-get install -y \
    zsh \
    && rm -rf /var/lib/apt/lists/*

# Copy local pkg's from host to container
COPY . /dev_ws/src/

WORKDIR /dev_ws
RUN ["/bin/bash", "-c", "source /opt/ros/$ROSDISTRO/setup.bash &&\
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release &&\
    catkin build \
    "]

# Set shell to zsh
SHELL ["zsh", "chsh -s ~/.zshrc" ]

# Copy and make setup script executable
COPY .docker/setup.zsh /dev_ws/setup.zsh
RUN ["/bin/bash", "-c", " chmod 777 /dev_ws/setup.zsh"]

# Copy and make entrypoint script executable
RUN ["/bin/bash", "-c", "cp /dev_ws/src/.docker/entrypoint_zsh.sh / && chmod 777 /entrypoint_zsh.sh"]
ENTRYPOINT ["zsh", "/entrypoint_zsh.sh" ]

# start ZSH shell
CMD ["zsh"]