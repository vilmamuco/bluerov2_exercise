FROM ros:iron-ros-base

# not expecting a cache mechanism like the one in buildx, the base image includes
# this config file that essentially purges the cache on every install operation
# which is why we have to remove this config to take advantage of the host's cache
# with --mount=cache further down
RUN rm /etc/apt/apt.conf.d/docker-clean

ENV DEBIAN_FRONTEND noninteractive


#--------------------------------------------Gazebo---------------------------------------------------
RUN \
  # holds the package _indexes_ (used for apt update)
  --mount=type=cache,target=/var/lib/apt/lists \
  # holds the package _contents_ (used for apt install)
  --mount=type=cache,target=/var/cache/apt/archives \
  apt update && \
  apt install -y --no-install-recommends \
  build-essential \
  keyboard-configuration \
  locales \
  libboost-all-dev \
  python3-pip \
  python3-vcstool \
  python3-rosdep \
  python-is-python3 \
  python3-colcon-common-extensions \
  tzdata \
  git \
  gnupg \
  lsb-release \
  cmake \
  wget \
  sudo \
  libgazebo11 \
  ros-iron-mavros ros-iron-mavros-extras ros-iron-mavros-msgs 


ENV NVIDIA_VISIBLE_DEVICES \
  ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
  ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Set the locale
RUN sed -i '/en_US.UTF-8/s/^# //g' /etc/locale.gen && \
  locale-gen
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# create new user
RUN adduser --disabled-password \
  --gecos '' docker

#  Add new user docker to sudo group
RUN adduser docker sudo
RUN adduser docker dialout

# Ensure sudo group users are not asked for a password when using
# sudo command by ammending sudoers file
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> \
  /etc/sudoers

# now we can set USER to the user we just created
USER docker

ENV DEBIAN_FRONTEND noninteractive

#--------------------------------------------Gazebo---------------------------------------------------
RUN ["/bin/bash","-c","sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg"]
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN sudo apt update && DEBIAN_FRONTEND=noninteractive sudo apt install -y \
  gz-garden \
  && sudo rm -rf /var/lib/apt/lists/

#-----------------------------------------Ardupilot---------------------------------------------------

WORKDIR /home/docker/
RUN git clone https://github.com/ArduPilot/ardupilot.git
WORKDIR /home/docker/ardupilot
RUN git checkout 4ae8379616
RUN git submodule update --init --recursive

# RUN git config --global --add safe.directory /ardupilot
WORKDIR /home/docker/ardupilot/Tools/environment_install/
RUN rm install-prereqs-ubuntu.sh
RUN wget https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/environment_install/install-prereqs-ubuntu.sh
WORKDIR /home/docker/ardupilot
RUN chmod +x Tools/environment_install/install-prereqs-ubuntu.sh
RUN USER=docker Tools/environment_install/install-prereqs-ubuntu.sh -y
RUN . ~/.profile

RUN sudo pip3 install -U mavproxy PyYAML

ENV PATH=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH
ENV PATH=$PATH:$HOME/docker/ardupilot/Tools/autotest
ENV PATH=/usr/lib/ccache:$PATH

WORKDIR /home/docker/ardupilot
RUN ["/bin/bash","-c","./waf configure && make sub"]
WORKDIR /home/docker/ardupilot/ArduSub

# ardupilot gazebo plugin ---------------------------------------------------------------------------
WORKDIR /home/docker/
RUN ["/bin/bash", "-c", "sudo apt update \
  && sudo apt install -y  rapidjson-dev libgz-sim7-dev  libopencv-dev protobuf-compiler \
  && sudo rm -rf /var/lib/apt/lists/"]
RUN export GZ_VERSION="garden"
RUN export GZ_DIR="libgz-sim7-dev"
#RUN git clone https://github.com/ArduPilot/ardupilot_gazebo /home/docker/ardupilot_gazebo
RUN git clone https://github.com/vilmamuco/ardupilot_gazebo /home/docker/ardupilot_gazebo
WORKDIR /home/docker/ardupilot_gazebo
#RUN git checkout add_link
RUN git checkout ros2
RUN mkdir build
WORKDIR /home/docker/ardupilot_gazebo/build
RUN cmake  .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
RUN make -j4
RUN make 
# set path of gazebo models
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/home/docker/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}
ENV GZ_SIM_RESOURCE_PATH=/home/docker/ardupilot_gazebo/models:/home/docker/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}

#-----------------------------------------ROS2 agent---------------------------------------------------
# test 
COPY --chown=docker:docker . /home/docker/bluerov2_exercise/src/bluerov2_agent
WORKDIR /home/docker/bluerov2_exercise/
RUN vcs import src < /home/docker/bluerov2_exercise/src/bluerov2_agent/bluerov2.rosinstall --recursive

RUN sudo rm -rf /etc/ros/rosdep/sources.list.d/*
RUN ["/bin/bash", "-c", "source /opt/ros/iron/setup.bash \
  && sudo apt update \
  && sudo rosdep init \
  && rosdep update \
  && sudo rosdep install --from-paths src --ignore-src -r -y \
  && sudo rm -rf /var/lib/apt/lists/"]

RUN ["/bin/bash", "-c", "sudo ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh"]
RUN ["/bin/bash", "-c", "sudo apt update \
  && sudo apt install ros-iron-image-transport ros-iron-actuator-msgs ros-iron-angles"]
ENV GZ_VERSION="garden"
RUN [ "/bin/bash","-c","export MAKEFLAGS='-j 1' \
  && source /opt/ros/iron/setup.bash \
  && colcon build --symlink-install"]
RUN sudo apt-get remove modemmanager -y

ENV GZ_SIM_RESOURCE_PATH=/home/docker/bluerov2_exercise/src/bluerov2_ignition/models:/home/docker/bluerov2_exercise/bluerov2_ignition/worlds:${GZ_SIM_RESOURCE_PATH}
ENV GZ_SIM_RESOURCE_PATH=/home/docker/bluerov2_exercise/src/remaro_worlds/models:/home/docker/bluerov2_exercise/src/remaro_worlds/worlds:${GZ_SIM_RESOURCE_PATH}


# work directory  ---------------------------------------------------------------------
WORKDIR /home/docker/bluerov2_exercise/
RUN ["/bin/bash","-c","source install/setup.bash "]
CMD  /bin/bash