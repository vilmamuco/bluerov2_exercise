
# BlueRov2 exercise for the ROS2 course 

This set of exercises serves to illustrate and consolidate the contents of the lectures.
The bluerov2.py script moves the bluerov in a square the goal is to subscribe to the sonar data and calculate the height of the walls. If we have time experiment with the different modes of operation of the bluerov and the truster commands to maintain the same depth.


## Summary
- [Computer setup](https://github.com/remaro-network/tudelft_hackathon#setup)
* [Installation](https://github.com/remaro-network/tudelft_hackathon#installation)
  - [Install prerequisites to run with docker](https://github.com/remaro-network/tudelft_hackathon#install-prerequisites-to-run-with-docker)
  - [Install locally](https://github.com/remaro-network/tudelft_hackathon#install-locally)
- [Run it with docker via CLI](https://github.com/remaro-network/tudelft_hackathon#run-it-with-docker-via-cli)
- [Run locally](https://github.com/remaro-network/tudelft_hackathon#run-it-locally)
## Setup

Tested with:
- Ubuntu 22.04
- [ROS2 Iron](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html)
- [Gazebo (Ignition) Garden](https://gazebosim.org/docs/garden/install_ubuntu)
- [ArduPilot (Sub-4.1)](https://github.com/ArduPilot/ardupilot/tree/f2af3c7ed2907be914c41d8512654a77498d3870)
- [ardupilot_gazebo plugin](https://github.com/ArduPilot/ardupilot_gazebo/tree/ignition-garden)
- [mavros2](https://github.com/mavlink/mavros)
- [remaro_world](https://github.com/vilmamuco/remaro_worlds/tree/ign-garden)
- [bluerov2_ignition](https://github.com/vilmamuco/bluerov2_ignition.git)

## Installation

There are 2 options to use this repo.
Install everything locally in your computer. This will require some effort, but when done should be easier to use.
Run everything with docker

### Install prerequisites to run with docker

- Install docker on your machine. You can find instructions [here](https://docs.docker.com/engine/install/ubuntu/)
- Allow non-root users to manage docker. Instructions [here](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user)
- Install VSCode. Instructions [here](https://code.visualstudio.com/download)
- Install [nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)(only needed if you have a nvidia GPU)

### Install locally
#### Install Gazebo Garden

Follow the [official instructions](https://gazebosim.org/docs/garden/install_ubuntu) for installing Gazebo Garden.

#### Install ROS2 Iron

Follow the [official instructions](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html) for installing ROS2 Iron.

#### Install ardusub

Instructions can be found [here](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux)

**Disclaimer:**
Problems may occur with different combinations of ArduPilot and MavROS versions.

```Bash
cd ~/
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git checkout e9f46b9
git submodule update --init --recursive
```

Note that the script used to install prerequisites available for this
version of ArduSub does not work in Ubuntu 22.04. Therefore, you need to replace them before
running ArduSub. To install the ArduPilot prerequisites, do the following.

```Bash
cd ~/ardupilot
cd Tools/environment_install/
rm install-prereqs-ubuntu.sh
wget https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/environment_install/install-prereqs-ubuntu.sh
cd ~/ardupilot
chmod +x Tools/environment_install/install-prereqs-ubuntu.sh
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

If you want to use MAC, follow [this instruction](https://ardupilot.org/dev/docs/building-setup-mac.html)

To test if the installation worked, run:

```Bash
sim_vehicle.py -v ArduSub -L RATBeach --console --map
```

Ardupilot SITL should open and a console plus a map should appear.

**Troubleshoot**
If you have problems with the install-prereqs-ubuntu.sh script try to install the dependencies manually with the following commands.

```Bash
pip3 install --user -U future lxml pymavlink MAVProxy pexpect flake8 geocoder empy dronecan pygame intelhex
```

```Bash
sudo apt-get --assume-yes install build-essential ccache g++ gawk git make wget python-is-python3 libtool libxml2-dev libxslt1-dev python3-dev python3-pip python3-setuptools python3-numpy python3-pyparsing python3-psutil xterm python3-matplotlib python3-serial python3-scipy python3-opencv libcsfml-dev libcsfml-audio2.5 libcsfml-dev libcsfml-graphics2.5 libcsfml-network2.5 libcsfml-system2.5 libcsfml-window2.5 libsfml-audio2.5 libsfml-dev libsfml-graphics2.5 libsfml-network2.5 libsfml-system2.5 libsfml-window2.5 python3-yaml libpython3-stdlib python3-wxgtk4.0 fonts-freefont-ttf libfreetype6-dev libpng16-16 libportmidi-dev libsdl-image1.2-dev libsdl-mixer1.2-dev libsdl-ttf2.0-dev libsdl1.2-dev libtool-bin g++-arm-linux-gnueabihf lcov gcovr
```

#### Install ardusub_plugin

Install dependencies:

```Bash
sudo apt install rapidjson-dev libgz-sim7-dev
```

Clone and build repo:

```Bash
cd ~/
git clone https://github.com/ArduPilot/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

Add required paths:

Assuming that you have clone the repository in `$HOME/ardupilot_gazebo`:
```bash
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
```

Reload your terminal with source ~/.bashrc

More info about the plugin can be found in the [repo](https://github.com/ArduPilot/ardupilot_gazebo/tree/ignition-garden)

#### Install workspace

Create new workspace:
```Bash
mkdir -p ~/bluerov2_exercise/src
cd ~/bluerov2_exercise/
```

Clone repos:
```Bash
wget https://raw.githubusercontent.com/remaro-network/tudelft_hackathon/ros2/hackathon.rosinstall
vcs import src < bluerov2.rosinstall --recursive
```

Add required paths:
```Bash
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/bluerov2_exercise/src/bluerov2_ignition/models:$HOME/bluerov2_exercise/src/bluerov2_ignition/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc

echo 'export GZ_SIM_RESOURCE_PATH=$HOME/bluerov2_exercise/src/remaro_worlds/models:$HOME/bluerov2_exercise/src/remaro_worlds/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
```

Before building the `ros_gz` package (one of the dependencies), you need to export the gazebo version:

```
export GZ_VERSION="garden"
```
You can also add this to your `~/.bashrc` to make this process easier.

Install deps:
```Bash
source /opt/ros/iron/setup.bash
cd ~/bluerov2_exercise/
rosdep install --from-paths src --ignore-src -r -y
```

Build project:
```Bash
cd ~/bluerov2_exercise/
colcon build --symlink-install
```

## Run it with docker via CLI

Create docker network:

```Bash
sudo docker network create ros_net
```

### Run Ignition simulation + ardupilot SITL:

If you a NVIDIA GPU:
```Bash
xhost +local:root
sudo docker run -it --rm --name ignition --net ros_net -e DISPLAY=$DISPLAY -e NO_AT_BRIDGE=1 -v /run/user/1000/at-spi/bus_0:/run/user/1000/at-spi/bus_0  -v /tmp/.X11-unix:/tmp/.X11-unix:ro --gpus all ghcr.io/remaro-network/tudelft_hackathon:nvidia bash
```

If you have an AMD GPU:
```Bash
xhost +local:root ;
sudo docker run -it --rm --name ignition --net ros_net -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro --device=/dev/dri --group-add video  ghcr.io/remaro-network/tudelft_hackathon:non-nvidia  bash
```

If you have an Intel GPU:
```Bash
xhost +local:root ;
sudo docker run -it --rm --name ignition --net ros_net -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro --device=/dev/dri:/dev/dri  ghcr.io/remaro-network/tudelft_hackathon:non-nvidia bash
```

### Development with docker via cli

Mount the directory todo

## Run it locally

### Simulation
Before running anything you need to source the workspace. With this command:

```Bash
source ~/bluerov2_exercise/install/setup.bash
```

Or you can add that to the ~/.bashrc file to prevent needing to source everytime.

```Bash
echo "source ~/bluerov2_exercise/install/setup.bash" >> ~/.bashrc
```
Don't forget to re-open your terminal after altering the `~/.bashrc` file.

In one terminal run ardusub SITL:
```Bash
./Tools/autotest/sim_vehicle.py -L RATBeach -v ArduSub --model=JSON --out=udp:0.0.0.0:14551  --console

```

In another terminal run the simulation + mavros + agent:
```Bash
ros2 launch bluerov2_agent bluerov_bringup.launch.py simulation:=true ardusub:=false mavros_url:='udp://127.0.0.1:14551'
```

## Acknowledgements

This repository is a simplified version of the REMARO Summer School Delft 2022 - Underwater robotics hackathon. For more info, please visit: <a href="https://github.com/remaro-network/tudelft_hackathon.git"> remaro-network/tudelft_hackathon