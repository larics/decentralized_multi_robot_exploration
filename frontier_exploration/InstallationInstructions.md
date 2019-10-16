# Installation Instructions

## Configure workspace

This instructions consider you have [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) installed. Following dependencies have to be installed before configuring the workspace:

```sudo apt-get install python-wstool python-catkin-tools libssh2-1-dev unzip libyaml-cpp0.5v5 libblas-dev liblapack-dev```

Next, initialize workspace using catkin tools:

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin config --extend /opt/ros/melodic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
cd ~/catkin_ws
catkin build
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Install required simulation packages

If you don't have git you can install it with:
```sudo apt-get install git```

Next, clone and checkout following packages in `src` folder:

```
cd ~/catkin_ws/src
git clone https://github.com/larics/decentralized_multi_robot_exploration
```

Before you build, install following dependencies:

CARTOGRAPHER AND GOOGLE CARTOGRAPHER:

```
sudo apt-get install google-mock libboost-all-dev libcairo2-dev libcurl4-openssl-dev libeigen3-dev libgflags-dev   libgoogle-glog-dev liblua5.2-dev libsuitesparse-dev python-sphinx

```
FRONTIER_EXPLORATION:

```
sudo apt-get install python-opencv
sudo apt-get install python-numpy
sudo apt-get install python-scikits-learn
```

Finally, build everything:

```
cd ~/catkin_ws
catkin build
```
