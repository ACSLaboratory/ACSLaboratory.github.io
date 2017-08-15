---
title: ROS Kinetic Installation
category: ROS
order: 2
---

## Introduction

The following guide will show you how to install ROS Kinetic LTS (Long-Term Support) and other necessary packages for the Raspberry Pi on the Pheeno. The installation differs depending on the operating system that you choose to use with the Raspberry Pi. For an easier installation, we recommend using Ubuntu Mate ARM edition `.iso`. The guide will be for Raspbian Jessie, but we do provide some installation suggestions for those installing ROS with Ubuntu Mate.

**NOTE:** The latter part of this guide are required package installations needed for our `pheeno_ros` ROS package. Once ROS Kinetic is installed on the Raspberry Pi, you will need to install those packages on whichever OS you chose to use on the Pi.

## Installing ROS Kinetic on Ubuntu Mate

The guide [here](http://wiki.ros.org/kinetic/Installation/Ubuntu) provides the details for installation on any Ubuntu system. That is how easy it is if you use Ubuntu Mate! One suggestion we would like to make is that you only install the *ROS-Base* (`ros-kinetic-ros-base`) and **NOT** the *Desktop-Full Install* (`ros-kinetic-desktop-full` or `ros-kinetic-desktop`). The Raspberry Pi is not powerful enough to run many of the GUI tools provided in the desktop installations (and it is a waste of space). Once you are done installing ROS Kinetic, skip to the [Installing Other Necessary Packages]({{ site.baseurl }}/ros/other-necessary-ros-packages#getting-started) section.

## Installing ROS Kinetic on Raspbian Jessie

This information used to derive is from the official documentation located [here](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi). If you would like some more information than what is provided in this guide, we ask that you refer to the official documentation.

### Setup ROS Repositories

First, we would like to pull packages from the official ROS repository, so we must add ROS to our list of Debian repositories.

```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```

From now on, updating and upgrading commands will result in listings and updates for ROS packages. We will do an initial update of the system with the newly added ROS source.

```bash
$ sudo apt-get update
$ sudo apt-get upgrade
```

### Install Bootstrap Dependencies

ROS requires some necessary Python packages. The first command will download Python specific packages that are good for development. As an aside, the Python packages we will be downloading are fantastic and should be learned irrespective of using ROS. Once those packages are installed, we will use pip, a Python package manager, to install ROS specific Python packages.

```bash
$ sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake
```

### Initialize rosdep

One of the previous packages installed called `rosdep` is a ROS package installer capable of fulfilling dependencies for many packages. Once installed from `pip`, we need to initialize it as such.

```bash
$ sudo rosdep init
$ rosdep update
```

### Create a catkin Workspace

To begin the ROS installation, we need to create a workspace to build everything. In the main directory, we will create `ros_catkin_ws` to begin building ROS onto the Raspberry Pi.

```bash
$ mkdir -p ~/ros_catkin_ws
$ cd ~/ros_catkin_ws
```

To begin building, we will create a ROS installation file containing all the files and dependencies needed. The Raspberry Pi is underpowered and won't really run the full desktop suite effectively. Therefore, we will just install the `ros_comm` package, which contains ROS, build, and basic communication libraries. `wstool`, a command line tool for maintaining workspaces, will then initialize a workspace around our `rosinstall` file.

```bash
$ rosinstall_generator ros_comm --rosdistro kinetic --deps --wet-only --tar > kinetic-ros_comm-wet.rosinstall
$ wstool init src kinetic-ros_comm-wet.rosinstall
```

### Installing Unavailable Dependencies

Since not all dependencies are available in the Debian (jessie) repos, we need to build some from source using `dpkg`. Unlike the ROS Indigo installation, `collada_urdf` is the only Unavailable dependency ROS Kinetic needs (but it seems that a source install is required due to runtime failure from the maintained Debian package).

```bash
$ mkdir -p ~/ros_catkin_ws/external_src
$ cd ~/ros_catkin_ws/external_src
$ wget http://sourceforge.net/projects/assimp/files/assimp-3.1/assimp-3.1.1_no_test_models.zip/download -O assimp-3.1.1_no_test_models.zip
$ unzip assimp-3.1.1_no_test_models.zip
$ cd assimp-3.1.1
$ cmake .
$ make -j2
$ sudo make install -j2
```

### Resolving Dependencies with rosdep

The remaining dependencies can just be installed with `rosdep` as such:

```bash
$ cd ~/ros_catkin_ws
$ rosdep install -y --from-paths src --ignore-src --rosdistro kinetic -r --os=debian:jessie
```

### Building the catkin Workspace

Once all our dependencies are resolved, we can finally install ROS on our Pi as shown below. I do want users of this guide to note the use of `-j2` at the end of future code snippits. This flag tells the compiler to use two cores instead of the default four. From my experience, not putting this flag resulted in the Pi overheating and the OS freezing. Therefore, our guide will continue using the flag at the end of installation directions to prevent future crashes.

```bash
$ sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/kinetic -j2
```

After the installation completes, we can source the installation as such:

```bash
$ source /opt/ros/kinetic/setup.bash
```

But to make it permanent, you should use this!

```bash
$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
```

**Congratulations!** ROS Kinetic is now installed on your Pi!

Next, we need to install some other packages for the `pheeno_ros` package to work. Directions to do so are found [here]({{ site.baseurl }}/ros/other-necessary-ros-packages#getting-started).
