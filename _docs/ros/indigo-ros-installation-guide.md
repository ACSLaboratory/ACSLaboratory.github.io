---
title: ROS Indigo Installation
category: ROS
order: 1
---

## Introduction

The following guide will show you how to install ROS Indigo LTS (Long-Term Support) and other necessary packages for the Raspberry Pi on the Pheeno.


## Installing ROS Indigo

This information used to derive is from the official documentation located [here](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi). If you would like some more information than what is provided in this guide, we ask that you refer to the official documentation.


### Setup ROS Repositories

First, we would like to pull packages from the official ROS repository, so we must add ROS to our list of Debian repositories.

```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu jessie main" > /etc/apt/sources.list.d/ros-latest.list'

$ wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
```


From now on, updating and upgrading commands will result in listings and updates for ROS packages. We will do an initial update of the system with the newly added ROS source.

```bash
$ sudo apt-get update
$ sudo apt-get upgrade
```


### Install Bootstrap Dependencies

ROS requires some necessary Python packages. The first command will download Python specific packages that are good for development. As an aside, the Python packages we will be downloading are fantastic and should be learned irrespective of using ROS. Once those packages are installed, we will use pip, a Python package manager, to install ROS specific Python packages.

```bash
$ sudo apt-get install python-pip python-setuptools python-yaml python-distribute python-docutils python-dateutil python-six
$ sudo pip install rosdep rosinstall_generator wstool rosinstall
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
$ mkdir ~/ros_catkin_ws
$ cd ~/ros_catkin_ws
```


To begin building, we will create a ROS installation file containing all the files and dependencies needed. The Raspberry Pi is underpowered and won't really run the full desktop suite effectively. Therefore, we will just install the `ros_comm` package, which contains ROS, build, and basic communication libraries. `wstool`, a command line tool for maintaining workspaces, will then initialize a workspace around our `rosinstall` file.

```bash
$ rosinstall_generator ros_comm --rosdistro indigo --deps --wet-only --exclude roslisp --tar > indigo-ros_comm-wet.rosinstall
$ wstool init src indigo-ros_comm-wet.rosinstall
```


### Installing Unavailable Dependencies

Since not all dependencies are available in the Debian (jessie) repos, we need to build some from source using `dpkg`. For just the `ros_comm` ROS installation we only need two, but we must first create a build directory.

```bash
$ mkdir ~/ros_catkin_ws/external_src
$ sudo apt-get install checkinstall cmake
$ sudo sh -c 'echo "deb-src http://mirrordirector.raspbian.org/raspbian/ testing main contrib non-free rpi" >> /etc/apt/sources.list'
$ sudo apt-get update
```

Now that the build directory is created, we begin by installing `libconsole-bridge-dev` as such:

```bash
$ cd ~/ros_catkin_ws/external_src
$ sudo apt-get build-dep console-bridge
$ apt-get source -b console-bridge
$ sudo dpkg -i libconsole-bridge0.2*.deb libconsole-bridge-dev_*.deb
```

The last dependency to install is `liblz4-dev`. The installation is shown below, but be warned that this package is known to take a long time to build and install.

```bash
$ cd ~/ros_catkin_ws/external_src
$ apt-get source -b lz4
$ sudo dpkg -i liblz4-\*.deb
```


### Resolving Dependencies with rosdep

The remaining dependencies can just be installed with `rosdep` as such:

```bash
$ cd ~/ros_catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro indigo -y -r --os=debian:jessie
```


### Building the catkin Workspace

Once all our dependencies are resolved, we can finally install ROS on our Pi as shown below. I do want users of this guide to note the use of `-j2` at the end of future code snippits. This flag tells the compiler to use two cores instead of the default four. From my experience, not putting this flag resulted in the Pi overheating and the OS freezing. Therefore, our guide will continue using the flag at the end of installation directions to prevent future crashes.

```bash
$ sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/indigo -j2
```

After the installation completes, we can source the installation as such:

```bash
$ source /opt/ros/indigo/setup.bash
```

But to make it permanent, you should use this!

```bash
$ echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
```

**Congratulations!** ROS Indigo is now installed on your Pi!

Next, we need to install some other packages for the `pheeno_ros` package to work. Directions to do so are found [here]({{ site.baseurl }}/ros/other-necessary-ros-packages#getting-started).
