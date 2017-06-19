---
title: ROS Installation for Pheeno
category: ROS
order: 1
---

## Introduction

Currently, Pheeno is only supported by ROS Indigo; However, ROS Kinetic support is next. The following guide will show you how to install ROS and other necessary packages for the Raspberry Pi on the Pheeno.


## Installing ROS Indigo

### Setup ROS Repositories

First, we would like to pull packages from the official ROS repository, so we must add ROS to our list of Ubuntu repositories.

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

```bash
$ mkdir ~/ros_catkin_ws/external_src
$ sudo apt-get install checkinstall cmake
$ sudo sh -c 'echo "deb-src http://mirrordirector.raspbian.org/raspbian/ testing main contrib non-free rpi" >> /etc/apt/sources.list'
$ sudo apt-get update
```

```bash
$ cd ~/ros_catkin_ws/external_src
$ sudo apt-get build-dep console-bridge
$ apt-get source -b console-bridge
$ sudo dpkg -i libconsole-bridge0.2*.deb libconsole-bridge-dev_*.deb
```

```bash
$ cd ~/ros_catkin_ws/external_src
$ apt-get source -b lz4
$ sudo dpkg -i liblz4-\*.deb
```


### Resolving Dependencies with rosdep

```bash
$ cd ~/ros_catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro indigo -y -r --os=debian:jessie
```


### Building the catkin Workspace

```bash
$ sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/indigo -j2
```

```bash
$ source /opt/ros/indigo/setup.bash
```

```bash
$ echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
```

Thats it!


## Installing Other Necessary Packages

The previous section explained the base installation of ROS Indigo on the Raspberry Pi. To use other packages is a bit tricky, but the following guide provides a convenient way of installing the packages on the Pheeno.

First, change back to the main directory.

```bash
$ cd
```


From here, we will make a directory called `catkin_ws` with a child directory called `src`.

```bash
$ mkdir -p ~/catkin_ws/src
```


Finally, we change directory to the `src` dictory and initialize the workspace.

```bash
$ cd ~/catkin_ws/src
$ catkin_init_workspace
```


This new catkin workspace will allow us to install other ROS packages.


### Getting the rosserial Package

`rosserial` is required for communication between the Raspberry Pi and the Teensy board. Our project requires three packages within the repository: `rosserial_client`, `rosserial_arduino`, and `rosserial_python`. To attain these files, we need to use `git clone` within the `src` folder.

```bash
$ cd ~/catkin_ws/src
$ git clone -b indigo-devel https://github.com/ros_drivers/rosserial.git
```


The `-b` flag indicates that we would like to use the ROS Indigo specific branch of their repository. The default branch for the project is currently only for ROS Jade.


### Getting the common_msgs Package

The base installation of ROS only comes with standard messages (`std_msgs`). To get other message types such as Twist and action messages, we need to install the `common_msgs` package. Again, we will use `git clone`.

```bash
$ cd ~/catkin_ws/src
$ git clone -b indigo-devel https://github.com/ros/common_msgs.git
```


We again pull from the ROS Indigo specific branch of the repository.


### Getting the actionlib Package

This is an optional download entirely dependent on your use case. This is typically a default package for ROS; however, the `ros_comm` installation from before does not contain it. To download, we just need to clone the `indigo-devel` branch from the official ROS GitHub repository.

```bash
$ cd ~/catkin_ws/src
$ git clone -b indigo-devel https://github.com/ros/actionlib.git
```


### Getting the cv_bridge Package

The pacakges within the vision_opencv repository contain files for interfacing ROS with OpenCV. In ROS, the `image` message type is not compatible with any OpenCV functions. Therefore, the `cv_bridge` bridge package has some nice functions for converting `image` messages to OpenCV `Mat` variables and *vica versa*. To install, we will clone from the `ros-perception` GitHub repository from their ROS indigo development branch.

```bash
$ cd ~/catkin_ws/src
$ git clone -b indigo-devel https://github.com/ros-perception/vision_opencv.git
```


### Getting the pheeno_ros Package

To get our ROS package for the Pheeno, we just need to clone it from our organization repository.

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/ACSLaboratory/pheeno_ros.git
```


### Installing the Packages

Once the packages download, we can then install the packages using `catkin_make`. Remember to be within the `catkin_ws` directory and not the `src` directory when running the following commands.

```bash
$ cd ~/catkin_ws
$ catkin_make -j2
$ catkin_make install -j2
```


## Installing PlatformIO

PlatformIO is not a necessary package, but it is definitely helpful when deploying multiple Pheenos. Teensy requires a small GUI to upload files into the microcontroller. Using PlatformIO circumvents this by allowing uploaded code using the command line, which is great if you want to ssh into the Pheeno. If you are ok with using a GUI each time you want to upload code to the Teensy, then you are more than welcome to skip this section.

For those who do want the package, we just need to do a quick `pip` installation.

```bash
$ cd
$ sudo pip install -U platformio
```


Next, create a directory that will act as a project folder. For example, we used the inventive name of `Teensy` as our project folder and then went into the directory.

```bash
$ mkdir Teensy
$ cd Teensy
```


From here, we will use PlatformIO to initialize the folder for work.

```bash
$ platformio init --board teensy31
```


The `--board teensy31` argument provides the workspace the knowledege we will be working with a Teensy 3.2 board. The command above creates 2 empty directories and a project file. Place all the Arduino/Teensy library files within the `lib` directory and the actually file you wish to upload within the `src` directory. Now within the Teensy directory, run the following commands.

```bash
$ cd ~/Teensy
$ platformio run
$ platformio run --target upload
```


Both PlatformIO commands will download some pertinent Teensy microcontroller files and then begin the upload process. **The above commands only need to be run once**. Every subsequent upload to the Teensy only requires the second command to be run (it is also much faster than the first run).

```bash
$ platformio run --target upload
```
