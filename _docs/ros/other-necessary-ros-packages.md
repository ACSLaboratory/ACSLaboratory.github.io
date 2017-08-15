---
title: Other Necessary ROS Packages
category: ROS
order: 3
---

## Getting Started

The previous section explained the base installation of ROS Indigo/Kinetic on the Raspberry Pi. To use other packages is a bit tricky, but the following guide provides a convenient way of installing the packages on the Pheeno.

First, change back to the main directory.

```bash
$ cd
```

From here, we will make a directory called `catkin_ws` with a child directory called `src`.

```bash
$ mkdir -p ~/catkin_ws/src
```

Finally, we change directory to the `src` directory and initialize the workspace.

```bash
$ cd ~/catkin_ws/src
$ catkin_init_workspace
```

This new catkin workspace will allow us to install other ROS packages.


## Getting the rosserial Package

`rosserial` is required for communication between the Raspberry Pi and the Teensy board. Our project requires three packages within the repository: `rosserial_client`, `rosserial_arduino`, and `rosserial_python`. To attain these files, we need to use `git clone` within the `src` folder.

```bash
$ cd ~/catkin_ws/src

# For ROS Indigo
$ git clone -b indigo-devel https://github.com/ros_drivers/rosserial.git

# For ROS Kinetic
$ git clone -b jade-devel https://github.com/ros-drivers/rosserial.git
```

The `-b` flag indicates that we would like to use the ROS Indigo specific branch of their repository. The default branch for the project is currently only for ROS Jade.


## Getting the common_msgs Package

The base installation of ROS only comes with standard messages (`std_msgs`). To get other message types such as Twist and action messages, we need to install the `common_msgs` package. Again, we will use `git clone`.

```bash
$ cd ~/catkin_ws/src

# For ROS Indigo
$ git clone -b indigo-devel https://github.com/ros/common_msgs.git

# For ROS Kinetic
$ git clone -b kinetic-devel https://github.com/ros/common_msgs.git
```

We again pull from the ROS Indigo specific branch of the repository.


## Getting the actionlib Package

This is an optional download entirely dependent on your use case. This is typically a default package for ROS; however, the `ros_comm` installation from before does not contain it. To download, we just need to clone the `indigo-devel` branch from the official ROS GitHub repository.

```bash
$ cd ~/catkin_ws/src

# For ROS Indigo
$ git clone -b indigo-devel https://github.com/ros/actionlib.git

# For ROS Kinetic
$ git clone -b kinetic-devel https://github.com/ros/actionlib.git
```


## Getting the cv_bridge Package

The pacakges within the vision_opencv repository contain files for interfacing ROS with OpenCV. In ROS, the `image` message type is not compatible with any OpenCV functions. Therefore, the `cv_bridge` bridge package has some nice functions for converting `image` messages to OpenCV `Mat` variables and *vica versa*. To install, we will clone from the `ros-perception` GitHub repository from their ROS indigo development branch.

```bash
$ cd ~/catkin_ws/src

# For ROS Indigo
$ git clone -b indigo-devel https://github.com/ros-perception/vision_opencv.git

# For ROS Kinetic
$ git clone -b kinetic-devel https://github.com/ros-perception/vision_opencv.git
```


## Getting the pheeno_ros Package

To get our ROS package for the Pheeno, we just need to clone it from our organization repository.

```bash
$ cd ~/catkin_ws/src

# For ROS Indigo
$ git clone -b indigo-devel https://github.com/ACSLaboratory/pheeno_ros.git

# For ROS Kinetic
$ git clone -b kinetic-devel https://github.com/ACSLaboratory/pheeno_ros.git
```


## Installing the Packages

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
