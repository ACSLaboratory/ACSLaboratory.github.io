---
title: RaspiCam on Pheeno
category: pheeno_ros
order: 4
---

## Introduction

In the `pheeno_ros` repository, there exists two files to utilize the Raspberry Pi's camera. The first is `pi_cam_node.py` and the other is `raspicam_ros_pub.cpp`. A cursory inspection reveals that they are the same code but written in different languages; however, one would be wrong to make that assumption. The python implementation of the camera is a node that publishes "raw" images taken from the node using Raspberry Pi's native camera API. The other uses a library called [RaspiCam](https://www.uco.es/investiga/grupos/ava/node/40), which was created by the brilliant researchers of the ["Applications of Artificial Vision" \(A.V.A\)](https://www.uco.es/investiga/grupos/ava/node/1) group located at the University of Cordoba.

By providing two ways to access the Pheeno's camera module, our platform provides greater flexibility for researchers, enthusiasts, and students. This guide will primarily focus on installing and using the RaspiCam (**NOT** the python version).


## Installing RaspiCam on a Raspberry Pi

> **NOTE:** We recommend installation of the OpenCV version, and this guide will reflect that installation!

These instructions are an adapted form of those located on [RaspiCam](https://www.uco.es/investiga/grupos/ava/node/40) project website. If you have more questions, or would like more information about the project, we would kindly refer you to their [website](https://www.uco.es/investiga/grupos/ava/node/40).

The first step is simply to download the package from [here](https://sourceforge.net/projects/raspicam/files/?). We are currently using version 0.0.16. If a newer version is available, we suggest you use that one. Once downloaded and in your `~/Downloads` folder, we will need to unzip the files.



```bash
$ cd ~/Downloads
$ unzip raspicam-x.x.x.zip
```

where the `x.x.x` will be your version number, for example:

```bash
$ cd ~/Downloads
$ unzip raspicam-0.1.6.zip
```

> **NOTE:** If an error occurs reading that `unzip` is not a command, then you will need to install the `unzip` package using `sudo apt-get install unzip`.

Once complete, we will `cd` into the unziped directory, create a build folder, and then `cd` into the build folder.

```bash
$ cd raspicam-0.1.6
$ mkdir build
$ cd build
```

Finally, we will run the cmake command to prepare the program for installation.

```bash
$ cmake ..
```

The command will print out various messages, but one message is particularly important to the installation:

```bash
-- CREATE OPENCV MODULE=1
```

If the message is printed out as such, then we know that OpenCV has been incorporated into the RaspCam build. If it is equal to 0, then you will need to install the Raspbian OpenCV package. The final step to install RaspiCam are these following commands:

```bash
$ make
$ sudo make install
$ sudo ldconfig
```

> **NOTE:** One issue that I have seen come up frequently is misplaced `.cmake` files in the `/usr/local/lib/cmake` folder. `cd` into that directory and check to see if a `raspicam` folder was created. If it was not create and you see `Findraspicam.cmake` and `raspicamConfig.cmake`files, we will need to make one and move those files into the newly created folder with the following commands:

```bash
$ cd /usr/local/lib/cmake
$ sudo mkdir raspicam
$ sudo mv raspicamConfig.cmake raspicam/
$ sudo mv Findraspicam.cmake raspicam/
```

|

|


## Installing image_transport

Once RaspiCam is installed, we will need to install the `image_transport` ROS packages to use our `raspicam_ros_pub.cpp` file. To start, we will need to clone the package from GitHub.

```bash
$ cd ~/Downloads
$ git clone https://github.com/ros-perception/image_common.git
```

We do not need everything in the `image_common` folder, so we will move just the `image_transport` package to our `~/catkin_ws/src/` directory.

```bash
$ cp ~/Downloads/image_common/image_transport/ ~/catkin_ws/src/
```

### Satisfying Other Dependencies

Before we can compile and install the `image_transport` package, we will need to download some other ROS pack dependencies. We will start by first changing directories to our catkin workspace's `src` folder.

```bash
$ cd ~/catkin_ws/src
```

The next steps will require cloning the packages from their respective GitHub repositories into the `src` folder.

#### image_transport_plugins

```bash
$ git clone -b indigo-devel https://github.com/ros-perception/image_transport_plugins.git
```

#### plugin_libs

```bash
$ git clone -b indigo-devel https://github.com/ros/pluginlib.git
```

#### dynamic_reconfigure

```bash
$ git clone https://github.com/ros/dynamic_reconfigure.git
```

#### audio_common

```bash
$ git clone indigo-devel https://github.com/ros-drivers/audio_common.git
```

#### gstreamer and Other Plugins

These dependencies are distribution specific and can be installed with an `apt-get` call.

```bash
$ sudo apt-get install gstreamer-0.10 libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev vorbis-tools libvorbis-dev libtheora-dev
```

### Compiling Everything with catkin_make

To finally compile and install all the packages, we will change directory to our catkin workspace and run `catkin_make` and `caktin_make install`

```bash
$ cd ~/catkin_ws
$ caktin_make -j2
$ catkin_make install -j2
```
