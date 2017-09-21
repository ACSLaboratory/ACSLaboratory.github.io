---
title: Modifying Gazebo World
category: pheeno_ros_sim
order: 1
---

> **NOTE:** This page is still under construction.

## Introduction


## Adding a Testbed Model

The following example will give users a guide about how to add a user generated model for use in Gazebo.

### The `.gazebo/models/` Directory

Hidden within the Home directory is a folder named `.gazebo`. Press Ctrl-H to see this folder within Nautilus. This folder contains many of the default files that Gazebo uses to run. It also stores models that you can download from the internet. You can actually add models to the folder yourself, and our guide will show you how to do that with one that is contained in 


### The `models` Folder

`model.config`

```xml
<?xml version="1.0"?>
<model>
    <name>testbed</name>
    <version>1.0</version>
    <sdf version="1.4">model.sdf</sdf>

    <author>
        <name>your_name</name>
        <email>your_email@emailprovider.com</email>
    </author>

    <description>
        A simple testbed for multiple Pheenos (or robots).
    </description>
</model>
```


### The `world` Folder

`testbed.world`

```xml
<?xml version="1.0"?>
<sdf version="1.4">
    <world name="default">
        <include>
            <uri>model://sun</uri>
        </include>
        <include>
            <uri>model://ground_plane</uri>
        </include>
        <include>
            <uri>model://testbed</uri>
        </include>
    </world>
</sdf>
```


### The World Launch File

`testbed.launch`

```xml
<launch>
    <!-- These are the arguments you can pass this launch file, for example paused:=true -->
    <arg name="paused" default="false"/>
    <arg name="use_sim_time" default="true"/>
    <arg name="extra_gazebo_args" default=""/>
    <arg name="gui" default="true"/>
    <arg name="headless" default="false"/>
    <arg name="debug" default="false"/>
    <arg name="physics" default="ode"/>
    <arg name="verbose" default="false"/>
    <arg name="world_name" default="$(find pheeno_ros_sim)/worlds/testbed.world"/>

    <group if="$(arg use_sim_time)">
        <param name="/use_sim_time" value="true"/>
    </group>

    <!-- set command arguments -->
    <arg unless="$(arg paused)" name="command_arg1" value=""/>
    <arg if="$(arg paused)" name="command_arg1" value="-u"/>
    <arg unless="$(arg headless)" name="command_arg2" value=""/>
    <arg if="$(arg headless)" name="command_arg2" value="-r"/>
    <arg unless="$(arg verbose)" name="command_arg3" value=""/>
    <arg if="$(arg verbose)" name="command_arg3" value="--verbose"/>
    <arg unless="$(arg debug)" name="script_type" value="gzserver"/>
    <arg if="$(arg debug)" name="script_type" value="debug"/>

    <!-- Start gazebo server -->
    <node name="gazebo"
          pkg="gazebo_ros"
          type="$(arg script_type)"
          respawn="false"
          output="screen"
          args="$(arg command_arg1) $(arg command_arg2) $(arg command_arg3) -e $(arg physics) $(arg extra_gazebo_args) $(arg world_name)"/>

    <!-- Start gazebo client -->
    <group if="$(arg gui)">
        <node name="gazebo_gui" pkg="gazebo_ros" type="gzclient" respawn="false" output="screen"/>
    </group>

</launch>
```

