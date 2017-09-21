---
title: Package Guide Pt. 1
category: pheeno_ros_sim
order: 1
---

> **NOTE:** This page is under contruction.

## Introduction

The `pheeno_ros_sim` package provides users with a way to simulate a single, or multiple, pheeno/s in a 3D environment. The code has been written to allow users near-perfect interoperability with a physical Pheeno. In essence, code written for the simulator should work the same way on an actual Pheeno. This part of the `pheeno_ros_sim` guide will show users the fundamentals of the package. Other parts will show users how to modify the Gazebo world around them.


## Installation

Unlike the `pheeno_ros` package, this package should be installed on a powerful computer with a full desktop installation of either ROS Indigo or Kinetic. If either distribution is installed, cloning the git repository within your catkin workspace and using `catkin_make` will install the Pheeno simulator ROS package.

The following will show the installation procedures:

```bash
$ cd ~/your_catkin_ws/src
```

Clone the proper GitHub repository depending on your ROS Distribution

```bash
// For ROS indigo
$ git clone -b indigo-devel https://github.com/acslaboratory/pheeno_ros_sim.git

// For ROS Kinetic
$ git clone -b kinetic-devel https://github.com/acslaboratory/pheeno_ros_sim.git
``` 

Finally, `cd` back to the your caktin workspace and run `catkin_make`.

```bash
$ cd ..
$ catkin_make
```

Remember to run `source devel/setup.bash` from your catkin workspace's root to access those files.


## Running a Single Pheeno

To run a single pheeno, you can either create your own launch file or use one of the predifined ones we have provided in the repository.


### Creating Your Own Launch File

Provided below is an example launch file you can write. We will then go through certain segments and explain the procedure behind running launch files.

```xml
<launch>
  <!-- Args for use later -->
  <arg name="paused" default="false"/>
  <arg name="use_sim_time" default="true"/>
  <arg name="gui" default="true"/>
  <arg name="headless" default="false"/>
  <arg name="debug" default="false"/>
    
  <!-- To use an empty world, keep the following as is. Otherwise, uncomment line 11 and comment line 10. -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
  <!-- <include file="$(find pheeno_ros_sim)/launch/testbed.launch"> -->
    <arg name="debug" value="$(arg debug)"/>
    <arg name="gui" value="$(arg gui)"/>
    <arg name="paused" value="$(arg paused)"/>
    <arg name="use_sim_time" value="$(arg use_sim_time)"/>
    <arg name="headless" value="$(arg headless)"/>
  </include>

  <!-- URDF XML Pheeno robot description loaded on the parameter server. -->
  <param name="robot_description" command="$(find xacro)/xacro.py '$(find pheeno_ros_sim)/urdf/pheeno_v1/pheeno.xacro'"/>

  <!-- Start Joint State Publisher -->
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher"></node>

  <!-- Start Robot State Publisher -->
  <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" output="screen">
    <param name="publish_frequency" type="double" value="50.0"/>
  </node>

  <!-- Run a python script to the send a service call to gazebo_ros to spawn a URDF robot -->
  <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen" args="-urdf -model pheeno_01 -param robot_description -robot_namespace pheeno_01"/>

</launch>
```

As you can see, a `.launch` file is essentially a ROS specific XML file. To begin a launch file, we start by creating an open and closed notation for the `launch` tag.

```xml
<launch>
    <!-- YOUR STUFF GOES HERE! -->
</launch>
```

This will serve as our starting point in writing the launch file. The first lines are to intialize some argument tags. Like variables in a programming language, the `<arg>` tag takes in a name for the variable and a default value to go along with it. In our file, we defined five variables.

```xml
  <arg name="paused" default="false"/>
  <arg name="use_sim_time" default="true"/>
  <arg name="gui" default="true"/>
  <arg name="headless" default="false"/>
  <arg name="debug" default="false"/>
```

The next section is very important to start a Gazebo simulation. One interesting concept of launch files is that they may be used to launch other launch files! This next part is where we see that in action. The `gazebo_ros` package provides multiple launch files that start Gazebo with pre-defined worlds. The most common is the `empty_world.launch` which, as the name suggests, launches an empty Gazebo world. Because swarm robotics is done in a confined area, we have provided users a world that they can use called `testbed.launch` located in the repository. You cannot use this immediately, however, because it must be installed. It is a fairly simple installation that can be found [here]().

Like functions, launch files may specify certain arguments that are need to work. Those five variables we defined earlier will be used as arguments for the `empty_world.launch` file.

```xml
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <!-- <include file="$(find pheeno_ros_sim)/launch/testbed.launch"> -->
    <arg name="debug" value="$(arg debug)"/>
    <arg name="gui" value="$(arg gui)"/>
    <arg name="paused" value="$(arg paused)"/>
    <arg name="use_sim_time" value="$(arg use_sim_time)"/>
    <arg name="headless" value="$(arg headless)"/>
  </include>
```

Both the `empty_world.launch` and `testbed.launch` file take the same types of arguments. If you would like an empty world, you can keep the `<include>` statement as it is; however, if you would like to have a testbed in Gazebo (and you have installed the file correctly), you can comment the `empty_world.launch` line and uncomment the `testbed.launch` line. To give an launch file an argument, you must use the `<arg>` tag. Instead of using `default` to define a value, we need to use the `value` attribute. With the `value` attribute, we can supply the variable we wish to use like this:

```xml
<arg name="debug" value"$(arg debug)"/>
```

As you can see, the launch file argument named "debug" is given the variable `debug` that we defined earlier. Notice how it is written. They use a `$` sign and ,enclosed in parentheses, use `arg` followed by `debug`. This XML convention allows users to supply variables. The world launch files take in 5 arguments. You can change the values to what you would like and test out what happens.

```xml
  <!-- URDF XML Pheeno robot description loaded on the parameter server. -->
  <param name="robot_description" command="$(find xacro)/xacro.py '$(find pheeno_ros_sim)/urdf/pheeno_v1/pheeno.xacro'"/>
```

Next, we use the `param` tag to load our `.xacro` file with our Pheeno model. The `name` attribute is a user-defined name that we will use to give to later tags. The command attribute `command` will run make just run the following python file and parse the `.xacro` file for the Pheeno into a readable model in Gazebo.

```xml
  <!-- Start Joint State Publisher -->
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher"></node>
    
  <!-- Start Robot State Publisher -->
  <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" output="screen">
    <param name="publish_frequency" type="double" value="50.0"/>
  </node>
```

The above mentioned tags will be our first exposure to the `node` tag. These two nodes are necessary for publishing information from our ROS files to the Gazebo model for robot actuation and motion.

```xml
  <!-- Run a python script to the send a service call to gazebo_ros to spawn a URDF robot -->
  <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen" args="-urdf -model pheeno_01 -param robot_description -robot_namespace pheeno_01"/>

```

To spawn our model in Gazebo, a ROS file named `spawn_model` (located `gazebo_ros` package) needs to be called. As you can see, the `node` tag takes 6 attributes: `name`, `pkg`, `type`, `respawn`, `output`, and `args`. `name` refers to the name we want the ROS node created in this XML tag to be called. `pkg` is the ROS package in which the node's source code is located. `type` is the node file. If you are using a Python file, you need to add a `.py` to the end of the name; however, if the file is a C++ executable, you do not need to add the file extension. In our case, the `spawn_model` file is a C++ executable, **but** if it was a python file it would be `"spawn_model.py"`. `respawn` refers to node failure. If the node dies and this option is `true`, ROS will attempt to respawn (or restart) the node. The `output` parameter has 2 options: `log` or `screen`. If `screen` is the option used, any type of ROS Log calls will be displayed on screen, but would be placed in a log file if the `log` option is chosen. Finally, the `args` tag are the options that the ROS node file requires to run. The `spawn_model` file takes many options that you can peruse through. I do, however, want to draw your attention to one of the options called `-param`. As you can see right after the `-param` phrase is the `robot_description` param that we made earlier!

Another couple important argument we give to `spawn_model` are `-model` and `-robot_namespace`. When running more than one Pheeno, it is important to give these names (such as `pheeno_01` like I have) because it will affect the ROS Topic names of the Gazebo model. For example, the `\cmd_vel` topic for the Pheeno Gazebo model, will now become `\pheeno_01\cmd_vel`. This is super useful when trying to simulate multiple Pheenos in Gazebo!


#### Adding Your Own Node

If you want to add your own ROS node file, you can use the XML `node` tag as shown in the section above. The following will be an example with a node that we provide.

```xml
  <!-- Run python script that will determine random movement and obstacle avoidance for all agents. -->
  <node name="pheeno_obstacle_avoidance_1" pkg="pheeno_ros_sim" type="obstacle_avoidance.py" args="-n 01"/>
```

Using a python script file located in our `pheeno_ros_sim` package, we launch a node that will make our Pheeno Gazebo model avoid any obstacles that come within a certain distance of it's IR sensors. As with `node` tag, this tag uses very similar tags though not all of them were used. The `node` tag only requires 3 attributes to execute. Other attributes may be added as needed. In this example, the `obstacle_avoidance.py` file tages in 1 argument; therefore, we need the `args` attribute. The value given to the `name` attribute will be the name of our node. As you can see, we add a number to the end of the node name. If you plan on adding more Pheenos using the same script/executable, `roslaunch` will fail to launch due to nodes having the same name. The `pkg` attribute should be the name of the package that the script/executable is located. Finally, `type` is the name of the script/executable. If you are using a python script, remember to keep the `.py` exetension to the name, but not the C++ executable.


### Using a Predefined One

Provided with the repository are predefined Gazebo launch files for users to use as examples or just to test out Pheeno on Gazebo. These files are located [here](https://github.com/ACSLaboratory/pheeno_ros_sim/tree/indigo-devel/launch). Remember to choose the correct branch for the ROS distribution you are using!


## Running Multiple Pheenos

