---
title: pheeno_ros ROS Package Guide Part 2
category: ROS
order: 3
use_math: true
---

## Introduction

Having recently uploaded your code to the Teensy/Arduino, Part 2 will show users how to run ROS nodes on the Raspberry Pi. This part will focus more on running ROS on a single Pheeno. If you want to see how to get ROS running with multiple Pheenos and a single computer acting as ROS master, please have a look here in [Part 3](pheeno-ros-guide-pt-3).


## Python Code

Lets take a look at a code example for a Pheeno to undergo a random walk with obstacle avoidance:

```python
#!/usr/bin/env python

import rospy
import random
import argparse
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


def get_args():
    """ Get arguments from rosrun for individual deployment. """
    parser = argparse.ArgumentParser(
        description="Obstacle avoidance python script."
    )

    # Required arguments
    parser.add_argument("-n", "--number",
                        action="namespace",
                        required=False,
                        help="Add a pheeno number namespace.",
                        default="")

    return parser.parse_args(rospy.myargv()[1:])


def random_turn():
    """ Random turn direction.
    Returns a random turn direction for the pheeno based on a uniform
    distribution.
    """
    if random.random() < 0.5:
        turn_direction = -0.05  # Turn right

    else:
        turn_direction = 0.05  # Turn left

    return turn_direction


def obstacle_check(msg, ir_location):
    global is_obstacle_in_way
    global sensor_triggered
    global sensor_limits
    if msg.data < sensor_limits[ir_location]:
        sensor_triggered = ir_location
        is_obstacle_in_way[ir_location] = True

    else:
        is_obstacle_in_way[ir_location] = False


if __name__ == "__main__":
    # Get arguments from argument parser.
    input_args = get_args()
    if input_args is "":
        pheeno_number = ""

    else:
        pheeno_number = "/pheeno_" + str(input_args.number)

    # Initialize Node
    rospy.init_node("pheeno_obstacle_avoidance")

    # Create Publishers and Subscribers
    pub = rospy.Publisher(
        pheeno_number + "/cmd_vel", Twist, queue_size=100)
    sub_ir_center = rospy.Subscriber(
        pheeno_number + "/scan_center", Float32, obstacle_check,
        callback_args="center")
    sub_ir_back = rospy.Subscriber(
        pheeno_number + "/scan_back", Float32, obstacle_check,
        callback_args="back")
    sub_ir_right = rospy.Subscriber(
        pheeno_number + "/scan_right", Float32, obstacle_check,
        callback_args="right")
    sub_ir_left = rospy.Subscriber(
        pheeno_number + "/scan_left", Float32, obstacle_check,
        callback_args="left")
    sub_ir_cr = rospy.Subscriber(
        pheeno_number + "/scan_cr", Float32, obstacle_check,
        callback_args="cr")
    sub_ir_cl = rospy.Subscriber(
        pheeno_number + "/scan_cl", Float32, obstacle_check,
        callback_args="cl")

    # Global Variables
    global is_obstacle_in_way
    global sensor_triggered
    global sensor_limits

    # Other important variables
    is_obstacle_in_way = {"center": False, "cr": False, "right": False,
                          "back": False, "left": False, "cl": False}
    sensor_triggered = 0
    sensors = {"center": 0.05, "cr": -0.05, "right": -0.05,
               "back": -0.05, "left": 0.05, "cl": 0.05}
    sensor_limits = {"center": 25.0, "cr": 10.0, "right": 10.0,
                     "back": 15.0, "left": 10.0, "cl": 10.0}
    cmd_vel_msg = Twist()
    obs_cmd_vel_msg = Twist()
    rate = rospy.Rate(2)
    count = 0

    while not rospy.is_shutdown():
        if True in is_obstacle_in_way.values():
            obs_cmd_vel_msg.linear.x = 0
            obs_cmd_vel_msg.angular.z = sensors[sensor_triggered]
            pub.publish(obs_cmd_vel_msg)

            # Prevent a random turn into an object after trying to avoid.
            count = 3

        else:
            if count is 0:
                cmd_vel_msg.linear.x = 0
                cmd_vel_msg.angular.z = random_turn()
                pub.publish(cmd_vel_msg)
                count += 1

            elif count is 3:
                cmd_vel_msg.linear.x = 0.05
                cmd_vel_msg.angular.z = 0
                pub.publish(cmd_vel_msg)
                count += 1

            elif count is 15:
                count = 0

            else:
                pub.publish(cmd_vel_msg)
                count += 1

        rate.sleep()
```


### Import Statements

```python
import rospy
import random
import argparse
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
```

For any Python script to run with ROS, the first library you to import should be `rospy`. It contains all the necessary methods, classes, and variables we will need to write our code. The next two are for this specific program. The last two libraries are specific to the messages we will use in our Publishers and Subscribers. For this example, we will be subscribing to sensor value topics that use a `Float32` message type. Our `cmd_vel` topic sends `Twist` messages.

If you recall from Part 1, the message types we are importing are the same as those in the Arduino code example!


### The Main Script

You may have noticed that we passed by the function definitions, but we promise we will get back to them! Here is where the meat of our python ROS script resides, and it will make understanding the function definitions easier when know when and where they are called!


#### Namespaces and Pheeno Numbering

```python
if __name__ == "__main__":
    # Get arguments from argument parser.
    input_args = get_args()
    if input_args is "":
        pheeno_number = ""

    else:
        pheeno_number = "/pheeno_" + str(input_args.number)
```

The `if` statement at the beginning is something that programmers should use often when writing Python code. For those who do not know why we use this statement, this [link](https://stackoverflow.com/a/419185) gives a good explanation for its use.

To begin, we start by taking in any arguments provided to use when the `rosrun` command attempts to create a node from this script. The only argument this script allows is to number your Pheeno.

Why do we need to add a number? Well, it helps to differentiate the ROS topics for one Pheeno and another one. Say for example, we have two Pheenos. If we sent a `Twist` message using the `/cmd_vel` topic to move *only* Pheeno \#1, both would actually move! This is because we never set a *namespace* for our device. There are more advanced ways to set up namespaces, but, for the remainder of this tutorial, we will just add a specific name when we want to publish or subscribe to topics related to a specific Pheeno. Like this!

```python
pub = rospy.Publisher("/pheeno_55/cmd_vel", Twist, queue_size=100)
```

If you followed the guide [here](../../pheeno/raspberry_pi_content#creating-a-static-ip-on-the-network) setting up your Pheeno, you will know that the Pheeno is given a network number, such as `192.168.119.55` by you. The first 3 sets of numbers are what a router uses to identify local connections, but the last number is used to identify *unique* devices. For example, if my phone is connected to the same router as the Pheeno, it may have the address `192.168.119.34` that is automatically (and randomly) assigned to it by the router. In our case, each Pheeno was given a *static* IP. This means that it won't be given a randomly assigned number when it connects to the router, but it will continue to use the one we gave it!

When using multiple Pheenos, it is good practice to number the robot based on the IP number you gave it. For this code and the code available online, we use `55` as our numbered Pheeno. You can even do this with a single Pheeno too since you gave it a static IP upon setting the Raspberry Pi as well. The `if` statement following the code, however, gives us an option! If you have a single Pheeno and have written all your Publishers and Subscribers in the Arduino code without a Pheeno namespace, then just leave the space after `-n` blank.


#### Nodes, Publishers, and Subscribers

```python
    # Initialize Node
    rospy.init_node("pheeno_obstacle_avoidance")
```

Here we see how to initialize the node. All it requires is a simple call to the `init_node()` function within the `rospy` library. All it requires is a `String` name argument that will be our nodes name. If you have multiple Pheenos you do **not** need to add a namespace to these node names.

```python
    # Create Publishers and Subscribers
    pub = rospy.Publisher(
        pheeno_number + "/cmd_vel", Twist, queue_size=100)
    sub_ir_center = rospy.Subscriber(
        pheeno_number + "/scan_center", Float32, obstacle_check,
        callback_args="center")
    sub_ir_back = rospy.Subscriber(
        pheeno_number + "/scan_back", Float32, obstacle_check,
        callback_args="back")
    sub_ir_right = rospy.Subscriber(
        pheeno_number + "/scan_right", Float32, obstacle_check,
        callback_args="right")
    sub_ir_left = rospy.Subscriber(
        pheeno_number + "/scan_left", Float32, obstacle_check,
        callback_args="left")
    sub_ir_cr = rospy.Subscriber(
        pheeno_number + "/scan_cr", Float32, obstacle_check,
        callback_args="cr")
    sub_ir_cl = rospy.Subscriber(
        pheeno_number + "/scan_cl", Float32, obstacle_check,
        callback_args="cl")
```

Creating a Publisher and Subscriber is very similar to the way it is written in our Arduino code section. For the Publisher, we see that two arguments are required and an optional one is added for this program. The first is the name of the ROS topic, the second is the ROS message type, and the optional third is for the queue size. The `queue_size` option establishes the amount of messages built up for asynchronous publishing.

The Subscribers are only a little different. They have three arguments and an optional one that we will use for just this code. The first is the ROS topic name, the second the the ROS message type, and the third is the callback function we wish to our program to use when a message arrives to the Subscriber. The optional argument is something I find very useful. It allows users to add extra arguments to the callback function. In our case, we are going to be having an extra argument that passes the location of the IR sensor. We will talk more about why we send back the IR scanner locations in the [Function Definitions](#function-definitions) section later.

#### Global and Other Variables

```python
    # Global Variables
    global is_obstacle_in_way
    global sensor_triggered
    global sensor_limits

    # Other important variables
    is_obstacle_in_way = {"center": False, "cr": False, "right": False,
                          "back": False, "left": False, "cl": False}
    sensor_triggered = 0
    sensors = {"center": 0.05, "cr": -0.05, "right": -0.05,
               "back": -0.05, "left": 0.05, "cl": 0.05}
    sensor_limits = {"center": 25.0, "cr": 10.0, "right": 10.0,
                     "back": 15.0, "left": 10.0, "cl": 10.0}
    cmd_vel_msg = Twist()
    obs_cmd_vel_msg = Twist()
    rate = rospy.Rate(2)
    count = 0
```

The following variables are created to help make the program run quicker and allows for easy changing. As you can see, I use four dictionary variables. Each key variable is a string containing the position of the sensor. In the [Function Definitions](#function-definitions) and [Loop](#loop) sections, we will see how we will use these variables.

There is also one more variable I would like to address and it is the `rate` variable that is given the `rospy.Rate(2)` value. This corresponds to the publish rate for our node in Hz. At the end of the `while` loop, which I will discuss later, a `rate.sleep()` statement is used. This will put the node to sleep for the remainder of the time left for $2$ Hz. $2$ Hz is $0.5$ seconds, so if the `while` loop finishes a single loop in $0.25$ seconds, the node will pause for the remainder of the time until it finishes reaches $0.5$ seconds.


## Loop

```python
    while not rospy.is_shutdown():
        if True in is_obstacle_in_way.values():
            obs_cmd_vel_msg.linear.x = 0
            obs_cmd_vel_msg.angular.z = sensors[sensor_triggered]
            pub.publish(obs_cmd_vel_msg)

            # Prevent a random turn into an object after trying to avoid.
            count = 3

        else:
            if count is 0:
                cmd_vel_msg.linear.x = 0
                cmd_vel_msg.angular.z = random_turn()
                pub.publish(cmd_vel_msg)
                count += 1

            elif count is 3:
                cmd_vel_msg.linear.x = 0.05
                cmd_vel_msg.angular.z = 0
                pub.publish(cmd_vel_msg)
                count += 1

            elif count is 15:
                count = 0

            else:
                pub.publish(cmd_vel_msg)
                count += 1

        rate.sleep()
```

This `while` loop represents the repeated actions that our Pheeno will take. The condition for this loop is a default and should be used anytime you create a node using Python. All it does is stop the loop (and node) if ROS shuts down or the node is closed.

The loop starts by evaluating if an object is in the way. The variable it checks in the `if` statement is a dictionary that stores if an object is detected at a certain distance from the Pheeno. Writing

```python
if True in is_obstacle_in_way.values():
```

checks if any of the values *in* the dictionary are `True`. If any of the values within the dictionary are `True`, then the Pheeno must attempt to avoid the object. It does this by publishing to the `/cmd_vel` topic where it wants to movie. Another dictionary (`sensors`) is used to determine which direction and speed the Pheeno should turn. Notice how we make the `linear.x` equal to 0. This just makes sure that the Pheeno will stop moving and just turn away from the object.

If there are no objects in the way, the `if` statement will continue to the other portion. This portion controls the random walk using a `count` variable. Since the publish rate ($2$ Hz), we know that every $0.5$ seconds a command is being published to the Pheeno. So every increase in the `count` variable corresponds to about $0.5$ seconds occurring in real-time (this isn't completely accurate, but it is a good guess).

When the `count` variable is $0$, it sets a random direction for the Pheeno to turn based on a returned value from the `random_turn()` function. As stated previously, the `linear.x` is kept at $0$ to ensure the Pheeno will only turn in place. The `Twist` message is then published to the `/cmd_vel` topic. At $3$, we switch to linear motion. The `angular.z` is set to $0$ and the `linear.x` is then given a static velocity value. Finally, the `count` variable is reset at $15$. If the `count` lands on a number that wasn't stated, it will continue to publish the current speeds on the `/cmd_vel` topic as shown in the final `else` statement.


### Function Definitions

Three functions were made for this example. The first is for an argument parser, so we can skip that because python scripts do not always require arguments.

```python
def random_turn():
    """ Random turn direction.
    Returns a random turn direction for the pheeno based on a uniform
    distribution.
    """
    if random.random() < 0.5:
        turn_direction = -0.05  # Turn right

    else:
        turn_direction = 0.05  # Turn left

    return turn_direction
```

This function will be used by our Pheeno to undergo a random walk. After being called, it will return an angular speed of either $0.05$ m/s or $-0.5$ m/s that is determined by a random number. This is where importing the `random` library comes into play!

```python
def obstacle_check(msg, ir_location):
    global is_obstacle_in_way
    global sensor_triggered
    global sensor_limits
    if msg.data < sensor_limits[ir_location]:
        sensor_triggered = ir_location
        is_obstacle_in_way[ir_location] = True

    else:
        is_obstacle_in_way[ir_location] = False
```

Each received message by our IR sensor subscribers will result in calling this callback function. `msg` is the argument containing the ROS message received by the Subscriber while the `ir_location` argument is what `callback_args` option assigns. As stated in Part 1 of this guide, ROS messages wrap the actual message. To access the data in the message requires a dot operator to get to it as can be seen by my use of `msg.data`.

A quick comparison is done to sensor limits that I have set within the `sensor_limits` dictionary variable. If the IR sensor at that location gives a reading lower than our limit, it will log the IR sensor that was triggered and change its boolean value within the `is_obstacle_in_way` dictionary variable. As you can see, the `ir_location` argument provided to us by the `callback_args` option is very useful, especially in our case with multiple Subscribers containing similar information.
