---
title: Package Guide Pt. 1
category: pheeno_ros
order: 2
---

## Introduction

The following guides will give a brief introduction to the use of the Robot Operating System with a single or multiple Pheenos. Part 1 will give users an idea of how `rosserial_arduino` works with an example code we have created. Towards the end of the Arduino code tutorial, a quick guide to uploading your code to the Arduino/Teensy is provided.


## Arduino Code

[Once ROS is installed on the Pheeno](ROS#pheeno-ros-installation-guide), we can begin to set up the Pheeno to work with ROS. First, we need to upload the proper Arduino srcipt shown below.

```cpp
#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include "PheenoV2Basic.h"

// Create ROS Node instance
ros::NodeHandle nh;


// Create Pheeno Instance
PheenoV2Basic pheeno_robot = PheenoV2Basic(2);


// Create Messages for Publishers
std_msgs::Float32 scan_center_msg;
std_msgs::Float32 scan_back_msg;
std_msgs::Float32 scan_right_msg;
std_msgs::Float32 scan_left_msg;
std_msgs::Float32 scan_cr_msg;
std_msgs::Float32 scan_cl_msg;
std_msgs::Int16 encoder_LL_msg;  // Left HBridge, Left Motor Encoder
std_msgs::Int16 encoder_LR_msg;  // Left HBridge, Right Motor Encoder
std_msgs::Int16 encoder_RL_msg;  // Right HBridge, Left Motor Encoder
std_msgs::Int16 encoder_RR_msg;  // Right HBridge, Right Motor Encoder


// Create Variables for Storing Motion Data
int linear = 0;   // Forward and Backward motion.
int angular = 0;  // Turning (Right and Left) motion.


// Create Publishers
ros::Publisher pub_ir_center("/pheeno_55/scan_center", &scan_center_msg);  // Center IR
ros::Publisher pub_ir_back("/pheeno_55/scan_back", &scan_back_msg);        // Back IR
ros::Publisher pub_ir_right("/pheeno_55/scan_right", &scan_right_msg);     // Right IR
ros::Publisher pub_ir_left("/pheeno_55/scan_left", &scan_left_msg);        // Left IR
ros::Publisher pub_ir_cr("/pheeno_55/scan_cr", &scan_cr_msg);          // Center Right IR
ros::Publisher pub_ir_cl("/pheeno_55/scan_cl", &scan_cl_msg);          // Center Left IR
ros::Publisher pub_encoder_LL("/pheeno_55/encoder_LL", &encoder_LL_msg);  // Encoder LL
ros::Publisher pub_encoder_LR("/pheeno_55/encoder_LR", &encoder_LR_msg);  // Encoder LR
ros::Publisher pub_encoder_RL("/pheeno_55/encoder_RL", &encoder_RL_msg);  // Encoder RL
ros::Publisher pub_encoder_RR("/pheeno_55/encoder_RR", &encoder_RR_msg);  // Encoder RR


// Callback for cmd_vel Subscriber.
// The following callback recieves a command in terms of m/s. This will have to
// be converted into a binary output between 0-255.
void callback(const geometry_msgs::Twist &msg) {
  if (msg.linear.x > 0 || msg.linear.x < 0) {
    linear = 2550 * msg.linear.x;
    angular = 0;

  } else if (msg.linear.x == 0) {
    linear = 0;

  }

  if (msg.angular.z > 0 || msg.angular.z < 0) {
    linear = 0;
    angular = 2550 * msg.angular.z;

  } else if (msg.angular.z == 0) {
    angular = 0;

  }

}


// Create Subscribers
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("/pheeno_55/cmd_vel", callback);


void setup() {
  // Setup Pheeno robot
  pheeno_robot.SetupBasic();

  // Initialize ROS Node
  nh.initNode();

  // Start Advertising
  nh.advertise(pub_ir_center);
  nh.advertise(pub_ir_back);
  nh.advertise(pub_ir_right);
  nh.advertise(pub_ir_left);
  nh.advertise(pub_ir_cr);
  nh.advertise(pub_ir_cl);
  nh.advertise(pub_encoder_LL);
  nh.advertise(pub_encoder_LR);
  nh.advertise(pub_encoder_RL);
  nh.advertise(pub_encoder_RR);

  // Start Subscribing
  nh.subscribe(sub_cmd_vel);

}


void loop() {

  // Refresh Sensor Readings
  pheeno_robot.readIR();
  pheeno_robot.readEncoders();

  // Assign sensor values to Msg variable
  scan_center_msg.data = pheeno_robot.CDistance;
  scan_back_msg.data = pheeno_robot.BDistance;
  scan_right_msg.data = pheeno_robot.RDistance;
  scan_left_msg.data = pheeno_robot.LDistance;
  scan_cr_msg.data = pheeno_robot.RFDistance;
  scan_cl_msg.data = pheeno_robot.LFDistance;
  encoder_LL_msg.data = pheeno_robot.encoderCountLL;
  encoder_LR_msg.data = pheeno_robot.encoderCountLR;
  encoder_RL_msg.data = pheeno_robot.encoderCountRL;
  encoder_RR_msg.data = pheeno_robot.encoderCountRR;

  // Publish the Topics
  pub_ir_center.publish(&scan_center_msg);
  pub_ir_back.publish(&scan_back_msg);
  pub_ir_right.publish(&scan_right_msg);
  pub_ir_left.publish(&scan_left_msg);
  pub_ir_cr.publish(&scan_cr_msg);
  pub_ir_cl.publish(&scan_cl_msg);
  pub_encoder_LL.publish(&encoder_LL_msg);
  pub_encoder_LR.publish(&encoder_LR_msg);
  pub_encoder_RL.publish(&encoder_RL_msg);
  pub_encoder_RR.publish(&encoder_RR_msg);

  if (linear != 0) {
    if (linear > 0) {
      PheenoMoveForward(linear);

    } else {
      PheenoMoveReverse(linear);

    }

  } else if (angular != 0) {
    if (angular > 0) {
      PheenoTurnRight(angular);

    } else {
      PheenoTurnLeft(angular);

    }

  } else {
    pheeno_robot.brakeAll();

  }

  nh.spinOnce();
  delay(100);  // Required because the Teensy sends messages too fast.

}


// Turns the Pheeno left.
// Currently the appropriate use is just by providing a speed between 0-255,
// without any error handling. Be careful!
void PheenoTurnLeft(int speed) {
  pheeno_robot.reverseLR(-1 * speed);
  pheeno_robot.forwardRL(-1 * speed);

}


// Turns the Pheeno Right.
// Currently, the appropriate use is just by providing a speed between 0-255,
// without any error handling. Be careful!
void PheenoTurnRight(int speed) {
  pheeno_robot.reverseRL(speed);
  pheeno_robot.forwardLR(speed);

}


// Moves the Pheeno Forward.
// Applying a specific speed value (0-255) and both motors will apply the speed
// without error handling. Be careful!
void PheenoMoveForward(int speed) {
  pheeno_robot.forwardLR(speed);
  pheeno_robot.forwardRL(speed);

}


// Moves the Pheeno Reverse.
// Applying a specific speed value (0-255) and both motors will apply the speed
// without error handling. Be careful!
void PheenoMoveReverse(int speed) {
  pheeno_robot.reverseLR(speed);
  pheeno_robot.reverseRL(speed);

}
```


We will go through the relevant parts of this Arduino script to help familiarize yourselves with how `rosserial_arduino` programs work. In our case, the Arduino script will function as a node that will subscribe to a `cmd_vel` input while simultaneously publishing encoder and IR sensor values.


### Include Statements

```cpp
#include <ArduinoHardware.h>
#include <ros.h>
```


These two `include` statements are required to use ROS with the Arduino/Teensy. It contains packages for defining Nodes, Publishers, and Subscribers.

```cpp
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
```


The next three `include` statements are for the message types we wish to use within this script. Currently, the IR Sensors are published with the `Float32` message type, the encoders are published with `Int16` message type, and we subscribe to the `cmd_vel` topic that uses the `Twist` message type. When wanting to add additional Publishers/Subscribers, you must add an `include` statement for each different message type you plan on using.

```cpp
#include "PheenoV2Basic.h"
```


The last include statement is required to help initialize the Arduino/Teensy connection to the remainder of the Pheeno hardware.


### Global Variables

Typically, variables defined within this section are considered global variables within the Arduino/Teensy script. This is where we will define most of our variables relating to ROS.

```cpp
// Create ROS Node instance
ros::NodeHandle nh;


// Create Pheeno Instance
PheenoV2Basic pheeno_robot = PheenoV2Basic(2);
```


`rosserial_arduino` requires the instantiation of a ROS node that contains modules to advertise and subscribe our Publishers and Subscribers, respectively. In our case, the `nh` variable will serve as our ROS node. Creating an instance of `PheenoV2Basic` takes a variable 1 through 3 as options. You do not need to know what those other options are yet; however, we will discuss what those options mean in a different section.

```cpp
// Create Messages for Publishers
std_msgs::Float32 scan_center_msg;
std_msgs::Float32 scan_back_msg;
std_msgs::Float32 scan_right_msg;
std_msgs::Float32 scan_left_msg;
std_msgs::Float32 scan_cr_msg;
std_msgs::Float32 scan_cl_msg;
std_msgs::Int16 encoder_LL_msg;
std_msgs::Int16 encoder_LR_msg;
std_msgs::Int16 encoder_RL_msg;
std_msgs::Int16 encoder_RR_msg;
```


In our code, we create a message variables for the Publishers and Subscriber made later. The reason we use `Float32` and `Int16` is due to the hardware actually generating the readings with those types. Notice that we are using `std_msgs::` version of `Float32` and `Int16`. The ROS Publishers and Subscribers will only use variables that contain the ROS message wrapper, *i.e.* `std_msgs::`.

```cpp
// Create Variables for Storing Motion Data
int linear = 0;   // Forward and Backward motion.
int angular = 0;  // Turning (Right and Left) motion.
```


We will talk more about the use of these variables later, but, for now, we define the previous two variables.

```cpp
// Create Publishers
ros::Publisher pub_ir_center("/pheeno_55/scan_center", &scan_center_msg);
ros::Publisher pub_ir_back("/pheeno_55/scan_back", &scan_back_msg);
ros::Publisher pub_ir_right("/pheeno_55/scan_right", &scan_right_msg);
ros::Publisher pub_ir_left("/pheeno_55/scan_left", &scan_left_msg);
ros::Publisher pub_ir_cr("/pheeno_55/scan_cr", &scan_cr_msg);
ros::Publisher pub_ir_cl("/pheeno_55/scan_cl", &scan_cl_msg);
ros::Publisher pub_encoder_LL("/pheeno_55/encoder_LL", &encoder_LL_msg);
ros::Publisher pub_encoder_LR("/pheeno_55/encoder_LR", &encoder_LR_msg);
ros::Publisher pub_encoder_RL("/pheeno_55/encoder_RL", &encoder_RL_msg);
ros::Publisher pub_encoder_RR("/pheeno_55/encoder_RR", &encoder_RR_msg);
```


Here we define our Publishers. Each publisher takes in two arguments. The first is the ROS Topic you would like to publish, and the second is the ROS message variable associated with the Topic. Note the use of an `&` ahead of the variable. This means that the second argument is actually the *reference* to the ROS message variable.

```cpp
// Create Subscribers
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("/pheeno_55/cmd_vel", callback);
```


I skipped a couple lines, but will refer back to them in a second. Subscribers are defined slightly different from Publishers. As you can see, we need to specify within the type declaration that the recieved message will be a `geometry_msgs::Twist`. Just like in the Publisher, our first argument is the ROS message Topic; however, the second is a function! We refer to this as a *callback function*. Once the Publisher of a Topic publishes a message, the Subscriber will get the message and send it to the callback function for parsing (interpreting). The callback function used in this code is simply called `callback`. If you plan on adding more callback functions in the future, make sure to give them more specific names! It is not necessary to define the callback function before defining your Subscriber, but I do this for organizational purposes.

```cpp
// Callback for cmd_vel Subscriber.
// The following callback recieves a command in terms of m/s. This will have to
// be converted into a binary output between 0-255.
void callback(const geometry_msgs::Twist &msg) {
  if (msg.linear.x > 0 || msg.linear.x < 0) {
    linear = 2550 * msg.linear.x;
    angular = 0;

  } else if (msg.linear.x == 0) {
    linear = 0;

  }

  if (msg.angular.z > 0 || msg.angular.z < 0) {
    linear = 0;
    angular = 2550 * msg.angular.z;

  } else if (msg.angular.z == 0) {
    angular = 0;

  }

}
```


`callback` takes with it one argument: the Twist message's reference. As seen above, dot operators can access a `msg` variables data. The specific path towards the data is dependent on the ROS message type. For `geometry_msgs::Twist`, we can access the information as such.

```cpp
msg.linear.x
msg.linear.y
msg.linear.z

msg.angular.x
msg.angular.y
msg.angular.z
```


For this differential drive robot, we will only be using `linear.x` and `angular.z`. We will use he former messages information to provide linear forward and backward motion while the later gives angular rotation about the z-axis of the robot. Why do we use only two?

Looking at the top of the Pheeno, imagine 3 vectors emanating from the pheeno: one from the center pointing forward, one from the center pointing out of the left side, and one from the center point vertically upwards. The distinction of *linear* means movement is in the direction of the vector. `linear.y` is not possible due to the fact that the robot is non-holonomic. The wheels cannot move the Pheeno to the left or right without turning! Unless the Pheeno has become equipped with a tiny jetpack, it won't be able to move upwards either, so `linear.z` isn't an option either.

Angular works differently. Instead of forward and backward motion from the vector, angular motion is rotation *about* the vector. Take out a pencil, if you have one. Put the eraser on a horizontal surface and have the tip pointing vertically. Now rotate in place. The eraser shouldn't move and the pencil should stay in place. This is what we mean when we say rotating *about* an axis or vector. In the Pheeno's case, the only axis that makes sense to rotate about is the z-axis. That is why we only need `angular.z`.

Going back to the code, the callback function assigns the subscribed value from the `cmd_vel` topic message to the linear and angular values.


#### Namespaces

You may be asking "Why do we need to add a `pheeno_55` in front of our ROS topics?" Well, it helps to differentiate the ROS topics for one Pheeno and another one. Say for example, we have two Pheenos. If we sent a `Twist` message using the `/cmd_vel` topic to move *only* Pheeno \#1, both would actually move because both Subscribers receive messages from the `/cmd_vel` ROS Topic! This is because we never set a *namespace* for our device to differentiate the two Pheenos.

If you followed the guide [here](../../pheeno/raspberry_pi_content#creating-a-static-ip-on-the-network) setting up your Pheeno, you will know that the Pheeno is given a network number, such as `192.168.119.55` by you. The first 3 sets of numbers are what a router uses to identify local connections, but the last number is used to identify *unique* devices. For example, if my phone is connected to the same router as the Pheeno, it may have the address `192.168.119.34` that is automatically (and randomly) assigned to it by the router. In our case, each Pheeno was given a *static* IP. This means that it won't be given a randomly assigned number when it connects to the router, but it will continue to use the one we gave it!

When using multiple Pheenos, it is good practice to number the robot based on the IP number you gave it. For this code and the code available online, we use `55` as our numbered Pheeno. You can even do this with a single Pheeno too since you gave it a static IP upon setting the Raspberry Pi as well.


### Setup Function

```cpp
void setup() {
  // Setup Pheeno robot
  pheeno_robot.SetupBasic();

  // Initialize ROS Node
  nh.initNode();

  // Start Advertising
  nh.advertise(pub_ir_center);
  nh.advertise(pub_ir_back);
  nh.advertise(pub_ir_right);
  nh.advertise(pub_ir_left);
  nh.advertise(pub_ir_cr);
  nh.advertise(pub_ir_cl);
  nh.advertise(pub_encoder_LL);
  nh.advertise(pub_encoder_LR);
  nh.advertise(pub_encoder_RL);
  nh.advertise(pub_encoder_RR);

  // Start Subscribing
  nh.subscribe(sub_cmd_vel);

}
```


The `setup()` function of Arduino code is initialized every time the Arduino is powered on or reset. Therefore, every time the Arduino is reset (or powered on) the Pheeno must run the `SetupBasic()` and the `initNode()` methods. The first initializes the Pheeno for use, while the second method initializes the ROS node we wish to use. As a note, **BOTH** these statements must be present when attempting to write your own code for the Pheeno. Next, the node `nh` must advertise and subscribe the Publishers and Subscribers, respectively.


### Loop Function

```cpp
void loop() {

  // Refresh Sensor Readings
  pheeno_robot.readIR();
  pheeno_robot.readEncoders();
```


The `loop()` function is repeated continuously until reset or a power cycle instance. Therefore, We will focus on this section because it is what allows interaction with ROS. At the beginning of every loop, we read both of our IR sensors and encoders.

```cpp
  // Assign sensor values to Msg variable
  scan_center_msg.data = pheeno_robot.CDistance;
  scan_back_msg.data = pheeno_robot.BDistance;
  scan_right_msg.data = pheeno_robot.RDistance;
  scan_left_msg.data = pheeno_robot.LDistance;
  scan_cr_msg.data = pheeno_robot.RFDistance;
  scan_cl_msg.data = pheeno_robot.LFDistance;
  encoder_LL_msg.data = pheeno_robot.encoderCountLL;
  encoder_LR_msg.data = pheeno_robot.encoderCountLR;
  encoder_RL_msg.data = pheeno_robot.encoderCountRL;
  encoder_RR_msg.data = pheeno_robot.encoderCountRR;
```


As I explained earlier for Twist messages, ROS data types are wrappers for the information that is sent. In the case of `std_msgs::Float32`, the variable is not `Float32`, but the variable has a parameter named `data` which *is* `Float32`. The values that come from the IR Sensor and Encoders are assigned to `variable_name.data` instead of just `variable_name`.

```cpp
  // Publish the Topics
  pub_ir_center.publish(&scan_center_msg);
  pub_ir_back.publish(&scan_back_msg);
  pub_ir_right.publish(&scan_right_msg);
  pub_ir_left.publish(&scan_left_msg);
  pub_ir_cr.publish(&scan_cr_msg);
  pub_ir_cl.publish(&scan_cl_msg);
  pub_encoder_LL.publish(&encoder_LL_msg);
  pub_encoder_LR.publish(&encoder_LR_msg);
  pub_encoder_RL.publish(&encoder_RL_msg);
  pub_encoder_RR.publish(&encoder_RR_msg);
```

After assigning the proper values from the hardware to the ROS Message variables, it is now time to publish those message in their respective ROS Topics. We use the `publish()` method for each Publisher to publish the messages we want. **Remember**, the argument must be the *reference* to the message variable.

```cpp
  if (linear != 0) {
    if (linear > 0) {
      PheenoMoveForward(linear);

    } else {
      PheenoMoveReverse(linear);

    }

  } else if (angular != 0) {
    if (angular > 0) {
      PheenoTurnRight(angular);

    } else {
      PheenoTurnLeft(angular);

    }

  } else {
    pheeno_robot.brakeAll();

  }
```

After publishing, `linear` and `angular` variables are analyzed and movement assigned to the proper functions to move the Pheeno. The Pheeno will stop if either variable is set to zero.

```cpp
  nh.spinOnce();
  delay(100);  // Required because the Teensy sends messages too fast.

}
```

The `spinOnce()` method gives control of the program to ROS for only *one* cycle, then lets go. ROS will only process callbacks if you use `spinOnce()`. Therefore, we must use it in our code if we want to receive information about the `cmd_vel` topic. This is how `rosserial_arduino` (and ROS C++ code) handles Subscribers within nodes. The `delay()` function following `spinOnce()` method serves to force the system to slow down slightly. If not present, errors about missing communications will occur when attempting to connect the Arduino/Teensy to the Pheeno's Raspberry Pi

Once you have a good grasp of the information provided, upload the code to the Arduino/Teensy!
