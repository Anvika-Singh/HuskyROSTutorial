# HuskyROSTutorial
An easy to follow tutorial for lab members to follow to get them up to speed with how to interface with the Husky A200 using Python, ROS, Gazebo and RVIZ  

*Jasper Geldenbott (jgelden@uw.edu), Autumn 2023*

# Introduction
This tutorial will use a combination of the provided ROS tutorials (which utilize Turtlesim), the Husky Gazebo documentation and custom written code that is more specific to the way the CTRL Lab uses ROS and the Husky A200 Robot. For this tutorial, it is assumed that you are somewhat comfortable with Python and the Linux command line.

# Initial Steps: Setting up ROS and learning how to navigate the file system
Navigate to the ROS Tutorials: http://wiki.ros.org/ROS/Tutorials  

Under <strong>Section 1.1</strong>, complete tutorials <strong>1</strong> and <strong>2</strong> ("Installing and Configuring Your ROS Environment" and "Navigating the ROS Filesystem")  

*Note: I also recommend completing up to <strong>Tutorial 6</strong> in <strong>Section 1.1</strong>, but it is not necessary as we will go through those elements in this tutorial, just in less depth.*

# Installing the Husky Simulation
Navigate to the Husky ROS tutorial page: http://wiki.ros.org/husky_gazebo/Tutorials/Simulating%20Husky  

Follow the tutorial on this page, and verify that you are able to launch the "husky_empty_world.launch"

![image](https://github.com/UW-CTRL/HuskyROSTutorial/blob/e037962c0e13186691ad3e49f6ae88054d584284/gazebo_empty_world.png)

Your screen should look like this.

# Setting up a Workspace and ROS Package
We will now need to set up a ROS workspace, which will allow us to properly create and run our package.

*If you continued with the ROS tutorials, this should be easy, otherwise, we can set up a simple workspace with these steps*

Let's create and build a catkin workspace, in your command line:
```
$ mkdir -p ~/husky_ros_tutorial/src
$ cd ~/husky_ros_tutorial/
$ catkin_make
```
To make sure your directory was succesfully built, run `ls` and check if you have the directories `build`, `devel`, and `src`  
We need to source our new setup file. In the same directory:
```
$ source devel/setup.bash
```

# Writing the path planning algorithm
This section consists of multiple parts to provide a holistic understanding up the way we use the ROS pipeline to interface with the Husky sim and Husky robot

## Understanding ROS topics and messages
To succesfully write a path planning algorithm, it is necessary to both receive messages from nodes and to publish messages to nodes. Let's take a look at how we can view running nodes and their respective message types; which will be crucial to know when we want to write our code.
Make sure the "husky_empty_world_world" is still launched. If not, you can run `$ husky_empty_world.launch` in your command line.  

Now we want to see all the live topics. Run `$ rostopic list`  

A large list of topics should appear. But there are some that we are particularly interested in for this tutorial. For the scope of this tutorial, we mainly want to know the *state* of the robot and we want to be able to *control* its linear and angular velocities.  

To obtain the state, we will subscribe to the "/husky_velocity_controller/odom" topic. This topic has a special type of message, so lets see how to query that.  

Run: 
```
$ rostopic info /husky_velocity_controller/odom
```
You should then see this in your terminal:
```
Type: nav_msgs/Odometry

Publishers: 
 * /gazebo (http://localhost:35673/)

Subscribers: 
 * /ekf_localization (http://localhost:35531/)
```
This gives us some useful information. We are able to see the publishers and subscribers of this information as well as the message type, which is of type "nav_msgs/Odometry"  

Now we will be able to see what this type of message looks like and how we can access its fields.

Run the command:
```
$ rosmsg show nav_msgs/Odometry
```
You should then see:
```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance
geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z
  float64[36] covariance
```
This message has a lot of fields in it. So lets go through it:
```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
```
This section of the method provides its information. This includes the time stamp and the specific name of the message. This can be useful when analyzing data after an experiment.

```
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance
```
This is the most important section for us. It provides the pose information of our robot. The x, y, z position is pretty trivial. This message also provides quaternion information, which is an alternate coordinate system for describing the orientation of an agent. We will need to translate this into regular polar rotation values later in our code.

```
geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z
  float64[36] covariance
```
This message provides us with the current control input of the robot. This includes x, y, z velocity and x, y, z axis rotation.  

Now that we know how to obtain the current state and control of the robot, we need to know how to publish controls to the robot. Lets once again run `$ rostopic list`  

The message we want to pay attention to this time is the "/husky_velocity_controller/cmd_vel" topic. Now run `$ rostopic info /husky_velocity_controller/cmd_vel` We will see that is has a message type of `geometry_msgs/Twist`  

Let's once again run:
```
$ rosmsg show geometry_msgs/Twist
```
This message is more simple than the last. All we will need to publish is linear and angular control values into their respective fields.

*It is important to note that the x, y, z coordinate frame is centered on the robot*

## Simple code to control the robot in Python
Now that we understand the types of messages we will need to handle. It's time to write our path planning code. First we need to go into our `src` directory:
```
$ cd ~/husky_ros_tutorial/src
```
Now open up your favorite IDE
