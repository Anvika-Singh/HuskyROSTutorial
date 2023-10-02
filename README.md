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

Now, to create the package:
```
$ cd ~/husky_ros_tutorial/src
$ catkin_create_pkg ctrl_husky_tutorial std_msgs geometry_msgs rospy roscpp
```
Notice how we import some specific message types. We will talk about these in the next section  

Now we need to build the package and source it:
```
$ cd ~/husky_ros_tutorial
$ catkin_make
$ source devel/setup.bash
```

Now go to <strong>section 6.1</strong> in this tutorial: http://wiki.ros.org/catkin/Tutorials/CreatingPackage  
And follow the steps to populate the "package.xml"
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
$ cd ~/husky_ros_tutorial/src/ctrl_husky_tutorial/src
```
Now open up your favorite IDE so that we can write some code  
We will start by opening a new file and naming is "husky_tutorial_planner.py". Paste the following code into your editor so we can go through line-by-line:
```python
#!/usr/bin/env python3
import rospy
import math
import numpy as np
import tf
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseWithCovariance, Pose
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

x = 0
y = 0
yaw = 0

def tf_callback(msg):
    global x, y, theta

    frame_id = "odom"
    child_frame_id = "base_link"

    for transform in msg.transforms:
        if transform.header.frame_id == frame_id and transform.child_frame_id == child_frame_id:
            # Extract the position and orientation
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            quat = (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            )
            euler = tf.transformations.euler_from_quaternion(quat)
            theta = euler[2]
            break
    
    # uncomment to see position info logged
    # rospy.loginfo(f"x: {x}, y: {y}, yaw: {yaw}")


def main():
    rospy.init_node("ctrl_husky")
    sub = rospy.Subscriber("/tf", TFMessage, tf_callback, queue_size=10)
    pub = rospy.Publisher("/husky_velocity_controller/cmd_vel", Twist, queue_size=10)

    initial_position = np.array([0., 0.])
    # for an extra challenge, see if you can make this initialize to any starting position

    while not rospy.is_shutdown():
        if np.linalg.norm(initial_position - np.array([x, y])) > 10:
            rospy.loginfo("Finished Planning")
            break
        else:
            control_message = Twist()
            control_message.linear.x = 1.0
            control_message.angular.z = 0.3
            pub.publish(control_message)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```
### Imports
This first block of code looks like your typical imports, except for line 1. When using ROS, you must specify the path of your Python install.  
We have typical imports such as `rospy`, `math` and `numpy`. We also import `tf` which allows us to transform from quaternion coordinate frames to euler.  
The rest of the imports are all necessarry to handle the message types we will be using in this script. As you can seen below, there is a standard method for importing message types into your program.
```python
#!/usr/bin/env python3
import rospy
import math
import numpy as np
import tf
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseWithCovariance, Pose
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
```
### Main
We will first skip to the `main` function before going over the `callback`.  
The first three lines are to setup our custom node and the subscribers and publishers.  
- In a subscriber, we need to define the topic, in this case it's `/tf`, the message type, `TFMessage`, the callback function, `tf_callback`, and optionally, we can define a queue size of messages.
- In the publisher, we just need to define the topic we are publishing to, `/husky_velocity_controller/cmd_vel`, the message type, `Twist` and the queue size.
```python
    rospy.init_node("ctrl_husky")
    sub = rospy.Subscriber("/tf", TFMessage, tf_callback, queue_size=10)
    pub = rospy.Publisher("/husky_velocity_controller/cmd_vel", Twist, queue_size=10)
```
Now that we have done the housekeeping, let us get into the meat of our planner. Which is actually *very* simple.  
Our planner is going to run in a while loop that continuously provides some control input to the robot.
```python
    while not rospy.is_shutdown():
        if np.linalg.norm(initial_position - np.array([x, y])) > 10:
            rospy.loginfo("Finished Planning")
            break
        else:
            control_message = Twist()
            control_message.linear.x = 1.0
            control_message.angular.z = 0.3
            pub.publish(control_message)
```
Let's first go through the termination criteria for this specific planner.  
This is a pretty simple, but useful termination criteria to use in path planning. Once our robot reachers a euclidean distance of 10 meters from the origin, it will break out of the planning loop, which will stop the robot from moving. We then log a message to ROS to confirm that the planning is finished.
```python
        if np.linalg.norm(initial_position - np.array([x, y])) > 10:
            rospy.loginfo("Finished Planning")
            break
```
Now we will take a look at our planner. This example is the most basic of applications. You could write powerful planners using other methods such as *trajectory optimization*, *social forces model* etc. in a script with a very similar format.
- We first define the message type that we are going to append to. In this case it is the type `Twist`, which makes sense because this is the message type we defined for our publisher.
- The next two lines are where we define the control input. You can change these to any value you would like, but know that the husky robot is bounded by physics, so it has upper bounds for the linear and angular velocities.
- The final line is where we finally publish our control. Once this line executes, we have finally sent the message to our robot, and it will then execute the given control command.
```python
            control_message = Twist()
            control_message.linear.x = 1.0
            control_message.angular.z = 0.3
            pub.publish(control_message)
```
### Callback Function
Now we are going to take a look at the `tf_callback` function. Callbacks are commonly used in ROS code because they will be called anytime a topic within the ROS system updates. This is important for getting the most up to date position info on an agent; which is what we will do here.
```python
def tf_callback(msg):
    global x, y, theta

    frame_id = "odom"
    child_frame_id = "base_link"

    for transform in msg.transforms:
        if transform.header.frame_id == frame_id and transform.child_frame_id == child_frame_id:
            # Extract the position and orientation
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            quat = (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            )
            euler = tf.transformations.euler_from_quaternion(quat)
            theta = euler[2]
            break
```
Let's first take a look at the code prior to the 'for loop':
- We want to make sure we are defining the position variables in a global scope.
- Lines 2 and 3 are where we define which fields from the `/tf` topic message we want. The `/tf` topic publishes a message with multiple different reference frames, so we want to make sure we are only using the refernce frame that provides the robot's position (`base_link`) relative to the origin (`odom`).
```python
    global x, y, theta
    frame_id = "odom"
    child_frame_id = "base_link"
```

After saving the code. You must first make it an executable and then build run catkin_make in the catkin folder.
