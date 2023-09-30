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
