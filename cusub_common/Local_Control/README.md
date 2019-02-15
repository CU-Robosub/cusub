# Local_Control
Repo that contains the local control methods for the Local controller.

This repo is designed to be placed in a catkin workspace. [Our Workspace](https://github.com/CU-Robosub/Final_Build)
## Contents
* ### Stabilization
  * #### launch
    * stabilization.launch
      * Links to all scripts in package
      * PID controllers for the sub are declared here [ROS PID](http://wiki.ros.org/pid)
  * #### Scripts
    * PID_Pololu.py - Takes the PID control efforts and converts sends them to the pololu
    * Pose_PID.py - Takes the Pose and parses it to the PID states
    * Other Files - Unused at the current time
## Usage
### PID Controllers
* Info about PID controllers : [PID](https://en.wikipedia.org/wiki/PID_controller)
### Pose 
* We use [Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html) messages
