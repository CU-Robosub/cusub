# Global_Control
Repo to store the global state machine.
## Contents
* ### Mapper
  * #### Action 
    * Action Lib file [Action Lib information](http://wiki.ros.org/actionlib)
  * #### launch
    * ROS launch file for this package
  * #### Scripts
    * mapper_client.py - This is the action that moves the sub from point a to point b
    * setpoint.py - Helper function that publish setpoints at a set rate
* ### Mapper
  * #### launch
    * ROS launch file for this package
  * #### Scripts
    * sub_main_state.py - The main statemachine for the sub. Using [SMACH](http://wiki.ros.org/smach)

## Usage
### Mapper Action
* Using an action lib client send an Odometry message as a goal. Returns a bool on completion
### Setpoint
* Publish to a pid controller setpoint topic and it will publish that to the PID package at a set rate
  * example /local_control/pid/yaw/setpoint
### State Machine
* To add to use the statemachine please refer to the wiki for [SMACH](http://wiki.ros.org/smach)
