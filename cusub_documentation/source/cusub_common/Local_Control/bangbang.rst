********
BangBang
********
This is a simple Bang Bang controller node.  It will command a preset control effort when the input state is above or below a certain amount of the setpoint.

Publishers
##########

- Control Effort
    :Topicname: ~control_effort_topic (From Parameter)
    :Type: std_msgs/Float64

    Outputs Control Effort

Subscribers
###########

- Setpoint
    :Topicname: ~setpoint_topic (From Parameter)
    :Type: std_msgs/Float64

    Sets desired state for control loop to achive

- State
    :Topicname: ~state_topic (From Parameter)
    :Type: std_msgs/Float64

    Feedback on current state

Parameters
##########

- ~setpoint_topic
    :Type: string
    :Default: Required (No default)

    Topic to subscribe to setpoints

- ~state_topic
    :Type: string
    :Default: Required (No default)

    Topic to get state feedback on

- ~control_effort_topic
    :Type: string
    :Default: Required (No default)

    Topic to publish control effort on.

- ~deadzone_size
    :Type: float
    :Default: 1.0

    Size of the deadzone where control effort will be set to zero.  Deadzone is centered around the setpoint.

- ~left_bang_effort
    :Type: float
    :Default: 1.0

    Control Effort that is output when state is less than setpoint - (deadzone_size / 2).

- ~right_bang_effort
    :Type: float
    :Default: -1.0

    Control Effort that is output when state is greater than setpoint + (deadzone_size / 2).
