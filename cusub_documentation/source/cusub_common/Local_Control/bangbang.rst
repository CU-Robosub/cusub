********
BangBang
********
This is a simple Bang Bang controller node.  It will command a preset control effort when the input state is above or below a certain amount of the setpoint.

Publishers
##########

- Control Effort
    ~control_effort_topic

    std_msgs/Float64

    Outputs Control Effort

Subscribers
###########

- Setpoint
    ~setpoint_topic

    std_msgs/Float64

    Sets desired state for control loop to achive

- State
    ~state_topic

    std_msgs/Float64

    Feedback on current state

Parameters
##########

- ~setpoint_topic
    type = string

    Required

    Topic to subscribe to setpoints

- ~state_topic
    type = string

    Required

    Topic to get state feedback on

- ~control_effort_topic
    type = string

    Required

    Topic to publish control effort on.

- ~deadzone_size
    type = float

    default = 1.0

    Size of the deadzone where control effort will be set to zero.  Deadzone is centered around the setpoint.

- ~left_bang_effort
    type = float

    default = 1.0

    Control Effort that is output when state is less than setpoint - (deadzone_size / 2).

- ~right_bang_effort
    type = float

    default = 1.0

    Control Effort that is output when state is greater than setpoint + (deadzone_size / 2).
