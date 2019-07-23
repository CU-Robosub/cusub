#ifndef MOVEMENT_CLASS_SRC_MOVEMENT_CLASS_H_
#ifndef MOVEMENT_CLASS_SRC_MOVEMENT_CLASS_H_

/*
    Tool that grants control over the PID loops
    Basically the old code worked by adjusting the setpoint and the state, using the PID loops response entirely
        What if I just did P control? I've got a 60Hz update rate, maybe I can artificially add some D in there..? I don't want to tune
        You always have to tune with controls... PI usually works
        Best case scenario: use the current PID loops and just adjust the setpoints, does it recalculate the controls each time??

        Yeah what if I just adjust the setpoints accordingly, no eventually we want the state of the system coming from perception
        --> how would such an algorithm work?

        We can adjust yaw + drive setpoint based on the xyz position
        We can just keep adjusting strafe and letting it integrate, so pretty much I just need to gate waypoint navigator, tell it
        "Yo dawg I got these pids" since we're not actually doing any visual stuff here. Fundamentally different problem

        ---
        For torpedos:

        We want the depth, strafe (and drive) states to come from optical flow & the real pids can use that
 */

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

namespace perception_control
{
    class Movement
    {
    public:
        Movement();
        ~Movement();
    private:
        bool take_control(void);
        bool controlling;

        // Add ability to take control of the pid loops
        // Add publishers and subscribers to all dose pids
    }
}

#endif