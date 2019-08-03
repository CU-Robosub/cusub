#ifndef VISUALMOVEMENT_H
#define VISUALMOVEMENT_H

#include <actionlib/server/simple_action_server.h>
#include <waypoint_navigator/ToggleControl.h>
#include <perception_control/VisualServoAction.h>
#include <perception_control/bb_proportional.h>
#include <perception_control/bb_controller.h>
#include <perception_control/bb_rotational.h>
#include <std_msgs/Float64.h>

namespace perception_control
{

class VisualMovement
{
public:
    VisualMovement();

protected:
    void initializeMovement(ros::NodeHandle * nh);
    bool controlPids(const bool takeControl);

    ros::ServiceClient m_wayToggleClient; 
    bool m_controllingPids;
    BBController * m_currentController;
    BBProportional * m_proportionalController;
    BBRotational * m_rotationalController;

};

};

#endif // VISUALMOVEMENT_H