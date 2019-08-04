#include "perception_control/bb_rotational.h"

using namespace perception_control;

#include <perception_control/bb_proportional.h>

BBRotational::BBRotational(ros::NodeHandle& nh) : BBController(nh)
{
    ROS_INFO("loading BB Rotational");

    // Load carrot + threshold params
    std::string occamNamespace = "perception_control/bb_controllers/rotational";
    if( !nh.getParam(occamNamespace + "/threshold", m_rotationThresh) )
    {
        std::cout << "BB rotational unable to locate params." << std::endl;
        std::abort();
    }
    nh.getParam(occamNamespace + "/carrot", m_rotationCarrot);

    // Occam transforms
    
    };
}

float BBRotational::getTargetYaw(const std::string &occamFrame)
{
    float targetYaw = yawState - m_occamTransforms[occamFrame]; // figure out correct 
    if (targetYaw > M_PI)
    {
        while (targetYaw > M_PI)
        {
            targetYaw -= 2*M_PI;
        }
    }
    else if (targetYaw < -M_PI)
    {
        while (targetYaw < -M_PI)
        {
            targetYaw += 2*M_PI;
        }
    }

    return targetYaw;
}

void BBRotational::targetYaw(float &target)
{
    std_msgs::Float64 setYaw;
    setYaw.data = target;
    if (abs(target - yawState) > m_rotationThresh) // check if we've exceeded our max setpoint range
    {
        yawPub.publish(setYaw);
    }
}