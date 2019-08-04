#include "perception_control/bb_rotational.h"

using namespace perception_control;

#include <perception_control/bb_proportional.h>

BBRotational::BBRotational(ros::NodeHandle& nh) : BBController(nh)
{
    ROS_INFO("loading BB Rotational");

    // OCCAM
    std::string occamNamespace = "perception_control/bb_controllers/rotational/occam";
    if( !nh.getParam(occamNamespace + "/rotation/threshold", m_rotationThresh) )
    {
        std::cout << "BB rotational unable to locate params." << std::endl;
        std::abort();
    }
    nh.getParam(occamNamespace + "/rotation/carrot", m_rotationCarrot);
    
    std::string ns;
    std::vector<std::string> occamFrames;
    float frameOffset;
    nh.getParam(occamNamespace + "/frames_ns", ns);
    nh.getParam(occamNamespace + "/frames", occamFrames);
    nh.getParam(occamNamespace + "/frame_offset", frameOffset);
    createOccamTfs(ns, occamFrames, frameOffset);
}

void BBRotational::createOccamTfs(const std::string &frameNS, std::vector<std::string> occamFrames, const float &frameOffset)
{
    float offset = frameOffset;
    for (int i = 0; i < occamFrames.size(); ++i)
    {
        offset = (frameOffset * (i+1));
        m_occamTransforms.insert(std::make_pair(frameNS + occamFrames.at(i), offset));
    }
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