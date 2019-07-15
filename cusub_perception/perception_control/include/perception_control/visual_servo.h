#ifndef VISUALSERVO_CLASS_SRC_VISUALSERVO_CLASS_H_
#define VISUALSERVO_CLASS_SRC_VISUALSERVO_CLASS_H_

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <actionlib/server/simple_action_server.h>
#include <waypoint_navigator/ToggleControl.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

namespace perception_control
{
    class VisualServo : public nodelet::Nodelet
    {
    public:
        virtual void onInit();
    private:
    };
}

#endif