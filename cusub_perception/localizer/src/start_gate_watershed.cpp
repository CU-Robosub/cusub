#include <localizer/start_gate_watershed.h>

namespace pose_generator
{   
    bool StartGateWatershed::generatePose(
        sensor_msgs::Image& image, 
        darknet_ros_msgs::BoundingBox bbs[],
        geometry_msgs::Pose& pose,
        std::string& class_name
    ){
        // ROS_INFO("Localizing the gate!");
        return false;
    }
    StartGateWatershed::StartGateWatershed()
    {
        ROS_INFO("Initializing StartGate Watershed!");
    }
}