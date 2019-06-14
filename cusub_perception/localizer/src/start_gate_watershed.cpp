/*
    The start gate localizer that uses the watershed algorithm
    along with a pnp solve to localize the gate.
 */

#include <localizer/start_gate_watershed.h>

namespace pose_generator
{   
    bool StartGateWatershed::generatePose(
        sensor_msgs::Image& image, 
        vector<darknet_ros_msgs::BoundingBox>& bbs,
        geometry_msgs::Pose& pose,
        string& class_name
    ){
        ROS_INFO("Localizing the gate!");
        return true;
    }
    StartGateWatershed::StartGateWatershed()
    {
        ROS_INFO("Initializing StartGate Watershed!");
    }
}

int main(int argc, char **argv)
{
    cout << "Starting Watershed Testing" <<endl;
    pose_generator::StartGateWatershed sgw;
    
    // really load the images & display them
    // pass them into generate pose
    return 0;
}