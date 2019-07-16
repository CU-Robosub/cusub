#include <perception_control/visual_servo.h>

namespace perception_control
{
    void VisualServo::onInit()
    {
        NODELET_INFO("Visual Servo Server Starting up!");
        ros::NodeHandle& nh = getMTNodeHandle();
        wayToggleClient = nh.serviceClient<waypoint_navigator::ToggleControl>("cusub_common/toggleWaypointControl");
        darknetSub = nh.subscribe("cusub_perception/darknet_ros/bounding_boxes", 1, &VisualServo::darknetCallback, this);
        server = new vsServer(nh, "visual_servo", boost::bind(&VisualServo::execute, this, _1), false);
        server->start();
    }
    void VisualServo::darknetCallback(const darknet_ros_msgs::BoundingBoxesConstPtr bbs)
    {
        // NODELET_INFO("Received bounding box");
        if( !controllingPids ) { return; }
    }
    bool VisualServo::controlPids(const bool takeControl)
    {
        waypoint_navigator::ToggleControl toggle_srv;
        toggle_srv.request.waypoint_controlling = !takeControl;
        toggle_srv.response.success = false;
        if( wayToggleClient.call(toggle_srv) )
        { 
            if ( takeControl) { NODELET_INFO("VisualServo TOOK control of pids."); }
            else { NODELET_INFO("VisualServo RELEASED control of pids."); }
        }
        else { NODELET_ERROR("VisualServo unable to change control of PIDs"); }
        controllingPids = takeControl;
        return toggle_srv.response.success;
    }

    void VisualServo::execute(const perception_control::VisualServoGoalConstPtr goal)
    {
        NODELET_INFO("VisualServo received request.");
        activeGoal = goal;
        VisualServoResult result;
        result.success = true;
        server->setSucceeded(result);
        ros::Duration(1.0).sleep(); // give the task code a second to update the waypoint nav
    }
}
PLUGINLIB_EXPORT_CLASS(perception_control::VisualServo, nodelet::Nodelet);