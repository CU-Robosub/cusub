#include <perception_control/path_orient.h>

namespace perception_control
{
    void PathOrient::onInit()
    {
        NODELET_INFO("Path Orient Server Starting Up!");
        controllingPids = false;
        orientedWithPath = false;
        ros::NodeHandle& nh = getMTNodeHandle();
        yawSub = nh.subscribe("cusub_common/motor_controllers/pid/yaw/state", 1, &PathOrient::yawCallback, this);
        yawPub = nh.advertise<std_msgs::Float64>("cusub_common/motor_controllers/pid/yaw/setpoint",1);
        wayToggleClient = nh.serviceClient<waypoint_navigator::ToggleControl>("cusub_common/toggleWaypointControl");
        darknetSub = nh.subscribe("cusub_perception/darknet_ros/bounding_boxes", 1, &PathOrient::darknetCallback, this);
        server = new pathServer(nh, "path_orient", boost::bind(&PathOrient::execute, this, _1), false);
        server->start();
    }

    void PathOrient::execute(const perception_control::PathOrientGoalConstPtr goal)
    {
        NODELET_INFO("Path Orient received request.");
        controlPids(true);
        ros::Rate r(10);
        while ( ros::ok() )
        {
            if (server->isPreemptRequested() )
            {
                controlPids(false);
                PathOrientResult result;
                result.oriented = false;
                server->setPreempted(result);
                break;
            } else if ( orientedWithPath )
            {
                PathOrientResult result;
                result.oriented = true;
                server->setSucceeded(result);
                controlPids(false);
                break;
            }
            r.sleep();
        }
    }

    void PathOrient::darknetCallback(const darknet_ros_msgs::BoundingBoxesConstPtr bbs)
    {
        if( !controllingPids ) { return; }
        orientedWithPath = true;
    }
    void PathOrient::yawCallback(const std_msgs::Float64ConstPtr state) { yawState = state->data; }

    bool PathOrient::controlPids(const bool takeControl)
    {
        waypoint_navigator::ToggleControl toggle_srv;
        toggle_srv.request.waypoint_controlling = !takeControl;
        toggle_srv.response.success = false;
        if( wayToggleClient.call(toggle_srv) )
        { 
            if ( takeControl) { NODELET_INFO("Orbitter TOOK control of pids."); }
            else { NODELET_INFO("Orbitter RELEASED control of pids."); }
        }
        else { NODELET_ERROR("Orbitter unable to change control of PIDs"); }
        controllingPids = takeControl;
        return toggle_srv.response.success;
    }
}
PLUGINLIB_EXPORT_CLASS(perception_control::PathOrient, nodelet::Nodelet);