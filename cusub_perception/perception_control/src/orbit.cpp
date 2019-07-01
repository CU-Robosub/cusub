#include <perception_control/orbit.h>

// For cancelation, use isPreempted() on server side

namespace perception_control
{
    void Orbit::onInit()
    {
        NODELET_INFO("Starting up!");
        numberSoloFrames = 0;
        numberSoloFramesThresh = 1;
        // setup subscribers & publishers
        ros::NodeHandle& nh = getMTNodeHandle();
        server = new orbitServer(nh, "orbit_buoy", boost::bind(&Orbit::execute, this, _1), false);
        wayToggleClient = nh.serviceClient<waypoint_navigator::ToggleControl>("toggleWaypointControl");
        darknetSub = nh.subscribe("cusub_perception/darknet_ros/bounding_boxes", 1, &Orbit::darknetCallback, this);
        timer = nh.createTimer(ros::Duration(1/80), &Orbit::update, this);
        server->start();
        // initialize rostimer callback for adjusting yaw, drive and strafe (gated by controllingPids)
    }

    void Orbit::darknetCallback(const darknet_ros_msgs::BoundingBoxesConstPtr bbs)
    {
        if( !controllingPids ) { return; }

        bool vampireFound = false;
        for(darknet_ros_msgs::BoundingBox box : bbs->bounding_boxes)
        {
            // Check if we're only seeing our target vampire on the bouy
            if (box.Class == targetVampire) { vampireFound = true;}
            else if ( box.Class == "vampire_fathead" || box.Class == "vampire_flying" || box.Class == "vampire_walking") { return; }
        }
        if ( vampireFound )
        {
            if ( numberSoloFrames++ > numberSoloFramesThresh)
            {
                OrbitBuoyResult result;
                result.orbiting = true;
                server->setSucceeded(result);
            }
        }
    }

    void Orbit::update(const ros::TimerEvent& e)
    {
        if ( !controllingPids ) { return; }
        if ( server->isPreemptRequested() )
        {
            controlPids(false);
            return;
        }

        // Calculate & change yaw to face target point (I have code for this! )
        // update strafe setpoint using strafe_setpoint
        // get our distance from the point and adjust drive
        
    }

    bool Orbit::controlPids(const bool control)
    {
        waypoint_navigator::ToggleControl toggle_srv;
        toggle_srv.request.waypoint_controlling = control;
        toggle_srv.response.success = false;
        if( wayToggleClient.call(toggle_srv) ) { NODELET_INFO("Orbitter changed control of pids."); }
        else { NODELET_ERROR("Orbitter unable to change control of PIDs"); }
        controllingPids = control;
        return toggle_srv.response.success;
    }

    void Orbit::execute(const perception_control::OrbitBuoyGoalConstPtr& goal)
    {
        NODELET_INFO("Received request.");
        // adjust setpoints
        // call controlPids
    }

    void Orbit::odomCallback(const nav_msgs::OdometryConstPtr odom) { subPose = odom->pose.pose; }
    void Orbit::driveCallback(const std_msgs::Float64ConstPtr state) { driveState = state->data; }
    void Orbit::yawCallback(const std_msgs::Float64ConstPtr state) { yawState = state->data; }
    void Orbit::strafeCallback(const std_msgs::Float64ConstPtr state) { strafeState = state->data; }
}
PLUGINLIB_EXPORT_CLASS(perception_control::Orbit, nodelet::Nodelet);