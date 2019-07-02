#include <perception_control/orbit.h>

namespace perception_control
{
    void Orbit::onInit()
    {
        NODELET_INFO("Orbit Server Starting up!");
        numberSoloFrames = 0;
        ros::NodeHandle& nh = getMTNodeHandle();
        odomSub = nh.subscribe("cusub_common/odometry/filtered", 1, &Orbit::odomCallback, this);
        driveSub = nh.subscribe("cusub_common/motor_controllers/pid/drive/state", 1, &Orbit::driveCallback, this);
        yawSub = nh.subscribe("cusub_common/motor_controllers/pid/yaw/state", 1, &Orbit::yawCallback, this);
        strafeSub = nh.subscribe("cusub_common/motor_controllers/pid/strafe/state", 1, &Orbit::strafeCallback, this);
        drivePub = nh.advertise<std_msgs::Float64>("cusub_common/motor_controllers/pid/drive/setpoint",1);
        yawPub = nh.advertise<std_msgs::Float64>("cusub_common/motor_controllers/pid/yaw/setpoint",1);
        strafePub = nh.advertise<std_msgs::Float64>("cusub_common/motor_controllers/pid/strafe/setpoint",1);
        server = new orbitServer(nh, "orbit_buoy", boost::bind(&Orbit::execute, this, _1), false);
        wayToggleClient = nh.serviceClient<waypoint_navigator::ToggleControl>("cusub_common/toggleWaypointControl");
        darknetSub = nh.subscribe("cusub_perception/darknet_ros/bounding_boxes", 1, &Orbit::darknetCallback, this);
        timer = nh.createTimer(ros::Duration(1/80), &Orbit::update, this);
        server->start();
    }

    Orbit::~Orbit()
    {
        controlPids(false);
    }

    void Orbit::darknetCallback(const darknet_ros_msgs::BoundingBoxesConstPtr bbs)
    {
        if( !controllingPids ) { return; }

        bool vampireFound = false;
        for(darknet_ros_msgs::BoundingBox box : bbs->bounding_boxes)
        {
            // Check if we're only seeing our target vampire on the bouy
            if (box.Class == activeGoal->target_class) { vampireFound = true;}
            else if ( box.Class == "vampire_fathead" || box.Class == "vampire_flying" || box.Class == "vampire_walking") { return; }
        }
        if ( vampireFound )
        {
            if ( numberSoloFrames++ > activeGoal->number_solo_frames)
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

        // YAW
        std_msgs::Float64Ptr yawSetpt(new std_msgs::Float64);
        float dx = activeGoal->buoy_pose.position.x - subPose.position.x;
        float dy = activeGoal->buoy_pose.position.y - subPose.position.y;
        yawSetpt->data = std::atan2(dy, dx);

        // STRAFE
        std_msgs::Float64Ptr strafeSetpt(new std_msgs::Float64);
        float strafeCarrot;
        if (activeGoal->orbit_left) { strafeCarrot = - std::abs(activeGoal->strafe_setpoint); }
        else { strafeCarrot = std::abs(activeGoal->strafe_setpoint); }
        strafeSetpt->data = strafeState + strafeCarrot;

        // DRIVE
        std_msgs::Float64Ptr driveSetpt(new std_msgs::Float64);
        float xy_dist = std::sqrt(std::pow(dx,2) + std::pow(dy,2));
        driveSetpt->data = driveState - (activeGoal->orbit_radius - xy_dist);

        yawPub.publish(yawSetpt);
        strafePub.publish(strafeSetpt);
        drivePub.publish(driveSetpt);
    }

    bool Orbit::controlPids(const bool takeControl)
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

    void Orbit::execute(const perception_control::OrbitBuoyGoalConstPtr goal)
    {
        NODELET_INFO("Orbitter received request.");
        activeGoal = goal;
        controlPids(true);
        // Loop here and check for preempts or darknet successes
        ros::Rate r(10);
        while ( ros::ok() )
        {
            if (server->isPreemptRequested() )
            {
                controlPids(false);
                OrbitBuoyResult result;
                result.orbiting = true;
                server->setPreempted(result);
                break;
            }
            r.sleep();
        }
    }

    void Orbit::odomCallback(const nav_msgs::OdometryConstPtr odom) { subPose = odom->pose.pose; }
    void Orbit::driveCallback(const std_msgs::Float64ConstPtr state) { driveState = state->data; }
    void Orbit::yawCallback(const std_msgs::Float64ConstPtr state) { yawState = state->data; }
    void Orbit::strafeCallback(const std_msgs::Float64ConstPtr state) { strafeState = state->data; }
}
PLUGINLIB_EXPORT_CLASS(perception_control::Orbit, nodelet::Nodelet);