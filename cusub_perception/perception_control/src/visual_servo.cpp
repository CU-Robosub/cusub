#include <perception_control/visual_servo.h>

namespace perception_control
{
    void VisualServo::onInit()
    {
        NODELET_INFO("Visual Servo Server Starting up!");
        ros::NodeHandle& nh = getMTNodeHandle();

        // Load controllers
        proportional_controller = new BBProportional(nh);

        wayToggleClient = nh.serviceClient<waypoint_navigator::ToggleControl>("cusub_common/toggleWaypointControl");
        controllingPids = false;
        darknetSub = nh.subscribe("cusub_perception/darknet_ros/bounding_boxes", 1, &VisualServo::darknetCallback, this);
        server = new vsServer(nh, "visual_servo", boost::bind(&VisualServo::execute, this, _1), false);
        server->start();
    }
    void VisualServo::darknetCallback(const darknet_ros_msgs::BoundingBoxesConstPtr bbs)
    {
        if( !controllingPids ) { return; }
        if (bbs->image_header.frame_id != target_frame)
        { 
            NODELET_WARN_THROTTLE(1, "Wrong frame for visual servoing:\nrecieved: %s\ninstead of: %s", bbs->image_header.frame_id.c_str(), target_frame.c_str());
            return;
        }
        NODELET_INFO("...... header correct");
        for(darknet_ros_msgs::BoundingBox box : bbs->bounding_boxes)
        {
            NODELET_INFO("......... looping");
            if (box.Class == target_class)
            {
                // calculate diffs
                current_controller->respond(0,0);
                VisualServoFeedback feedback;
                feedback.centered = true;
                server->publishFeedback(feedback);
            }
        }
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

        if (goal->visual_servo_type == goal->PROPORTIONAL)
        {
            NODELET_INFO("Selecting visual servo proportional");
            current_controller = proportional_controller;
            target_class = goal->target_class;
            target_frame = goal->target_frame;
            std::cout << target_frame << std::endl;
            controlPids(true);
        } else {
            NODELET_ERROR("Unrecognized visual servo type: %d", goal->visual_servo_type);
            VisualServoResult result;
            result.success = false;
            server->setSucceeded(result);
        }
        
        ros::Duration(10).sleep();
        NODELET_INFO("Setting Success.");
        VisualServoResult result;
        result.success = true;
        server->setSucceeded(result);
        ros::Duration(1.0).sleep(); // give the task code a second to update the waypoint nav
    }
}
PLUGINLIB_EXPORT_CLASS(perception_control::VisualServo, nodelet::Nodelet);