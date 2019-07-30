#include <perception_control/visual_servo.h>

namespace perception_control
{
    void VisualServo::onInit()
    {
        NODELET_INFO("Visual Servo Server Starting up!");
        ros::NodeHandle& nh = getMTNodeHandle();

        // Load controllers
        proportional_controller = new BBProportional(nh);

        frozen_controls = false;
        wayToggleClient = nh.serviceClient<waypoint_navigator::ToggleControl>("cusub_common/toggleWaypointControl");
        controllingPids = false;
        darknetSub = nh.subscribe("cusub_perception/darknet_ros/bounding_boxes", 1, &VisualServo::darknetCallback, this);
        server = new vsServer(nh, "visual_servo", boost::bind(&VisualServo::execute, this, _1), false);
        server->start();
    }
    void VisualServo::darknetCallback(const darknet_ros_msgs::BoundingBoxesConstPtr bbs)
    {
        if( !controllingPids ) { return; }

        // Locate target box if in image
        darknet_ros_msgs::BoundingBox target_box;
        bool box_found = false;
        for(darknet_ros_msgs::BoundingBox box : bbs->bounding_boxes)
        {
            if (box.Class == target_class && !box_found)
            {
                box_found = true;
                target_box = box;
            } else if (box.Class == target_class )
            {
                NODELET_WARN_THROTTLE(1, "Multiple Boxes of the target class given in the same image! Choosing the closest to our target goal");
                int center_x1 = (box.xmax + box.xmin) / 2;
                int center_y1 = (box.ymax + box.ymin) / 2;
                int center_x2 = (target_box.xmax + target_box.xmin) / 2;
                int center_y2 = (target_box.ymax + target_box.ymin) / 2;

                int error_x1 = center_x1 - target_pixel_x;
                int error_y1 = center_y1 - target_pixel_y;
                int error_x2 = center_x2 - target_pixel_x;
                int error_y2 = center_y2 - target_pixel_y;

                if (std::abs(error_x1) + std::abs(error_y1) < std::abs(error_x2) + std::abs(error_y2)) // switch the target box
                    target_box = box;
                // else don't switch the target_box
            }
        }

        if (box_found)
        {
            if (bbs->image_header.frame_id != target_frame) // Check that the box was in our target frame
            { 
                NODELET_WARN_THROTTLE(1, "Wrong frame for visual servoing:\nrecieved: %s\ninstead of: %s", bbs->image_header.frame_id.c_str(), target_frame.c_str());
                return;
            }
            int center_x = (target_box.xmax + target_box.xmin) / 2;
            int center_y = (target_box.ymax + target_box.ymin) / 2;
            int error_x = center_x - target_pixel_x;
            int error_y = center_y - target_pixel_y;

            VisualServoFeedback feedback;
            if ( (std::abs(error_x) < target_pixel_threshold / 2) && (std::abs(error_y) < target_pixel_threshold / 2) ) // we're centered
            {
                feedback.centered = true; 
            }
            if ( (std::abs(error_x) < target_pixel_threshold / 4) && (std::abs(error_y) < target_pixel_threshold / 4) ) // we're centered
            {
                if ( !frozen_controls ) // we just centered on our target
                {
                    NODELET_INFO_THROTTLE(2, "Freezing controls.");
                    frozen_x_set.data = *current_controller->x_state;
                    frozen_y_set.data = *current_controller->y_state;
                    frozen_controls = true;
                } // else we've been centered on the target
                current_controller->x_pub->publish(frozen_x_set);
                current_controller->y_pub->publish(frozen_y_set);
            } else { // Still need to center
                NODELET_INFO_THROTTLE(1, "Servoing x_error: %d\ty_error: %d", error_x, error_y);
                frozen_controls = false;
                feedback.centered = false;
                current_controller->respond(error_x,error_y);
            }
            server->publishFeedback(feedback);
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
            NODELET_INFO("Selecting visual servo: PROPORTIONAL");
            if (goal->camera != goal->OCCAM && goal->camera != goal->DOWNCAM)
            {
                NODELET_ERROR("Unrecognized camera: %s", goal->camera.c_str());
                abort();
            }
            proportional_controller->configureByCamera(goal->camera);
            current_controller = proportional_controller;            
            
            // TODO read which axes to configure and adjust
            current_controller->configureAxes( (AxisConfig) goal->x_axis, (AxisConfig) goal->y_axis);
            target_class = goal->target_class;
            target_frame = goal->target_frame;
            target_pixel_x = goal->target_pixel_x;
            target_pixel_y = goal->target_pixel_y;
            target_pixel_threshold = goal->target_pixel_threshold;
            controlPids(true);
        } else {
            NODELET_ERROR("Unrecognized visual servo type: %d", goal->visual_servo_type);
            VisualServoResult result;
            result.success = false;
            server->setSucceeded(result);
        }
        ros::Rate r(1);
        while( ros::ok() )   // Loop until we've been preempted
        {
            if (server->isPreemptRequested() )
            {
                controlPids(false);
                VisualServoResult result;
                result.success = true;
                server->setPreempted(result);
                break;
            }
            r.sleep();
        }
    }
}
PLUGINLIB_EXPORT_CLASS(perception_control::VisualServo, nodelet::Nodelet);