#include <perception_control/visual_servo.h>

#include <assert.h>

namespace perception_control
{
void VisualServo::onInit()
{
    NODELET_INFO("Visual Servo Server Starting up!");
    nh = &(getMTNodeHandle());

    // Load controllers
    proportional_controller = new BBProportional(*nh);

    frozen_controls = false;
    wayToggleClient = nh->serviceClient<waypoint_navigator::ToggleControl>("cusub_common/toggleWaypointControl");
    controllingPids = false;
    server = new vsServer(*nh, "visual_servo", boost::bind(&VisualServo::execute, this, _1), false);
    server->start();
}

bool VisualServo::getTarget(const std::vector<darknet_ros_msgs::BoundingBox> &boxes, darknet_ros_msgs::BoundingBox &targetBox)
{
    bool box_found = false;
    std::map<std::string, std::vector<darknet_ros_msgs::BoundingBox> > foundTargets;
    for (std::string classname : target_classes)
    {
        foundTargets.insert(std::make_pair(classname, std::vector<darknet_ros_msgs::BoundingBox>{}));
    }
    for(const darknet_ros_msgs::BoundingBox &box : boxes)
    {
        bool is_target_class = std::find(target_classes.begin(), target_classes.end(), box.Class) != target_classes.end();
        
        if (is_target_class)
        {
            // first one
            if (foundTargets[box.Class].size() == 0)
            {
                foundTargets[box.Class].push_back(box);
            }
            // coffin should have both boxes
            else if (box.Class == "coffin")
            {
                foundTargets[box.Class].push_back(box);
                std::cout << "Adding multiple coffins!" << std::endl;
            }
            else // we only want one class for everything else
            {
                targetBox = foundTargets[box.Class].at(0);
                NODELET_WARN_THROTTLE(1, "Multiple Boxes of the target class given in the same image! Choosing the closest to our target goal");
                int center_x1 = (box.xmax + box.xmin) / 2;
                int center_y1 = (box.ymax + box.ymin) / 2;
                int center_x2 = (targetBox.xmax + targetBox.xmin) / 2;
                int center_y2 = (targetBox.ymax + targetBox.ymin) / 2;

                int error_x1 = center_x1 - target_pixel_x;
                int error_y1 = center_y1 - target_pixel_y;
                int error_x2 = center_x2 - target_pixel_x;
                int error_y2 = center_y2 - target_pixel_y;

                if (std::abs(error_x1) + std::abs(error_y1) < std::abs(error_x2) + std::abs(error_y2)) // switch the target box
                {
                    foundTargets[box.Class].at(0) = box;
                }
                // else don't switch the target_box
            }
        }
    }

    bool foundBox = false;

    int xmin = 1e9, ymin = 1e9;
    int xmax = -1e9, ymax = -1e9;
    for (auto iter : foundTargets)
    {
        for (darknet_ros_msgs::BoundingBox box : iter.second) // each will have 1
        {
            foundBox = true;

            if (box.xmax > xmax)
            {
                xmax = box.xmax;
            }
            if (box.xmin < xmin)
            {
                xmin = box.xmin;
            }
            if (box.ymax > ymax)
            {
                ymax = box.ymax;
            }
            if (box.ymin < ymin)
            {
                ymin = box.ymin;
            }
        }
    }

    targetBox.xmin = xmin;
    targetBox.xmax = xmax;
    targetBox.ymin = ymin;
    targetBox.ymax = ymax;

    return foundBox;
}

void VisualServo::darknetCallback(const darknet_ros_msgs::BoundingBoxesConstPtr bbs)
{
    if( !controllingPids ) { return; }

    // Locate target box if in image
    darknet_ros_msgs::BoundingBox target_box;
    bool box_found = getTarget(bbs->bounding_boxes, target_box);

    if (box_found)
    {
        if (bbs->image_header.frame_id != target_frame) // Check that the box was in our target frame
        { 
            NODELET_WARN_THROTTLE(1, "Wrong frame for visual servoing:\nrecieved: %s\ninstead of: %s", bbs->image_header.frame_id.c_str(), target_frame.c_str());
            return;
        }
        int center_x = (target_box.xmax + target_box.xmin) / 2;
        int center_y = (target_box.ymax + target_box.ymin) / 2;
        int box_area = (target_box.xmax - target_box.xmin) * (target_box.ymax - target_box.ymin);
        float error_x = (float) ( center_x - target_pixel_x );
        float error_y = (float) ( center_y - target_pixel_y );
        float error_area = std::sqrt(target_box_area) - std::sqrt(box_area);
        NODELET_INFO_THROTTLE(1, "Servoing x_error: %f\ty_error: %f\tarea_error: %f", error_x, error_y, std::sqrt(std::abs(error_area)));

        bool x_centered=false, y_centered=false, area_centered=false;
        // RESPOND X
        if (std::abs(error_x) < target_pixel_threshold / 4)
        {
            x_centered = true;
            respondError(X_AXIS, 0); // make the error zero to induce no movement
        } else {
            if (std::abs(error_x) < target_pixel_threshold / 2)
            {
                x_centered = true;
            }
            x_centered = respondError(X_AXIS, error_x);
        }

        // RESPOND Y
        if (std::abs(error_y) < target_pixel_threshold / 4)
        {
            y_centered = true;
            respondError(Y_AXIS, 0); // make the error zero to induce no movement
        } else {
            if (std::abs(error_y) < target_pixel_threshold / 2)
            {
                y_centered = true;
            }
            y_centered = respondError(Y_AXIS, error_y);
        }

        // RESPOND Area
        if (std::abs(error_area) < target_pixel_threshold / 4)
        {
            area_centered = true;
            respondError(AREA_AXIS, 0); // make the error zero to induce no movement
        } else {
            if (std::abs(error_area) < target_pixel_threshold / 2)
            {
                area_centered = true;
            }
            area_centered = respondError(AREA_AXIS, error_area);
        }

        VisualServoFeedback feedback;
        if ( ( x_centered && y_centered ) && area_centered )
        {
            feedback.centered = true;
        } else { feedback.centered = false; }

        server->publishFeedback(feedback);
    }
}
bool VisualServo::respondError(ImageAxis image_axis, float error)
{
    if( activeCamera == perception_control::VisualServoGoal::OCCAM)
    {
        return respondOccamError(image_axis, error);
    } else if( activeCamera == perception_control::VisualServoGoal::DOWNCAM) {
        return respondDowncamError(image_axis, error);
    } else {
        NODELET_ERROR("Unrecognized camera!");
        abort();
    }
}

bool VisualServo::respondDowncamError(ImageAxis image_axis, float error)
{
    switch( image_axis )
    {
        case X_AXIS:
            switch(x_map_axis)
            {
            case perception_control::VisualServoGoal::NO_AXIS:
                return true;
                break;
            case perception_control::VisualServoGoal::DRIVE_AXIS:
                current_controller->respondDowncamDrive(error);
                break;
            case perception_control::VisualServoGoal::STRAFE_AXIS:
                current_controller->respondDowncamStrafe(error);
                break;
            case perception_control::VisualServoGoal::YAW_AXIS:
                current_controller->respondDowncamYaw(error);
                break;
            case perception_control::VisualServoGoal::DEPTH_AXIS:
                current_controller->respondDowncamDepth(error);
                break;
            default:
                ROS_ERROR("Unrecognized AxisConfig!");
                abort();
            }
            break;
        case Y_AXIS:
            switch(y_map_axis)
            {
            case perception_control::VisualServoGoal::NO_AXIS:
                return true;
                break;
            case perception_control::VisualServoGoal::DRIVE_AXIS:
                current_controller->respondDowncamDrive(error);
                break;
            case perception_control::VisualServoGoal::STRAFE_AXIS:
                current_controller->respondDowncamStrafe(error);
                break;
            case perception_control::VisualServoGoal::YAW_AXIS:
                current_controller->respondDowncamYaw(error);
                break;
            case perception_control::VisualServoGoal::DEPTH_AXIS:
                current_controller->respondDowncamDepth(error);
                break;
            default:
                ROS_ERROR("Unrecognized AxisConfig!");
                abort();
            }
            break;
        case AREA_AXIS:
            areaResponse = true;
            switch(area_map_axis)
            {
            case perception_control::VisualServoGoal::NO_AXIS:
                return true;
                break;
            case perception_control::VisualServoGoal::DRIVE_AXIS:
                current_controller->respondDowncamDrive(error);
                break;
            case perception_control::VisualServoGoal::STRAFE_AXIS:
                current_controller->respondDowncamStrafe(error);
                break;
            case perception_control::VisualServoGoal::YAW_AXIS:
                current_controller->respondDowncamYaw(error);
                break;
            case perception_control::VisualServoGoal::DEPTH_AXIS:
                current_controller->respondDowncamDepth(error);
                break;
            default:
                ROS_ERROR("Unrecognized AxisConfig!");
                abort();
                break;
            }
            break;
        default:
            NODELET_ERROR("Visual Servo didn't recognize requested error axis.");
            abort();
            break;
    }
    return false;
}

bool VisualServo::respondOccamError(ImageAxis axis, float error)
{
    switch( axis )
    {
        case X_AXIS:
            switch(x_map_axis)
            {
            case perception_control::VisualServoGoal::NO_AXIS:
                return true;
                break;
            case perception_control::VisualServoGoal::DRIVE_AXIS:
                current_controller->respondOccamDrive(error);
                break;
            case perception_control::VisualServoGoal::STRAFE_AXIS:
                current_controller->respondOccamStrafe(error);
                break;
            case perception_control::VisualServoGoal::YAW_AXIS:
                current_controller->respondOccamYaw(error);
                break;
            case perception_control::VisualServoGoal::DEPTH_AXIS:
                current_controller->respondOccamDepth(error);
                break;
            default:
                ROS_ERROR("Unrecognized AxisConfig!");
                abort();
            }
            break;
        case Y_AXIS:
            switch(y_map_axis)
            {
            case perception_control::VisualServoGoal::NO_AXIS:
                return true;
                break;
            case perception_control::VisualServoGoal::DRIVE_AXIS:
                current_controller->respondOccamDrive(error);
                break;
            case perception_control::VisualServoGoal::STRAFE_AXIS:
                current_controller->respondOccamStrafe(error);
                break;
            case perception_control::VisualServoGoal::YAW_AXIS:
                current_controller->respondOccamYaw(error);
                break;
            case perception_control::VisualServoGoal::DEPTH_AXIS:
                current_controller->respondOccamDepth(error);
                break;
            default:
                ROS_ERROR("Unrecognized AxisConfig!");
                abort();
            }
            break;
        case AREA_AXIS:
            switch(area_map_axis)
            {
            case perception_control::VisualServoGoal::NO_AXIS:
                return true;
                break;
            case perception_control::VisualServoGoal::DRIVE_AXIS:
                current_controller->respondOccamDrive(error);
                break;
            case perception_control::VisualServoGoal::STRAFE_AXIS:
                current_controller->respondOccamStrafe(error);
                break;
            case perception_control::VisualServoGoal::YAW_AXIS:
                current_controller->respondOccamYaw(error);
                break;
            case perception_control::VisualServoGoal::DEPTH_AXIS:
                current_controller->respondOccamDepth(error);
                break;
            default:
                ROS_ERROR("Unrecognized AxisConfig!");
                abort();
                break;
            }
            break;
        default:
            NODELET_ERROR("Visual Servo didn't recognize requested error axis.");
            abort();
            break;
    }
    return false;
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
        current_controller = proportional_controller;
    } else {
        NODELET_ERROR("Unrecognized visual servo type: %d", goal->visual_servo_type);
        VisualServoResult result;
        result.success = false;
        server->setSucceeded(result);
        return;
    }
    
    if (goal->camera != goal->OCCAM && goal->camera != goal->DOWNCAM)
    {
        NODELET_ERROR("Unrecognized camera: %s", goal->camera.c_str());
        abort();
    } else {
        activeCamera = goal->camera;
    }
    x_map_axis = (ImageAxis) goal->x_axis;
    y_map_axis = (ImageAxis) goal->y_axis;
    area_map_axis = (ImageAxis) goal->area_axis;

    target_classes = goal->target_classes;
    target_frame = goal->target_frame;
    target_pixel_x = goal->target_pixel_x;
    target_pixel_y = goal->target_pixel_y;
    target_box_area = goal->target_box_area;
    target_pixel_threshold = goal->target_pixel_threshold;
    controlPids(true);

    // Subscribe to darknet with a local reference so that we unsubscribe when it goes out of scope
    ros::Subscriber darknetSub = nh->subscribe("cusub_perception/darknet_ros/bounding_boxes", 1, &VisualServo::darknetCallback, this);

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