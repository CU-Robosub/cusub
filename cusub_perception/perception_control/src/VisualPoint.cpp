#include "perception_control/VisualPoint.h"

#include <assert.h>

using namespace perception_control;

const std::string occamNamespace = "perception_control/bb_controllers/proportional/occam";

void VisualPoint::onInit()
{
    NODELET_INFO("Visual Point Server Starting up!");
    m_nh = &(getMTNodeHandle());
    yawPub = m_nh->advertise<std_msgs::Float64>("cusub_common/motor_controllers/pid/yaw/setpoint",1);
    yawSub = m_nh->subscribe("cusub_common/motor_controllers/pid/yaw/state", 1, &VisualPoint::yawCallback, this);
    NODELET_INFO("...waiting for yaw state");
    ros::topic::waitForMessage<std_msgs::Float64>("cusub_common/motor_controllers/pid/yaw/state");
    NODELET_INFO("\tgot yaw state");
    m_wayToggleClient = m_nh->serviceClient<waypoint_navigator::ToggleControl>("cusub_common/toggleWaypointControl");
    m_nh->getParam(occamNamespace + "/rotational/timeout", m_timeoutFrames);
    m_requestNum = 0;
    m_server = new vpServer(*m_nh, "visual_point", boost::bind(&VisualPoint::execute, this, _1), false);
    m_server->start();
}

void VisualPoint::darknetCallback(const darknet_ros_msgs::BoundingBoxesConstPtr bbs)
{
    std::string foundFrame;
    darknet_ros_msgs::BoundingBox foundBox;

    // Loop through boxes, find our target classes
    for(const darknet_ros_msgs::BoundingBox &box : bbs->bounding_boxes)
    {
        if (inVector(box.Class, m_activeGoal->target_classes))
        {
            foundFrame = bbs->image_header.frame_id;
            foundBox = box;
            break;
        }
    }

    VisualPointFeedback feedback;
    if ( !foundFrame.empty() )
    {
        std_msgs::Float64 new_yaw;        
        if (foundFrame == m_activeGoal->target_frame) // indicate if target class is in our target frame!
        {
            m_seenFrames++;
            new_yaw.data = yawState;
        }
        else // target class is not in target frame
        {
            std::string logMsg = "Visual point servo detected " + foundBox.Class + 
                                 " in frame " + foundFrame;
            NODELET_INFO_THROTTLE(1, "%s", logMsg.c_str());
            controlPids(true);
            std::cout << foundFrame << std::endl;
            new_yaw.data = yawState + m_occamTransforms[foundFrame];
        }
        yawPub.publish(new_yaw);
    }
    
    feedback.target_frame_count = m_seenFrames;
    m_server->publishFeedback(feedback);
}

bool VisualPoint::controlPids(const bool takeControl)
{
    bool success;
    waypoint_navigator::ToggleControl toggle_srv;
    toggle_srv.request.waypoint_controlling = !takeControl;
    toggle_srv.response.success = false;
    if( m_wayToggleClient.call(toggle_srv) )
    {
        success = true;
    }
    else
    {
        success = false;
    }
    m_controllingPids = takeControl;
    success = toggle_srv.response.success;
    return success;
}

void VisualPoint::yawCallback(const std_msgs::Float64ConstPtr state) { yawState = state->data; }

void VisualPoint::execute(const perception_control::VisualPointGoalConstPtr goal)
{
    NODELET_INFO("VisualPoint received request: %d", m_requestNum);
    m_activeGoal = goal;
    m_seenFrames = 0;
    // Subscribe to darknet with a local reference so that we unsubscribe when it goes out of scope
    ros::Subscriber darknetSub = m_nh->subscribe("cusub_perception/darknet_ros/bounding_boxes", 1, &VisualPoint::darknetCallback, this);
    NODELET_INFO("Visual Point processing request: %d", m_requestNum);
    m_requestNum += 1;
    ros::Rate r(1);
    while( ros::ok() )   // Loop until we've been preempted
    {
        if (m_server->isPreemptRequested() )
        {
            controlPids(false);
            VisualPointResult result;
            result.done = true;
            m_server->setPreempted(result);
            break;
        }
        r.sleep();
    }
}
PLUGINLIB_EXPORT_CLASS(perception_control::VisualPoint, nodelet::Nodelet);
