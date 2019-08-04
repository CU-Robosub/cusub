#include "perception_control/VisualPoint.h"

#include <assert.h>

using namespace perception_control;

const std::string occamNamespace = "perception_control/bb_controllers/proportional/occam";

void VisualPoint::onInit()
{

    NODELET_INFO("Visual Point Server Starting up!");
    m_nh = &(getMTNodeHandle());

    initializeMovement(m_nh);
    
    m_nh->getParam(occamNamespace + "/rotational/timeout", m_timeoutFrames);
    
    m_wayToggleClient = m_nh->serviceClient<waypoint_navigator::ToggleControl>("cusub_common/toggleWaypointControl");
    m_requestNum = 0;
    m_server = new vpServer(*m_nh, "visual_point", boost::bind(&VisualPoint::execute, this, _1), false);
    m_server->start();
}


void VisualPoint::darknetCallback(const darknet_ros_msgs::BoundingBoxesConstPtr bbs)
{
    std::string foundFrame;
    darknet_ros_msgs::BoundingBox foundBox;
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
    // found the frame
    if (foundFrame.empty() == false)
    {
        if (foundFrame == m_activeGoal->target_frame)
        {
            m_seenFrames++;
        }
        else
        {
            std::string logMsg = "Visual point servo detected " + foundBox.Class + 
                                 " in frame " + foundFrame;
            NODELET_INFO("%s", logMsg.c_str());
            controlPids(true);
            m_targetYaw.active = true;
            m_targetYaw.fromFrame = foundFrame;
            m_targetYaw.value = m_currentController->getTargetYaw(bbs->image_header.frame_id);
            NODELET_INFO("%f", m_targetYaw.value);
        }
    }

    if (m_targetYaw.active)
    {
        m_currentController->targetYaw(m_targetYaw.value);
        m_targetYaw.attemptedFrames++;

        if (m_targetYaw.attemptedFrames > m_timeoutFrames)
        {
            m_seenFrames = -1;
            m_targetYaw = TargetYaw();
            controlPids(false);
        }
    }

    feedback.target_frame_count = m_seenFrames;
    m_server->publishFeedback(feedback);
}

void VisualPoint::execute(const perception_control::VisualPointGoalConstPtr goal)
{
    // error catching will come with better base class Impl.
    m_requestNum += 1;
    NODELET_INFO("VisualPoint received request: %d", m_requestNum);
    m_activeGoal = goal;
    // todo SK make more dyanmic
    NODELET_INFO("Selecting visual control: ROTATIONAL");
    m_currentController = m_rotationalController;

    m_seenFrames = 0;
    m_targetYaw = TargetYaw();
    
    // Subscribe to darknet with a local reference so that we unsubscribe when it goes out of scope
    ros::Subscriber darknetSub = m_nh->subscribe("cusub_perception/darknet_ros/bounding_boxes", 1, &VisualPoint::darknetCallback, this);

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
