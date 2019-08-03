/**
 * @file VisualPoint.h
 * @author Soroush Khadem (soroush.khadem@colorado.edu)
 * @brief 
 * 
 * 
 * 
 */
#ifndef VISUALPOINT_H
#define VISUALPOINT_H

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <actionlib/server/simple_action_server.h>
#include <perception_control/VisualPointAction.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <std_msgs/Float64.h>
#include <cstdint>
#include <waypoint_navigator/ToggleControl.h>

namespace perception_control
{

typedef actionlib::SimpleActionServer<perception_control::VisualPointAction> vpServer;

class VisualPoint : public nodelet::Nodelet
{
public:
    void onInit();
private:
    void darknetCallback(const darknet_ros_msgs::BoundingBoxesConstPtr bbs);
    void execute(const perception_control::VisualPointGoalConstPtr goal);
    bool controlPids(const bool takeControl);
    void yawCallback(const std_msgs::Float64ConstPtr state);

    float yawState;
    ros::Publisher yawPub;
    ros::Subscriber yawSub;

    ros::NodeHandle* m_nh;
    int m_requestNum;
    perception_control::VisualPointGoalConstPtr m_activeGoal;
    vpServer* m_server;
    ros::ServiceClient m_wayToggleClient; 
    bool m_controllingPids;
    int m_timeoutFrames;
    int m_seenFrames;

    std::map<std::string, float> m_occamTransforms = 
                        { {"leviathan/description/occam0_frame_optical", 0},
                        { "leviathan/description/occam1_frame_optical", -1.2566 },
                        { "leviathan/description/occam2_frame_optical", -2.5132 },
                        { "leviathan/description/occam3_frame_optical", -3.7698 },
                        { "leviathan/description/occam4_frame_optical", -5.0264 }
    };
    template<typename T>
    bool inVector(const T &target, std::vector<T> vector) { return std::find(vector.begin(), vector.end(), target) != vector.end(); }
};

}; // neamespace perception_control

#endif // VISUALPOINT_H