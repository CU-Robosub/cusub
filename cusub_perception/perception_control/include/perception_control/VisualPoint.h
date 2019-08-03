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

#include "VisualMovement.h"

namespace perception_control
{

typedef actionlib::SimpleActionServer<perception_control::VisualPointAction> vpServer;

class VisualPoint : public nodelet::Nodelet, VisualMovement
{
public:
    virtual void onInit();
private:
    virtual void darknetCallback(const darknet_ros_msgs::BoundingBoxesConstPtr bbs);
    virtual void execute(const perception_control::VisualPointGoalConstPtr goal);

    ros::NodeHandle* m_nh;
    int m_requestNum;
    perception_control::VisualPointGoalConstPtr m_activeGoal;
    vpServer* m_server;


    std::string activeCamera;
    int m_seenFrames;
    struct TargetYaw
    {
        int attemptedFrames;
        bool active;
        float value;
        std::string fromFrame;
        TargetYaw()
        {
            active = false;
            fromFrame = "";
            value = 0.0;
            attemptedFrames = 0;
        }
    } m_targetYaw;
    
    int m_timeoutFrames;

    template<typename T>
    bool inVector(const T &target, std::vector<T> vector) { return std::find(vector.begin(), vector.end(), target) != vector.end(); }
};

}; // neamespace perception_control

#endif // VISUALPOINT_H