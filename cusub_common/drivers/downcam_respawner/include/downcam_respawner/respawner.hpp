#ifndef DOWNCAM_RESPAWNER_NODE_SRC_DOWNCAM_RESPAWNER_NODE_H_
#define DOWNCAM_RESPAWNER_NODE_SRC_DOWNCAM_RESPAWNER_NODE_H_

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <ros/timer.h>
#include <sensor_msgs/Image.h>


namespace downcam_watcher_ns
{
class Respawner : public nodelet::Nodelet
{
    public:
        virtual void onInit();
    private:
        void downcamCallback(const sensor_msgs::ImageConstPtr image);
        void timerCallback(const ros::TimerEvent&);
};
}

#endif