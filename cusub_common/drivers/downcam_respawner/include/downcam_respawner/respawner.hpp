#ifndef DOWNCAM_RESPAWNER_NODE_SRC_DOWNCAM_RESPAWNER_NODE_H_
#define DOWNCAM_RESPAWNER_NODE_SRC_DOWNCAM_RESPAWNER_NODE_H_

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <ros/timer.h>
#include <sensor_msgs/Image.h>
#include <stdio.h>
#include <stdlib.h> 


namespace downcam_watcher_ns
{
class Respawner : public nodelet::Nodelet
{
    public:
        virtual void onInit();
    private:
        void downcamCallback(const sensor_msgs::ImageConstPtr image);
        void timerCallback(const ros::TimerEvent& event);
        void cuprint(std::string str);
        void cuprint_warn(std::string str);
        ros::Subscriber downcam_sub;
        ros::Timer timer;
        bool image_received;
        double watchdog_period;

};
}

#endif