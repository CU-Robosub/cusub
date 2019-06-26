#ifndef MULTIPLEXER_CLASS_SRC_MULTIPLEXER_CLASS_H_
#define MULTIPLEXER_CLASS_SRC_MULTIPLEXER_CLASS_H_

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <vector>

namespace darknet_multiplexer_ns
{
    class Multiplexer : public nodelet::Nodelet
    {
    public:
        virtual void onInit();
    private:
        void publishFrame(const ros::TimerEvent& event);
        void cameraCallback(const sensor_msgs::ImagePtr image);
        ros::NodeHandle nh;
        std::vector<ros::Subscriber> subs;
        ros::Timer timer;
    };
}
#endif