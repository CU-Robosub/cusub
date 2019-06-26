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
        void occamCallback0(const sensor_msgs::ImagePtr image);
        void occamCallback1(const sensor_msgs::ImagePtr image);
        void occamCallback2(const sensor_msgs::ImagePtr image);
        void occamCallback3(const sensor_msgs::ImagePtr image);
        void occamCallback4(const sensor_msgs::ImagePtr image);
        void downcamCallback(const sensor_msgs::ImagePtr image);
        ros::NodeHandle nh;
        ros::Timer timer;
        std::vector<ros::Subscriber> subs;
        std::vector<sensor_msgs::ImagePtr> recent_images = std::vector<sensor_msgs::ImagePtr>(6);
        std::vector<bool> image_received = std::vector<bool>(6);
        ros::Publisher darknet_pub;
        std::vector<bool> activePublishers;
        int current_pub_index;
    };
}
#endif