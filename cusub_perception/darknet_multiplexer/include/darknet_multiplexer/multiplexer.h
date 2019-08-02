#ifndef MULTIPLEXER_CLASS_SRC_MULTIPLEXER_CLASS_H_
#define MULTIPLEXER_CLASS_SRC_MULTIPLEXER_CLASS_H_

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <vector>
#include <darknet_multiplexer/DarknetCameras.h>
#include <std_msgs/UInt8MultiArray.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace darknet_multiplexer_ns
{
    class Multiplexer : public nodelet::Nodelet
    {
    public:
        virtual void onInit();
    private:
        bool configureActives(darknet_multiplexer::DarknetCameras::Request& request, darknet_multiplexer::DarknetCameras::Response& response);
        void publishFrame(const ros::TimerEvent& event);
        void occamCallback0(const sensor_msgs::ImagePtr image);
        void occamCallback1(const sensor_msgs::ImagePtr image);
        void occamCallback2(const sensor_msgs::ImagePtr image);
        void occamCallback3(const sensor_msgs::ImagePtr image);
        void occamCallback4(const sensor_msgs::ImagePtr image);
        void downcamCallback(const sensor_msgs::ImagePtr image);
        void torpedoCallback(const sensor_msgs::ImagePtr image);
        ros::NodeHandle nh;
        ros::Timer timer;
        ros::ServiceServer service;
        std::vector<ros::Subscriber> subs;
        std::vector<sensor_msgs::ImagePtr> recent_images = std::vector<sensor_msgs::ImagePtr>(6);
        std::vector<bool> image_received = std::vector<bool>(7);
        ros::Publisher darknet_pub;
        ros::Publisher actives_pub;
        std::vector<bool> activeCameras;
        int current_pub_index;
        bool firstImageReceived;
    };
}
#endif