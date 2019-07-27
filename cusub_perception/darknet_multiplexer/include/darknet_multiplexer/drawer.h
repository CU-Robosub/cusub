#ifndef DARKNET_DRAWER_CLASS_SRC_DARKNET_DRAWER_CLASS_H_
#define DARKNET_DRAWER_CLASS_SRC_DARKNET_DRAWER_CLASS_H_

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace darknet_drawer_ns
{
    class DarknetDrawer : public nodelet::Nodelet
    {
    public:
        virtual void onInit();
    private:
        void darknetCallback(const darknet_ros_msgs::BoundingBoxesConstPtr bbs);
        ros::Subscriber m_darknetSub;
        ros::Publisher m_pub;

        std::map<std::string, cv::Scalar> m_classColors;
        void loadClassNames(const std::vector<std::string> &classNames);
    };
}

#endif
