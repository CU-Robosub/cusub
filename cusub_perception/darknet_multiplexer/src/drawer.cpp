#include <darknet_multiplexer/drawer.h>

namespace darknet_drawer_ns
{
    void DarknetDrawer::onInit()
    {
        NODELET_INFO("Starting up Darknet Drawer");
        ros::NodeHandle& nh = getMTNodeHandle();
        darknetSub = nh.subscribe("cusub_perception/darknet_ros/bounding_boxes", 1, &DarknetDrawer::darknetCallback, this);
        pub = nh.advertise<sensor_msgs::Image>("cusub_perception/darknet_drawer/out",1);
        // load the config
    }
    void DarknetDrawer::darknetCallback(const darknet_ros_msgs::BoundingBoxesPtr bbs)
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(bbs->image, sensor_msgs::image_encodings::RGB8);
        for(darknet_ros_msgs::BoundingBox box : bbs->bounding_boxes)
        {
            // Check class and adjust color
            cv::rectangle(cv_ptr->image, cv::Point(box.xmin,box.ymin), cv::Point(box.xmax, box.ymax), cv::Scalar(255,0,0));
        }
        cv_bridge::CvImage bridge = cv_bridge::CvImage(bbs->image_header, sensor_msgs::image_encodings::RGB8, cv_ptr->image);
        pub.publish( bridge.toImageMsg() );
    }
}

 PLUGINLIB_EXPORT_CLASS(darknet_drawer_ns::DarknetDrawer, nodelet::Nodelet);