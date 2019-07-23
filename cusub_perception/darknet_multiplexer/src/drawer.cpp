#include <darknet_multiplexer/drawer.h>

namespace darknet_drawer_ns
{
    void DarknetDrawer::onInit()
    {
        NODELET_INFO("Starting up Darknet Drawer");
        ros::NodeHandle& nh = getMTNodeHandle();
        m_darknetSub = nh.subscribe("/leviathan/cusub_perception/darknet_ros/bounding_boxes", 1, &DarknetDrawer::darknetCallback, this);
        m_pub = nh.advertise<sensor_msgs::Image>("cusub_perception/darknet_drawer/out",1);
        // load the config
        std::vector<std::string> detectionClasses;
        ros::NodeHandle nhPrivate = getPrivateNodeHandle();
        if (nhPrivate.getParam("class_names", detectionClasses))
        {
            loadClassNames(detectionClasses);
        }
    }
    void DarknetDrawer::darknetCallback(const darknet_ros_msgs::BoundingBoxesPtr bbs)
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(bbs->image, sensor_msgs::image_encodings::RGB8);
        for(darknet_ros_msgs::BoundingBox box : bbs->bounding_boxes)
        {
            // Check class and adjust color
            std::map<std::string, cv::Scalar>::iterator iter = m_classColors.find(box.Class);
            cv::Scalar color;
            if (iter != m_classColors.end())
            {
                color = iter->second;
            }
            else
            {
                color = cv::Scalar(0,0,255);
            }
            
            cv::rectangle(cv_ptr->image, cv::Point(box.xmin,box.ymin), cv::Point(box.xmax, box.ymax), color);\
            cv::putText(cv_ptr->image, box.Class, cv::Point(box.xmin - 2, box.ymin - 2), cv::FONT_HERSHEY_SIMPLEX, 0.4, color);
        }
        cv_bridge::CvImage bridge = cv_bridge::CvImage(bbs->image_header, sensor_msgs::image_encodings::RGB8, cv_ptr->image);
        m_pub.publish( bridge.toImageMsg() );
    
    }
    void DarknetDrawer::loadClassNames(const std::vector<std::string> &classNames)
    {
        cv::RNG rng(12345);
        for (const std::string className : classNames)
        {
            // generate random color
            cv::Scalar color(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            m_classColors.insert(std::make_pair(className, color));
        }
    }
}

 PLUGINLIB_EXPORT_CLASS(darknet_drawer_ns::DarknetDrawer, nodelet::Nodelet);
