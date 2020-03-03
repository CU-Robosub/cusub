#include <darknet_multiplexer/drawer.h>

namespace darknet_drawer_ns
{
    void DarknetDrawer::onInit()
    {
        if (is_nodelet)
        {
            ros::NodeHandle& nh = getMTNodeHandle();
            ros::NodeHandle nhPrivate = getPrivateNodeHandle();
            cuprint("running as a nodelet.");
            init(nh, nhPrivate);
        } else
        {
            ros::NodeHandle nh;
            ros::NodeHandle nhPrivate("~");
            cuprint("\033[95mNOT\033[0m running as nodelet");
            init(nh, nhPrivate);
        }
    }

    void DarknetDrawer::init(ros::NodeHandle& nh, ros::NodeHandle& nhPrivate)
    {
        m_darknetSub = nh.subscribe("cusub_perception/darknet_ros/bounding_boxes", 1, &DarknetDrawer::darknetCallback, this);
        m_pub = nh.advertise<sensor_msgs::Image>("cusub_perception/darknet_drawer/out",1);
        // load the config
        std::vector<std::string> detectionClasses;
        if (nhPrivate.getParam("class_names", detectionClasses))
        {
            loadClassNames(detectionClasses);
        }
    }

    void DarknetDrawer::darknetCallback(const darknet_ros_msgs::BoundingBoxesConstPtr bbs)
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
            
            cv::rectangle(cv_ptr->image, cv::Point(box.xmin,box.ymin), cv::Point(box.xmax, box.ymax), color, 2);\
            cv::putText(cv_ptr->image, box.Class + ": " + std::to_string(box.probability), cv::Point(box.xmin - 2, box.ymin - 2), cv::FONT_HERSHEY_SIMPLEX, 0.4, color);
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
    void DarknetDrawer::cuprint(std::string str)
    {
        std::string print_str = std::string("[\033[92mDrawer\033[0m] ");
        ROS_INFO( (print_str + str).c_str());
    }

    void DarknetDrawer::cuprint_warn(std::string str)
    {
        std::string print_str = std::string("[\033[92mDrawer\033[0m] ");
        print_str = print_str + std::string("\033[93m[WARN] ") + str + std::string("\033[0m");
        ROS_INFO( print_str.c_str());
    }

    void DarknetDrawer::set_not_nodelet(void)
    {
      is_nodelet = false;
    }
}

PLUGINLIB_EXPORT_CLASS(darknet_drawer_ns::DarknetDrawer, nodelet::Nodelet);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drawer_node");
  darknet_drawer_ns::DarknetDrawer d;
  d.set_not_nodelet();
  d.onInit();
  ros::spin();
}
