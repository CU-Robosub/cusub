#include "detection_tree/detection_tree_node.hpp"

using namespace det_tree_ns;

void DetectionTree::onInit()
{
    sub_name = "leviathan"; // TODO Pull from config
    ros::NodeHandle& nh = getMTNodeHandle();
    
    // dvector_pub = nh.advertise<localizer::Detection>("cusub_perception/mapper/task_poses",1);        
    // Temporarily subscribe to all camera info topics
    for( auto topic_frame : camera_topic_frame_map)
    {
        std::cout << "Added Camera: " << topic_frame.second << std::endl;
        camera_info_subs.insert({topic_frame.second, nh.subscribe(topic_frame.first, 1, &DetectionTree::cameraInfoCallback, this)});
    }    
    darknet_sub = nh.subscribe("cusub_perception/darknet_ros/bounding_boxes", 1, &DetectionTree::darknetCallback, this);
    NODELET_INFO("Loaded Detection Tree");
}   

/* 
@brief Turns hits into dvectors and adds them to corresponding dobject or creates a new one
        Publishes dvector to all listeners
@param pointer to darknet bounding box msg
@return None
 */
void DetectionTree::darknetCallback(const darknet_ros_msgs::BoundingBoxesConstPtr bbs)
{
    std::string frame = bbs->image_header.frame_id;
    if( camera_info.find(frame) == camera_info.end() )
    {
        // NODELET_INFO("Unreceived camera info: %s",  frame.c_str());
        return;
    }

    // Get matrices needed to calculate bearing to object
    sensor_msgs::CameraInfo ci = camera_info[bbs->image_header.frame_id];
    cv::Mat K(3, 3, CV_64FC1, (void *) ci.K.data());
    cv::Mat Kinv = K.inv();
    cv::Mat D(ci.D.size(), 1, CV_64FC1, (void *) ci.D.data());

    for ( auto box : bbs->bounding_boxes)
    {
        int center_x = (box.xmax + box.xmin) / 2;
        int center_y = (box.ymax + box.ymin) / 2;
        cv::Point2f pt(center_x, center_y);
        // std::cout << "Points: " << pt << std::endl;
        std::vector<cv::Point2f> pts = {pt};
        std::cout << "pts: " << pts[0] << std::endl;
        std::vector<cv::Point2f> upts;
        cv::undistortPoints(pts, upts, K, D, cv::noArray(),  K);
        std::cout << "upts: " << upts[0] << std::endl;
        std::vector<double> ray_points{ upts[0].x, upts[0].y, 1 };
        cv::Mat ray_point{3,1,cv::DataType<double>::type, ray_points.data()};
        std::cout << "ray pt: " << ray_point << std::endl;
        // std::vector<cv::Vec3f> homoPts;
        // cv::convertPointsToHomogeneous(upts, homoPts);
        // std::cout << "homoPts: " << homoPts[0] << std::endl;
        // cv::Mat homoVec(3, 1, CV_32FC1, (void *) homoPts[0].data());
        // cv::Mat homoVec(3, 1, CV_32FC1, homoPts[0]);
        std::cout << "K: " << K << std::endl;
        std::cout << "Kinv: " << K.inv() << std::endl;
        cv::Mat bearing_vec = Kinv * ray_point;
        std::cout << "Class: " << box.Class << std::endl;
        std::cout << "M " << std::endl << bearing_vec << std::endl;
    }
    // std::string sub_frame = "/" + sub_name + "/description/odom";
    // try{
    //     if( listener.waitForTransform(bbs->image_header.frame_id,
    //         sub_frame, bbs->image_header.stamp, ros::Duration(1.0)) )
    //     {
    //         tf::Stamped<tf::Pose> sub_pose;
    //         tf::Stamped<tf::Pose> camera_pose();
    //         tf::Pose odom_pose;
    //         listener.transformPose(sub_frame, )
    //     } else {
    //         NODELET_WARN("Detection Tree missed transform");
    //     }
    // } catch (tf::TransformException ex){
    //     NODELET_WARN("Detection Tree Error in transform");
    // }

    // do tf lookup based on camera's time stamp
    // create new dvector (use new keyword)
    // determine dobject
    // ? create new dobject
    // add dvector to dobject
    // check if we can pose solve the dobject
    ;
}

void DetectionTree::cameraInfoCallback(const sensor_msgs::CameraInfo ci)
{
    std::string frame_id = ci.header.frame_id;

    // Loop through and find corresponding camera info topic
    // record and unsubscribe from the topic
    for (auto itr_pair : camera_info_subs)
    {
        if( itr_pair.first == frame_id )
        {
            NODELET_INFO("Received : %s", frame_id.c_str());
            // TODO remove on actual sub

            camera_info.insert({frame_id + "_optical", ci}); // adding _optical is 100% a hack
            camera_info_subs[itr_pair.first].shutdown(); // unsubscribe
            if ( camera_info_subs.size() == camera_info.size() )
            {
                NODELET_INFO("All camera's info acquired");
            }
            return;
        }
    }
    NODELET_WARN("Unrecognized camera frame: %s", frame_id.c_str());
}

dvector* DetectionTree::createDvector(darknet_ros_msgs::BoundingBox& bb, std_msgs::Header& image_header)
{
    ;
}


PLUGINLIB_EXPORT_CLASS(det_tree_ns::DetectionTree, nodelet::Nodelet);