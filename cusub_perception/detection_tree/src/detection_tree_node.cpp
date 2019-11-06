#include "detection_tree/detection_tree_node.hpp"

using namespace det_tree_ns;

void DetectionTree::onInit()
{
    sub_name = "leviathan"; // TODO Pull from config
    ros::NodeHandle& nh = getMTNodeHandle();
    
    darknet_sub = nh.subscribe("cusub_perception/darknet_ros/bounding_boxes", 1, &DetectionTree::darknetCallback, this);
    // dvector_pub = nh.advertise<localizer::Detection>("cusub_perception/mapper/task_poses",1);    
    
    // Subscribe to the camera info topics
    all_info_topics_received = false;
    for(int i = 0; i < camera_info_topics.size(); i++)
        camera_info_subs.push_back(nh.subscribe(camera_info_topics[i], 1, &DetectionTree::cameraInfoCallback, this));

    NODELET_INFO("Loaded Detection Tree");
}   

/* 
@brief Turns hits into dvectors and adds them to corresponding dobject or creates a new one
@param pointer to darknet bounding box msg
@return None
 */
void DetectionTree::darknetCallback(const darknet_ros_msgs::BoundingBoxesPtr bbs)
{
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

void DetectionTree::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr ci)
{
    std::string frame_id = ci->header.frame_id;
    NODELET_INFO("Received : %s", frame_id.c_str());
    // check in here if we've received all of the cameras & unsubscribe
}

dvector* DetectionTree::createDvector(darknet_ros_msgs::BoundingBox& bb, std_msgs::Header& image_header)
{
    ;
}


PLUGINLIB_EXPORT_CLASS(det_tree_ns::DetectionTree, nodelet::Nodelet);