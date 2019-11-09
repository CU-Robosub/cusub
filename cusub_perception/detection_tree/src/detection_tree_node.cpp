#include "detection_tree/detection_tree_node.hpp"

using namespace det_tree_ns;

void DetectionTree::onInit()
{
    sub_name = "leviathan"; // TODO Pull from config
    ros::NodeHandle& nh = getMTNodeHandle();
    
    darknet_sub = nh.subscribe("cusub_perception/darknet_ros/bounding_boxes", 1, &DetectionTree::darknetCallback, this);
    // dvector_pub = nh.advertise<localizer::Detection>("cusub_perception/mapper/task_poses",1);    
    
    // Subscribe to the camera info topics
    std::string cam_info = "/camera_info";
    int chars_to_trim = cam_info.length();

    // Temporarily subscribe to all camera info topics
    for( auto topic_frame : camera_topic_frame_map)
    {
        std::cout << "Added Camera: " << topic_frame.second << std::endl;
        camera_info_subs.insert({topic_frame.second, nh.subscribe(topic_frame.first, 1, &DetectionTree::cameraInfoCallback, this)});
    }
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

void DetectionTree::cameraInfoCallback(const sensor_msgs::CameraInfo ci)
{
    std::string frame_id = ci.header.frame_id;

    // Loop through and find corresponding camera info topic
    // record and unsubscribe from the topic
    for (auto itr_pair : camera_info_subs)
    {
        if( itr_pair.first == frame_id ) // not found
        {
            NODELET_INFO("Received : %s", frame_id.c_str());
            camera_info.insert({frame_id, ci});
            camera_info_subs[itr_pair.first].shutdown(); // unsubscribe
            break;
        }
    }
}

dvector* DetectionTree::createDvector(darknet_ros_msgs::BoundingBox& bb, std_msgs::Header& image_header)
{
    ;
}


PLUGINLIB_EXPORT_CLASS(det_tree_ns::DetectionTree, nodelet::Nodelet);