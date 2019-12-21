#include "detection_tree/detection_tree_node.hpp"

using namespace det_tree_ns;
using namespace std;
using namespace cv;

void DetectionTree::onInit()
{
    sub_name = "leviathan"; // TODO Pull from config
    ros::NodeHandle& nh = getMTNodeHandle();
    
    debug_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("cusub_perception/detection_tree/poses",10);
    // dvector_pub = nh.advertise<localizer::Detection>("cusub_perception/mapper/task_poses",1);        
    // Temporarily subscribe to all camera info topics
    for( auto topic_frame : camera_topic_frame_map)
    {
        cout << "Added Camera: " << topic_frame.second << endl;
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
    string frame = bbs->image_header.frame_id;
    if( camera_info.find(frame) == camera_info.end() )
    {
        // NODELET_INFO("Unreceived camera info: %s",  frame.c_str());
        return;
    }

    // Get matrices needed to calculate bearing to object
    sensor_msgs::CameraInfo ci = camera_info[bbs->image_header.frame_id];
    Mat K(3, 3, CV_64FC1, (void *) ci.K.data());
    Mat Kinv = K.inv();
    Mat D(ci.D.size(), 1, CV_64FC1, (void *) ci.D.data());

    Mat bearing_vec;
    for ( auto box : bbs->bounding_boxes)
    {
        int center_x = (box.xmax + box.xmin) / 2;
        int center_y = (box.ymax + box.ymin) / 2;
        Point2f pt(center_x, center_y);
        vector<cv::Point2f> pts = {pt};
        vector<cv::Point2f> upts;
        undistortPoints(pts, upts, K, D, cv::noArray(),  K);
        vector<double> ray_points{ upts[0].x, upts[0].y, 1 };
        Mat ray_point{3,1,cv::DataType<double>::type, ray_points.data()};
        bearing_vec = Kinv * ray_point;
    }

    // Transform bearing vector to odom frame
    std::string sub_frame = "/" + sub_name + "/description/odom";
    geometry_msgs::PoseStamped odom_cam_pose;
    size_t length = bbs->image_header.frame_id.length();
    string cam_frame = bbs->image_header.frame_id.substr(0, length-8); // trim _optical
    try{
        if( listener.waitForTransform(cam_frame, sub_frame, bbs->image_header.stamp, ros::Duration(1.0)) )
        {
            geometry_msgs::PoseStamped cam_pose;
            cam_pose.header = bbs->image_header;
            cam_pose.header.frame_id = cam_frame; // don't want "_optical" in frame id
            // Calculate the orientation in quaternions
            tf2::Quaternion cam_quat_tf;
            cout << "bearing vec: " << bearing_vec << endl;
            cv::Size s = bearing_vec.size();
            cout << "bearing vec dims: " << s << endl;
            float yaw = -bearing_vec.at<double>(0,0); // flip yaw to match odom coords
            cout << "yaw: " << yaw << endl;
            float pitch = -bearing_vec.at<double>(0,1); // flip pitch
            cout << "pitch: " << pitch << endl;
            float roll = 0;
            cam_quat_tf.setRPY( roll, pitch, yaw );
            cam_quat_tf.normalize();

            // Convert tf::quaternion to geometry_msgs::Quaternion
            cam_pose.pose.orientation = tf2::toMsg(cam_quat_tf);

            // cout << "transforming pose..." << endl;
            listener.transformPose(sub_frame, cam_pose, odom_cam_pose);
            cout << "odom pose: " << odom_cam_pose << endl;
            
            tf2::Quaternion odom_quat;
            tf2::fromMsg(odom_cam_pose.pose.orientation, odom_quat);
            tf2::Matrix3x3 m(odom_quat);
            double roll2, pitch2, yaw2;
            m.getRPY(roll2, pitch2, yaw2);
            cout << "odom yaw: " << yaw2 << endl;
            cout << "odom pitch: " << pitch2 << endl;
            cout << "--------------------" << endl;
        } else {
            NODELET_WARN("Detection Tree missed transform. Throwing out detection.");
        }
    } catch (tf::TransformException ex){
        NODELET_WARN("Detection Tree Error in transform");
    }
    debug_pose_pub.publish(odom_cam_pose);

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