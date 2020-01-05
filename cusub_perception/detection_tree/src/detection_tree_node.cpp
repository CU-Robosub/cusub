#include "detection_tree/detection_tree_node.hpp"

using namespace det_tree_ns;
using namespace std;
using namespace cv;

void DetectionTree::onInit()
{
    sub_name = "leviathan"; // TODO Pull from config
    ros::NodeHandle& nh = getMTNodeHandle();
    
    debug_dv_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("cusub_perception/detection_tree/poses",10);
    debug_dobj_poses_pub = nh.advertise<geometry_msgs::PoseArray>("cusub_perception/detection_tree/dobjects",10);
    dvector_pub = nh.advertise<detection_tree::Dvector>("cusub_perception/detection_tree/dvectors",10);
    // Temporarily subscribe to all camera info topics
    for( auto topic_frame : camera_topic_frame_map)
    {
        det_print(string("Added Camera: ") + topic_frame.second);
        camera_info_subs.insert({topic_frame.second, nh.subscribe(topic_frame.first, 1, &DetectionTree::cameraInfoCallback, this)});
    }    
    darknet_sub = nh.subscribe("cusub_perception/darknet_ros/bounding_boxes", 1, &DetectionTree::darknetCallback, this);
    dobj_pub_timer = nh.createTimer(ros::Duration(0.5), &DetectionTree::dobjPubCallback, this); // TODO pull config
    det_print("Loaded Detection Tree");
}

/*
    @brief Transforms a bearing vector from the local camera frame to the odom frame
    @param bearing_vec : [yaw, pitch, roll] to the object
    @param image_header : timestamp + frame_id used
    @return odom_cam_pose : position of the sub in odom + orientation with the bearing
*/
int DetectionTree::transformBearingToOdom(geometry_msgs::PoseStamped& odom_cam_pose, cv::Mat& bearing_vec, std_msgs::Header& image_header)
{
    // Transform bearing vector to odom frame
    std::string sub_frame = "/" + sub_name + "/description/odom";
    // geometry_msgs::PoseStamped odom_cam_pose;
    size_t length = image_header.frame_id.length();
    // size_t length = bbs->image_header.frame_id.length();
    string cam_frame = image_header.frame_id.substr(0, length-8); // trim _optical
    try{
        if( listener.waitForTransform(cam_frame, sub_frame, image_header.stamp, ros::Duration(1.0)) )
        {
            geometry_msgs::PoseStamped cam_pose;
            cam_pose.header = image_header;
            cam_pose.header.frame_id = cam_frame; // don't want "_optical" in frame id
            // Calculate the orientation in quaternions
            tf2::Quaternion cam_quat_tf;
            cv::Size s = bearing_vec.size();
            float yaw = -bearing_vec.at<double>(0,0); // flip yaw to match odom coords
            float pitch = -bearing_vec.at<double>(0,1); // flip pitch
            float roll = 0;
            cam_quat_tf.setRPY( roll, pitch, yaw );
            cam_quat_tf.normalize();

            // Convert tf::quaternion to geometry_msgs::Quaternion
            cam_pose.pose.orientation = tf2::toMsg(cam_quat_tf);
            listener.transformPose(sub_frame, cam_pose, odom_cam_pose);

        } else {
            det_print_warn("Detection Tree missed transform. Throwing out detection.");
            return -1;
        }
    } catch (tf::TransformException ex){
        det_print_warn("Detection Tree Error in transform");
        return -1;
    }
    return 0;
}

/* 
@brief Turns hits into dvectors and adds them to corresponding dobject or creates a new one
        Publishes dvector to all listeners
@param pointer to darknet bounding box msg
@return None
 */
void DetectionTree::darknetCallback(const darknet_ros_msgs::BoundingBoxesConstPtr bbs)
{
    // Check if we've received this camera's info
    string frame_optical = bbs->image_header.frame_id;
    if( camera_info.find(frame_optical) == camera_info.end() )
    {
        // det_print("Unreceived camera info: %s",  frame.c_str());
        return;
    }

    // Get matrices needed to calculate bearing to object
    sensor_msgs::CameraInfo ci = camera_info[bbs->image_header.frame_id];
    Mat K(3, 3, CV_64FC1, (void *) ci.K.data());
    Mat Kinv = K.inv();
    Mat D(ci.D.size(), 1, CV_64FC1, (void *) ci.D.data());

    // Loop through boxes and generate dvectors
    Mat bearing_vec;
    for ( auto box : bbs->bounding_boxes)
    {
        // det_print("Box received");

        // Get bearing vector in local camera frame
        int center_x = (box.xmax + box.xmin) / 2;
        int center_y = (box.ymax + box.ymin) / 2;
        Point2f pt(center_x, center_y);
        vector<cv::Point2f> pts = {pt};
        vector<cv::Point2f> upts;
        undistortPoints(pts, upts, K, D, cv::noArray(),  K);
        vector<double> ray_points{ upts[0].x, upts[0].y, 1 };
        Mat ray_point{3,1,cv::DataType<double>::type, ray_points.data()};
        bearing_vec = Kinv * ray_point;

        // Transform local bearing to bearing in odom
        geometry_msgs::PoseStamped odom_cam_pose; // sub's position + orientation along odom bearing to object
        std_msgs::Header image_header = bbs->image_header;
        int ret = transformBearingToOdom(odom_cam_pose, bearing_vec, image_header);
        if (ret) continue; // failed transform
        debug_dv_pose_pub.publish(odom_cam_pose);

        // Extract roll, pitch, yaw in odom frame
        double roll_odom, pitch_odom, yaw_odom;
        tf2::Quaternion odom_quat;
        tf2::fromMsg(odom_cam_pose.pose.orientation, odom_quat);
        tf2::Matrix3x3 m(odom_quat);
        m.getRPY(roll_odom, pitch_odom, yaw_odom);

        // Create Dvector
        // det_print("Creating Dvector");
        detection_tree::Dvector* dv = new detection_tree::Dvector; // we'll be storing it globally
        dv->sub_pt = odom_cam_pose.pose.position;
        dv->azimuth = yaw_odom;
        dv->elevation = pitch_odom;
        dv->camera_header = image_header;
        dv->class_id = box.Class;
        dv->probability = box.probability;
        dv->magnitude = (box.xmax - box.xmin) * (box.ymax - box.ymin);
        // det_print("Determining Dobject");
        int dobj_num = determineDobject(dv);

        if( dobj_num == -1) // Create new dobject
            dobj_num = createDobject(dv);
        else // Existing dobject, append the dvector
        {
            dobject_list[dobj_num]->dvector_list.push_back(dv);
            // Check for the pose solve
        }            
        dv->dobject_num = dobj_num;
        // det_print("Publishing Dvector");
        dvector_pub.publish(*dv);
    }
}

/*  @brief creates a new dobject
    @param first dvector
    @return dobject number
*/
int DetectionTree::createDobject(detection_tree::Dvector* dv)
{   
    string s = string("creating new dobj for ") + DETECTION_TREE_COLOR_START + dv->class_id + DETECTION_TREE_END;
    det_print(s);
    Dobject* dobj = new Dobject;
    dobj->num = dobject_list.size();
    dobj->class_id = dv->class_id;
    dobj->dvector_list.push_back(dv);
    dobject_list.push_back(dobj);
    return dobj->num;
}

/* @brief given a dvector, determine which dobject it belongs to or if a new one should be created
   @param dvector
   @return the dobject number or -1 for new dobject
*/
int DetectionTree::determineDobject(detection_tree::Dvector* dv)
{
    for( auto dobj : dobject_list )
    {
        return dobj->num; // TODO association algorithm
    }
    return -1;
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
            det_print(string("Received : ") + frame_id.c_str());
            // TODO remove on actual sub

            camera_info.insert({frame_id + "_optical", ci}); // adding _optical is 100% a hack
            camera_info_subs[itr_pair.first].shutdown(); // unsubscribe
            if ( camera_info_subs.size() == camera_info.size() )
            {
                det_print("All camera's info acquired");
            }
            return;
        }
    }
    det_print_warn(string("Unrecognized camera frame: ") + frame_id);
}

void DetectionTree::dobjPubCallback(const ros::TimerEvent&)
{
    geometry_msgs::PoseArray pose_arr;
    pose_arr.header.frame_id = "/" + sub_name + "/description/odom";

    for( auto dobj : dobject_list )
    {
        geometry_msgs::Pose pose;
        pose.position = dobj->dvector_list.back()->sub_pt;
        tf2::Quaternion quat_tf;
        double yaw = dobj->dvector_list.back()->azimuth;
        double pitch = dobj->dvector_list.back()->elevation;
        quat_tf.setRPY(0.0, pitch, yaw);
        quat_tf.normalize();
        pose.orientation = tf2::toMsg(quat_tf);
        pose_arr.poses.push_back(pose);
    }
    pose_arr.header.stamp = ros::Time::now();
    if( !pose_arr.poses.empty() ) // publish if not empty
    {
        debug_dobj_poses_pub.publish(pose_arr);
    }
}

void DetectionTree::det_print(std::string str)
{
    ROS_INFO( (DETECTION_TREE_NAME + str).c_str());
}

void DetectionTree::det_print_warn(std::string str)
{
    string s = DETECTION_TREE_NAME + DETECTION_TREE_WARN_START  + str + DETECTION_TREE_END;
    ROS_INFO( s.c_str());
}

PLUGINLIB_EXPORT_CLASS(det_tree_ns::DetectionTree, nodelet::Nodelet);