#include "detection_tree/detection_tree_node.hpp"

using namespace det_tree_ns;
using namespace std;
using namespace cv;

void DetectionTree::onInit()
{
    sub_name = "leviathan"; // TODO Pull from config
    ros::NodeHandle& nh = getMTNodeHandle();
    dvector_num = 0;
    
    debug_dv_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("cusub_perception/detection_tree/poses",10);
    debug_dobj_poses_pub = nh.advertise<geometry_msgs::PoseArray>("cusub_perception/detection_tree/dobjects",10);
    dvector_pub = nh.advertise<detection_tree::Dvector>("cusub_perception/detection_tree/dvectors",10);
    // Temporarily subscribe to all camera info topics
    for( auto topic_frame : camera_topic_frame_map)
    {
        // det_print(string("Added Camera: ") + topic_frame.second);
        camera_info_subs.insert({topic_frame.second, nh.subscribe(topic_frame.first, 1, &DetectionTree::cameraInfoCallback, this)});
    }    
    darknet_sub = nh.subscribe("cusub_perception/darknet_ros/bounding_boxes", 1, &DetectionTree::darknetCallback, this);
    dobj_pub_timer = nh.createTimer(ros::Duration(0.5), &DetectionTree::dobjPubCallback, this); // TODO pull config
    det_print(string("Loaded Detection Tree"));
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
    geometry_msgs::PoseStamped odom_point_pose;
    size_t length = image_header.frame_id.length();
    string cam_frame = image_header.frame_id; //.substr(0, length-8); // trim _optical
    try{
        if( listener.waitForTransform(cam_frame, sub_frame, image_header.stamp, ros::Duration(1.0)) )
        {
            geometry_msgs::PoseStamped cam_pose, point_pose, odom_point_pose;
            cam_pose.header = image_header;
            point_pose.header = image_header;
            cam_pose.pose.orientation.w = 1;
            point_pose.pose.orientation.w = 1;

            // transform camera pose
            listener.transformPose(sub_frame, cam_pose, odom_cam_pose);
            
            // transform detection point
            point_pose.pose.position.x = bearing_vec.at<double>(0,0);
            point_pose.pose.position.y = bearing_vec.at<double>(0,1);
            point_pose.pose.position.z = bearing_vec.at<double>(0,2);
            listener.transformPose(sub_frame, point_pose, odom_point_pose);

            // take the difference between the two transformed points, then get the angles from the difference
            double delta1 = odom_point_pose.pose.position.x - odom_cam_pose.pose.position.x;
            double delta2 = odom_point_pose.pose.position.y - odom_cam_pose.pose.position.y;
            double delta3 = odom_point_pose.pose.position.z - odom_cam_pose.pose.position.z;
            double yaw = atan2(delta2, delta1);
            double tmp = sqrt( pow(delta1,2) + pow(delta2,2) );
            double pitch = atan2(delta3, tmp);
            pitch *= -1; // flip pitch to actually align with odom frame
            double roll = 0.0;

            tf2::Quaternion cam_quat_tf;
            cam_quat_tf.setRPY( roll, pitch, yaw );
            cam_quat_tf.normalize();
            odom_cam_pose.pose.orientation = tf2::toMsg(cam_quat_tf);

            // det_print("------------------");
            // det_print(string("yaw: ") + to_string(yaw * (180 / 3.1415)) + string(" deg"));
            // det_print(string("pitch ") + to_string(pitch * (180 / 3.1415)) + string(" deg"));
        } else {
            det_print_warn(string("Detection Tree missed transform. Throwing out detection."));
            return -1;
        }
    } catch (tf::TransformException ex){
        det_print_warn(string("Detection Tree Error in transform: ") + string(ex.what()));
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
        // det_print(string("Unreceived camera info: ") + frame_optical);
        return;
    }

    // Get matrices needed to calculate bearing to object
    sensor_msgs::CameraInfo ci = camera_info[bbs->image_header.frame_id];
    Mat K(3, 3, CV_64FC1, (void *) ci.K.data());
    Mat Kinv = K.inv();
    Mat D(ci.D.size(), 1, CV_64FC1, (void *) ci.D.data());

    // Loop through boxes and generate dvectors
    Mat bearing_vec;
    vector<detection_tree::Dvector*> dv_list;
    for ( auto box : bbs->bounding_boxes)
    {
        // Get bearing vector in local camera frame
        int center_x = (box.xmax + box.xmin) / 2;
        int center_y = (box.ymax + box.ymin) / 2;

        // Correct the center point for camera image scaling
        if(bbs->image.height != ci.height || bbs->image.width != ci.width){

            float scale_x = ((float) ci.width) / ((float) bbs->image.width);
            float scale_y = ((float) ci.height) / ((float) bbs->image.height); 
            center_x *= scale_x;
            center_y *= scale_y;

        }

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
        detection_tree::Dvector* dv = new detection_tree::Dvector; // we'll be storing it globally
        dv->sub_pt = odom_cam_pose.pose.position;
        dv->azimuth = yaw_odom;
        dv->elevation = pitch_odom;
        dv->camera_header = image_header;
        dv->class_id = box.Class;
        dv->probability = box.probability;
        dv->magnitude = (box.xmax - box.xmin) * (box.ymax - box.ymin);
        dv->dvector_num = dvector_num;
        dvector_num++;
        dv_list.push_back(dv);
    }
    map<detection_tree::Dvector*, int> dv_dobj_map;
    associateDvectors(dv_list, dv_dobj_map);

    // Add dvectors to dobjects or create a new dobject
    // Publish dvectors
    for ( auto it = dv_dobj_map.begin(); it != dv_dobj_map.end(); it++ )
    {
        detection_tree::Dvector* dv = it->first;
        int dobj_num = it->second;
        // Create new dobject
        if( dobj_num == DOBJECT_NOT_FOUND)
        {
            dobj_num = createDobject(dv);
        } 
        else {
            dobject_list[dobj_num]->dvector_list.push_back(dv);
        }
        dv->dobject_num = dobj_num;
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
    dobj->pose = geometry_msgs::Pose();
    dobj->dvector_list.push_back(dv);
    dobject_list.push_back(dobj);
    return dobj->num;
}

void DetectionTree::averageBearing(vector<detection_tree::Dvector*>& dvs, double& average_az, double& average_elev, double& average_mag)
{
    average_az = 0.0;
    average_elev = 0.0;
    average_mag = 0.0;
    for ( auto dv : dvs )
    {
        average_az += dv->azimuth;
        average_elev += dv->elevation;
        average_mag += dv->magnitude;
    }
    average_az /= dvs.size();
    average_elev /= dvs.size();
    average_mag /= dvs.size();
}

void DetectionTree::assignDobjScores(std::vector<detection_tree::Dvector*>& dv_list, map<detection_tree::Dvector*, map<int, double>*>& dvs_scored)
{
    for ( auto dv : dv_list )
    {
        map<int, double>* dobj_scores = new map<int, double>();
        vector<Dobject*> dobjects;
        for (auto dobj : dobject_list )
        {
            if (dobj->class_id == dv->class_id) // class matches; score here
            {
                std::vector<detection_tree::Dvector*> dvs;
                getLastDvectors(dobj, 5, dvs);
                double average_az, average_elev, average_mag;
                averageBearing(dvs, average_az, average_elev, average_mag);

                // Compute distance between bearing vectors
                double azimuth_diff = pow(dv->azimuth - average_az, 2);
                double elevation_diff = pow(dv->azimuth - average_az, 2);
                double distance = sqrt(azimuth_diff + elevation_diff);
                // det_print(string("distance: ") + to_string(distance));

                // Turn distance into a score
                double score = 1 / distance;
                dobj_scores->insert({dobj->num, score});
            }
        }
        dvs_scored.insert({dv, dobj_scores});
    }
}

/* @brief sets a dobject's probability to zero because we have already assigned this dobject to another dvector in the same image frame
   @param dvs_scored map
   @param the dobject_number
   @return None
*/
void DetectionTree::setDobjProbabilityToZero(map<detection_tree::Dvector*, map<int, double>*>& dvs_scored, int dobj_num)
{
    for ( auto dv_it = dvs_scored.begin(); dv_it != dvs_scored.end(); dv_it++ )
    {
        for ( auto dobj_it = dv_it->second->begin(); dobj_it != dv_it->second->end(); dobj_it++ )
        {
            if (dobj_it->first == dobj_num)
            {
                dobj_it->second = DOBJECT_PROBABILITY_ZERO;
            }
        }
    }
}

detection_tree::Dvector* DetectionTree::findBestMatch(map<detection_tree::Dvector*, map<int, double>*>& dvs_scored, int& matching_dobj)
{
    detection_tree::Dvector* best_matched_dv;
    double top_score = DOBJECT_PROBABILITY_ZERO;
    for ( auto dv_it = dvs_scored.begin(); dv_it != dvs_scored.end(); dv_it++ )
    {
        for ( auto dobj_it = dv_it->second->begin(); dobj_it != dv_it->second->end(); dobj_it++ )
        {
           if( dobj_it->second > top_score ) // check if this dobject match beats our top score
            {
                best_matched_dv = dv_it->first;
                matching_dobj = dobj_it->first;
                top_score = dobj_it->second;
            }
        }
    }
    // Check if we had no matches (this means there are no available dobjects for this class)
    if (top_score == DOBJECT_PROBABILITY_ZERO)
    {
        best_matched_dv = dvs_scored.begin()->first;
        matching_dobj = DOBJECT_NOT_FOUND;
    }
    return best_matched_dv;
}

/* @brief given a dvector, determine which dobject it belongs to or if a new one should be created
   @param dvector
   @return the dobject number or -1 for new dobject
   @return map{ dv : dobj_num }
*/
void DetectionTree::associateDvectors(std::vector<detection_tree::Dvector*>& dv_list, map<detection_tree::Dvector*, int>& dv_dobj_map)
{
    map<detection_tree::Dvector*, map<int, double>*> dvs_scored;
    assignDobjScores(dv_list, dvs_scored);

    // while we have more dvectors to assign
    while( dvs_scored.size() > 0)
    {
        int dobj_num_matched;
        detection_tree::Dvector* dv = findBestMatch(dvs_scored, dobj_num_matched);
        dv_dobj_map.insert({dv, dobj_num_matched});
        dvs_scored.erase(dv);
        if( dobj_num_matched != DOBJECT_NOT_FOUND )
        {
            setDobjProbabilityToZero(dvs_scored, dobj_num_matched); // make this dobject not be considered for all other dvectors
        }
    }

    // Free map memory in dvs_scored
    for ( auto it_dv = dvs_scored.begin(); it_dv != dvs_scored.end(); it_dv++ )
    {
        delete it_dv->second;
    }
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
            // det_print(string("Received : ") + frame_id.c_str());
            // TODO remove on actual sub

            camera_info.insert({frame_id + "_optical", ci}); // adding _optical is 100% a hack
            camera_info_subs[itr_pair.first].shutdown(); // unsubscribe
            if ( camera_info_subs.size() == camera_info.size() )
            {
                det_print(string("All camera's info acquired"));
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