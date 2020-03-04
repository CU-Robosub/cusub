#ifndef DETECTION_TREE_NODE_SRC_DETECTION_TREE_NODE_H_
#define DETECTION_TREE_NODE_SRC_DETECTION_TREE_NODE_H_

// Logging Formats
#include <string>
#define DETECTION_TREE_NAME             std::string("[\033[92mDetection Tree\033[0m] ")
#define DETECTION_TREE_WARN_START       std::string("\033[93m[WARN] ")
#define DETECTION_TREE_COLOR_START      std::string("\033[95m")
#define DETECTION_TREE_END              std::string("\033[0m")

# define DOBJECT_PROBABILITY_ZERO       0.0
# define DOBJECT_NOT_FOUND              -1

#include <pluginlib/class_list_macros.h>
#include "detection_tree/detection_tree.hpp"
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <ros/timer.h>
#include <vector>
#include <map>
#include <string>
#include <cmath>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/calib3d.hpp>
#include <detection_tree/Dvector.h>

// Camel case classes + functions, underscore variables + namespaces

namespace det_tree_ns
{
    class DetectionTree : public nodelet::Nodelet
    {
        public:
            virtual void onInit();
            void set_not_nodelet(void);
        private:
            bool is_nodelet = true;
            void init(ros::NodeHandle& nh);
            std::string sub_name;
            void darknetCallback(const darknet_ros_msgs::BoundingBoxesConstPtr bbs);
            bool checkIllegalDetection(int image_height, int image_width, darknet_ros_msgs::BoundingBox& box);
            void cameraInfoCallback(const sensor_msgs::CameraInfo ci);
            int transformBearingToOdom(geometry_msgs::PoseStamped& odom_cam_pose, cv::Mat& bearing_vec, std_msgs::Header& image_header);
            int createDobject(detection_tree::Dvector* dv);
            bool poseSolveDobject(Dobject* dobj, geometry_msgs::Pose& pose);
            void dobjPubCallback(const ros::TimerEvent&);
            void cuprint(std::string str);
            void cuprint_warn(std::string str);

            // Dvector Association Functions
            void associateDvectors(std::vector<detection_tree::Dvector*>& dv_list, std::map<detection_tree::Dvector*, int>& dv_dobj_map);            
            void assignDobjScores(std::vector<detection_tree::Dvector*>& dv_list, std::map<detection_tree::Dvector*, std::map<int, double>*>& dvs_scored);
            void setDobjProbabilityToZero(std::map<detection_tree::Dvector*, std::map<int, double>*>& dvs_scored, int dobj_num);
            detection_tree::Dvector* findBestMatch(std::map<detection_tree::Dvector*, std::map<int, double>*>& dvs_scored, int& matching_dobj);
            void averageBearing(std::vector<detection_tree::Dvector*>& dvs, double& average_az, double& average_elev);

            int dvector_num;
            ros::Timer dobj_pub_timer; // timer to publish most recent dvector for all dobjs
            std::vector<Dobject*> dobject_list;
            ros::Subscriber darknet_sub;
            ros::Publisher dvector_pub;
            ros::Publisher debug_dv_pose_pub, debug_dobj_poses_pub;
            tf::TransformListener listener;
            std::map<std::string, ros::Subscriber> camera_info_subs;
            std::map<std::string, sensor_msgs::CameraInfo> camera_info;
            std::map<std::string, std::string> camera_topic_frame_map = {
               {"/leviathan/cusub_common/occam/image0/camera_info", "leviathan/description/occam0_frame"},
               {"/leviathan/cusub_common/occam/image1/camera_info", "leviathan/description/occam1_frame"},
               {"/leviathan/cusub_common/occam/image2/camera_info", "leviathan/description/occam2_frame"},
               {"/leviathan/cusub_common/occam/image3/camera_info", "leviathan/description/occam3_frame"},
               {"/leviathan/cusub_common/occam/image4/camera_info", "leviathan/description/occam4_frame"},
               {"/leviathan/cusub_common/downcam/camera_info", "leviathan/description/downcam_frame"}
            }; // TODO move to launch file for configurability
            // std::map<std::string, std::string> camera_topic_frame_map = {
            //    {"/leviathan/cusub_common/occam/image0/camera_info", "leviathan/description/occam0_frame_optical"},
            //    {"/leviathan/cusub_common/occam/image1/camera_info", "leviathan/description/occam1_frame_optical"},
            //    {"/leviathan/cusub_common/occam/image2/camera_info", "leviathan/description/occam2_frame_optical"},
            //    {"/leviathan/cusub_common/occam/image3/camera_info", "leviathan/description/occam3_frame_optical"},
            //    {"/leviathan/cusub_common/occam/image4/camera_info", "leviathan/description/occam4_frame_optical"},
            //    {"/leviathan/cusub_common/downcam/camera_info", "leviathan/description/downcam_frame_optical"}
            // }; // TODO move to launch file for configurability
    };
}

#endif