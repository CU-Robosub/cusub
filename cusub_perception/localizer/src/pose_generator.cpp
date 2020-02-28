#include <localizer/pose_generator.h>

namespace pose_generator
{
    /*
        Given Truth Points and image points, use a pnpsolve to calculate the object's pose
        Returns pose through parameter pointer
     */
    void PoseGenerator::getPoseFromPoints(vector<Point3f>& truth_pts, vector<Point2f>& img_points, geometry_msgs::Pose& pose)
    {
        // SolvePnp
        Mat rvec(3,1,cv::DataType<double>::type);
        Mat tvec(3,1,cv::DataType<double>::type);
        solvePnP(truth_pts, img_points, occam_camera_matrix, occam_dist_coefs, rvec, tvec);

        // Generate Rotation Matrix from rvec -> turn into Quaternion
        Mat rot_matrix;
        tf2::Quaternion q;

        Rodrigues(rvec, rot_matrix);
        tf2::Matrix3x3 m3{
            tf2Scalar(rot_matrix.at<double>(0,0)),
            tf2Scalar(rot_matrix.at<double>(0,1)),
            tf2Scalar(rot_matrix.at<double>(0,2)),
            tf2Scalar(rot_matrix.at<double>(1,0)),
            tf2Scalar(rot_matrix.at<double>(1,1)),
            tf2Scalar(rot_matrix.at<double>(1,2)),
            tf2Scalar(rot_matrix.at<double>(2,0)),
            tf2Scalar(rot_matrix.at<double>(2,1)),
            tf2Scalar(rot_matrix.at<double>(2,2))
        };
        m3.getRotation(q);
        pose.orientation = tf2::toMsg(q);
        pose.position.x = tvec.at<double>(0);
        pose.position.y = tvec.at<double>(1) - 0.7; // added to correct for optical to odom bad transform
        pose.position.z = tvec.at<double>(2);
    }

//     #define DETECTION_TREE_NAME             std::string("[\033[92mDetection Tree\033[0m] ")
// #define DETECTION_TREE_WARN_START       std::string("\033[93m[WARN] ")
// #define DETECTION_TREE_COLOR_START      std::string("\033[95m")
// #define DETECTION_TREE_END              std::string("\033[0m")

    void PoseGenerator::cuprint(std::string str)
    {
        std::string print_str = std::string("[\033[92m") + pose_gen_name + std::string("\033[0m] ");
        ROS_INFO( (print_str + str).c_str());
    }

    void PoseGenerator::cuprint_warn(std::string str)
    {
        std::string print_str = std::string("[\033[92m") + pose_gen_name + std::string("\033[0m] ");
        print_str = print_str + std::string("\033[93m[WARN] ") + str + std::string("\033[0m");
        ROS_INFO( print_str.c_str());
    }
}