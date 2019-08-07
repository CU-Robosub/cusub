/*
    The Z-Plane localizer uses known object/sub z-positions as well
    as camera calibration/orientation to generate map poses from the
    center point of a bounding box detection.

    This method requires
     1. Accurate object depth (Z)
     2. Accurate sub depth (Z)
     3. Accurate sub orientation
     4. Camera Matrix and distortion coefficents for the detecting camera

 */

#include <localizer/z_plane.h>

namespace pose_generator
{  
    ZPlane::ZPlane()
    {
        ros::NodeHandle nh;
        ROS_INFO("Z-Plane localizer enabled");
    }

    geometry_msgs::Point ZPlane::transformPoint(std_msgs::Header header, std::string target_frame, double x, double y, double z){

            geometry_msgs::PointStamped point_in;
            point_in.header = header;
            point_in.point.x = x;
            point_in.point.y = y;
            point_in.point.z = z;
            geometry_msgs::PointStamped point_out;

            listener.transformPoint(
                target_frame, // Target frame, should be robot odom
                point_in,
                point_out
            );

            return point_out.point;

    }

    bool ZPlane::generatePose(
        sensor_msgs::Image& image, 
        vector<darknet_ros_msgs::BoundingBox>& bbs,
        geometry_msgs::PoseStamped& pose,
        string& class_name
        ){

            vector<Point2f> points;
            vector<Point2f> undistorted_points;

            class_name = "dropper_cover";

            // Reject images that potentialy have bad detections
            if( bbs.size() != 1 ) { return false; }

            // Find the box center point
            points.push_back(Point2f((bbs[0].xmin + bbs[0].xmax) / 2, (bbs[0].ymin + bbs[0].ymax) / 2 ));

            //ROS_DEBUG_STREAM("\ncenter_point: \n" << points);

            // Undistort the point
            undistortPoints(points, undistorted_points, downcam_camera_matrix, downcam_dist_coefs, noArray(),  downcam_camera_matrix);

            //ROS_DEBUG_STREAM("\nundistorted_points: \n" << undistorted_points);

            // Project the point into camera space
            vector<double> ray_points{
                undistorted_points[0].x, undistorted_points[0].y, 1
            };
            Mat ray_point{1,3,DataType<double>::type, ray_points.data()};

            //ROS_DEBUG_STREAM("\nrp: \n" << ray_point);
            //ROS_DEBUG_STREAM("\ninv: \n" << downcam_camera_matrix.inv());
            //ROS_DEBUG_STREAM("\nt: \n" << ray_point.t());

            Mat point_cam_space = downcam_camera_matrix.inv() * ray_point.t();

            //ROS_DEBUG_STREAM("\npoint_cam_space: \n" << point_cam_space);

            // Transform the point into world space as well as the camera center
            std::string reference_frame("leviathan/description/odom");
            std::string camera_frame(image.header.frame_id);

            listener.waitForTransform(reference_frame, camera_frame, image.header.stamp, ros::Duration(5.0));

            geometry_msgs::Point cam_pt =
                transformPoint(
                    image.header,
                    reference_frame,
                    0,
                    0,
                    0
                );

            geometry_msgs::Point ray_pt =
                transformPoint(
                    image.header,
                    reference_frame,
                    point_cam_space.at<double>(0),
                    point_cam_space.at<double>(1),
                    point_cam_space.at<double>(2)
                );

            //ROS_DEBUG_STREAM("\ncam: \n" << cam_pt);
            //ROS_DEBUG_STREAM("\nraw: \n" << ray_pt);

            // Use the ray defined by the camera center and the ray point to
            //  intersect the Z-Plane that is defined by the floor
            //  We can do this by scaling the vector by the factor that makes
            //  the z of the ray equal to the object height
            double object_z = -4.2;
            double scale = (cam_pt.z - object_z) / (cam_pt.z - ray_pt.z);

            // Create the posez
            pose.header.frame_id = reference_frame;
            pose.pose.position.x = cam_pt.x + (ray_pt.x - cam_pt.x) * scale;
            pose.pose.position.y = cam_pt.y + (ray_pt.y - cam_pt.y) * scale;
            pose.pose.position.z = cam_pt.z + (ray_pt.z - cam_pt.z) * scale;

            //ROS_DEBUG_STREAM("\npose: \n" << pose.pose.position);

            return true;
        }
}