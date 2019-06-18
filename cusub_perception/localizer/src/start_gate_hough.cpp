/*
    The start gate localizer that uses the Hough algorithm
    along with a pnp solve to localize the gate.
 */

#include <localizer/start_gate_hough.h>

namespace pose_generator
{   
    void StartGateHough::getPoseFromPoints(vector<Point2f>& img_points, geometry_msgs::Pose& pose)
    {
        // SolvePnp
        Mat rvec(3,1,cv::DataType<double>::type);
        Mat tvec(3,1,cv::DataType<double>::type);
        solvePnP(gate_truth_pts, img_points, occam_camera_matrix, occam_dist_coefs, rvec, tvec);

        // Generate Rotation Matrix from rvec -> turn into Quaternion
        Mat rot_matrix;
        tf2::Quaternion q;

        Rodrigues(rvec, rot_matrix);
        tf2::Matrix3x3 m3{
            tf2Scalar(rot_matrix.at<double>(0,0)),
            tf2Scalar(rot_matrix.at<double>(1,0)),
            tf2Scalar(rot_matrix.at<double>(2,0)),
            tf2Scalar(rot_matrix.at<double>(0,1)),
            tf2Scalar(rot_matrix.at<double>(1,1)),
            tf2Scalar(rot_matrix.at<double>(2,1)),
            tf2Scalar(rot_matrix.at<double>(0,2)),
            tf2Scalar(rot_matrix.at<double>(1,2)),
            tf2Scalar(rot_matrix.at<double>(2,2))
        };
        m3.getRotation(q);
        pose.orientation = tf2::toMsg(q);
        pose.position.x = tvec.at<double>(0);
        pose.position.y = tvec.at<double>(1);
        pose.position.z = tvec.at<double>(2);
    }

    void StartGateHough::sortBoxes(vector<darknet_ros_msgs::BoundingBox>& bbs)
    {
        // We want [left_leg, middle_leg, right_leg]
        ;
    }

    bool StartGateHough::getPoints(Mat& img, vector<Point2f>& points)
    {
        Mat gray, binary, eroded;
        vector<Vec4i> lines;

        cvtColor(img, gray, COLOR_BGR2GRAY);
        threshold(gray, binary, 0, 255, THRESH_BINARY_INV + THRESH_OTSU);
        erode(binary, eroded, Mat());
        HoughLinesP(eroded, lines, 1, CV_PI/180, img.rows/4,img.rows/4,3);
        if(lines.empty()) {return false;}
        
        // Choose highest and lowest points out of all lines
        Point bottom_pt = Point2f(0,0);
        Point top_pt = Point2f(0,img.rows);
        for( size_t i = 0; i < lines.size(); i++ )
        {
            Vec4i l = lines[i];
            if(l[1] > bottom_pt.y)
                bottom_pt = Point2f(l[0],l[1]);
            if(l[3] < top_pt.y)
                top_pt = Point2f(l[2],l[3]);
        }
        points.push_back(bottom_pt);
        points.push_back(top_pt);
        return true;
    }

    bool StartGateHough::generatePose(
        sensor_msgs::Image& image, 
        vector<darknet_ros_msgs::BoundingBox>& bbs,
        geometry_msgs::Pose& pose,
        string& class_name
    ){
        if(bbs.size() != 2) {return false;} // to be adjusted for 3 leg case
        sortBoxes(bbs);
        ROS_INFO("Localizing the gate!");
        // Double check that its RGB8 not BGR8
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);

        // Trim each leg out of image
        Rect left_rect(bbs[0].xmin, bbs[0].ymin,bbs[0].xmax-bbs[0].xmin,bbs[0].ymax-bbs[0].ymin);
        Rect right_rect(bbs[1].xmin, bbs[1].ymin,bbs[1].xmax-bbs[1].xmin,bbs[1].ymax-bbs[1].ymin);
        Mat left_pole = cv_ptr->image(left_rect);
        Mat right_pole = cv_ptr->image(right_rect);

        // Find gate's corner points
        vector<Point2f> img_points;
        if(!getPoints(left_pole, img_points)) {return false;}
        if(!getPoints(right_pole, img_points)) {return false;}

        // Transform Points back to main image
        img_points[0].x += bbs[0].xmin;
        img_points[0].y += bbs[0].ymin;
        img_points[1].x += bbs[0].xmin;
        img_points[1].y += bbs[0].ymin;
        img_points[2].x += bbs[1].xmin;
        img_points[2].y += bbs[1].ymin;
        img_points[3].x += bbs[1].xmin;
        img_points[3].y += bbs[1].ymin;

        // Get pose from image points using a solvepnp
        getPoseFromPoints(img_points, pose);
        class_name = "start_gate";
        return true;
    }
}