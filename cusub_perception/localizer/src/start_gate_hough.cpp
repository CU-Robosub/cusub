/*
    The start gate localizer that uses the Hough algorithm
    along with a pnp solve to localize the gate.
 */

#include <localizer/start_gate_hough.h>

namespace pose_generator
{   
    void StartGateHough::getPoseFromVectors(Mat rvec, Mat tvec, geometry_msgs::Pose& pose)
    {
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

        Mat rvec(3,1,cv::DataType<double>::type);
        Mat tvec(3,1,cv::DataType<double>::type);

        solvePnP(gate_truth_pts, img_points, occam_camera_matrix, occam_dist_coefs, rvec, tvec);
        // cout << tvec << endl;
        getPoseFromVectors(rvec, tvec, pose);
        // cout << pose << endl;
        class_name = "start_gate";
        return true;
    }
}

void draw_on_image(Mat& image)
{   
    // Column then row
    // loop through cols and rows (.cols, .rows)
    for(int i=0; i < image.rows; i += 50)
    {
        line(image, Point(0,i), Point(image.cols,i), Scalar(0,0,255));
    }
    for(int j=0; j < image.cols; j += 50)
    {
        line(image, Point(j,0), Point(j,image.rows), Scalar(0,0,255));
    }
}

int main(int argc, char **argv)
{
    cout << "Starting Hough Testing" <<endl;
    pose_generator::StartGateHough sgw;

    // string image_name = "/home/luke/ros/robosub_ws/src/cusub/cusub_perception/localizer/src/frame0000.jpg";
    string image_name = "/home/luke/ros/robosub_ws/src/cusub/cusub_perception/localizer/src/T0_10-27-18-image0_0137.jpg";
    // string image_name = "/home/luke/ros/robosub_ws/src/cusub/cusub_perception/localizer/src/T0_10-27-18-image0_0160.jpg";
    Mat image = imread(image_name, CV_LOAD_IMAGE_COLOR);
    

    // First image
    // darknet_ros_msgs::BoundingBox image_left;
    // darknet_ros_msgs::BoundingBox image_right;
    // image_left.Class = "start_gate_pole";
    // image_left.probability = 1.0;
    // image_left.xmin = 390;
    // image_left.ymin = 160;
    // image_left.xmax = 420;
    // image_left.ymax = 300;
    // image_right.Class = "start_gate_pole";
    // image_right.probability = 1.0;
    // image_right.xmin = 650;
    // image_right.ymin = 155;
    // image_right.xmax = 680;
    // image_right.ymax = 310;
    // vector<darknet_ros_msgs::BoundingBox> bbs;
    // bbs.push_back(image_left);
    // bbs.push_back(image_right);

    // Second image
    darknet_ros_msgs::BoundingBox image_left;
    darknet_ros_msgs::BoundingBox image_right;
    image_left.Class = "start_gate_pole";
    image_left.probability = 1.0;
    image_left.xmin = 340;
    image_left.ymin = 50;
    image_left.xmax = 375;
    image_left.ymax = 225;
    image_right.Class = "start_gate_pole";
    image_right.probability = 1.0;
    image_right.xmin = 575;
    image_right.ymin = 100;
    image_right.xmax = 640;
    image_right.ymax = 275;
    vector<darknet_ros_msgs::BoundingBox> bbs;
    bbs.push_back(image_left);
    bbs.push_back(image_right);

    std_msgs::Header header;
    cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image);
    sensor_msgs::Image test_image;
    img_bridge.toImageMsg(test_image);

    geometry_msgs::Pose pose;
    string class_id;
    if(sgw.generatePose(test_image, bbs, pose, class_id))
    {
        cout << "localized pose!" <<endl;
    }

    // line(image0, Point(0,10), Point(752,10), Scalar(0,0,255));
    // draw_on_image(image);
    // rectangle(image, Point(image_left.xmin,image_left.ymin), Point(image_left.xmax, image_left.ymax), Scalar(0,0,255));
    // rectangle(image, Point(image_right.xmin,image_right.ymin), Point(image_right.xmax, image_right.ymax), Scalar(0,0,255));
    // namedWindow("Display Window", WINDOW_AUTOSIZE);
    // imshow("Display Window", image);
    // waitKey(0);
    
    // really load the images & display them
    // pass them into generate pose
    return 0;
}

// Add to generatePose to visualize dots
// circle(cv_ptr->image, img_points[0],3,Scalar(0,0,255),3);
// circle(cv_ptr->image, img_points[1],3,Scalar(0,0,255),3);
// circle(cv_ptr->image, img_points[2],3,Scalar(0,0,255),3);
// circle(cv_ptr->image, img_points[3],3,Scalar(0,0,255),3);
// imshow("Display Window", cv_ptr->image);
// waitKey(0);
// ADD to getPoints to visualize single leg
// Mat c_eroded;
// cvtColor(eroded, c_eroded, COLOR_GRAY2BGR);
// circle(c_eroded, bottom_pt,3,Scalar(0,0,255),3);
// circle(c_eroded, top_pt,3,Scalar(0,0,255),3);
// imshow("Display Window", c_eroded);
// waitKey(0);