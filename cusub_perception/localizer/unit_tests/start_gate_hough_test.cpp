#include <localizer/start_gate_hough.h>
#include <ros/ros.h>

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
    ros::init(argc, argv, "start_gate_hough_tester");
    cout << "Starting Hough Testing" <<endl;
    pose_generator::StartGateHough sgh;

    string image_name = "/home/luke/ros/robosub_ws/src/cusub/cusub_perception/localizer/unit_tests/images/start_gate0.jpg";
    // string image_name = "/home/luke/ros/robosub_ws/src/cusub/cusub_perception/localizer/unit_test/images/start_gate1.jpg";
    Mat image = imread(image_name, CV_LOAD_IMAGE_COLOR);

    // First image
    darknet_ros_msgs::BoundingBox image_left;
    darknet_ros_msgs::BoundingBox image_right;
    darknet_ros_msgs::BoundingBox image_middle;    
    image_left.Class = "start_gate_pole";
    image_left.probability = 1.0;
    image_left.xmin = 390;
    image_left.ymin = 160;
    image_left.xmax = 420;
    image_left.ymax = 300;
    image_right.Class = "start_gate_pole";
    image_right.probability = 1.0;
    image_right.xmin = 650;
    image_right.ymin = 155;
    image_right.xmax = 680;
    image_right.ymax = 310;
    image_middle.Class = "start_gate_pole";
    image_middle.probability = 1.0;
    image_middle.xmin = 500; // 500, 580
    image_middle.ymin = 155;
    image_middle.xmax = 520; // 520, 600
    image_middle.ymax = 250; 
    vector<darknet_ros_msgs::BoundingBox> bbs;
    bbs.push_back(image_left);
    bbs.push_back(image_right);
    bbs.push_back(image_middle);
    // Second image
    // darknet_ros_msgs::BoundingBox image_left;
    // darknet_ros_msgs::BoundingBox image_right;
    // image_left.Class = "start_gate_pole";
    // image_left.probability = 1.0;
    // image_left.xmin = 340;
    // image_left.ymin = 50;
    // image_left.xmax = 375;
    // image_left.ymax = 225;
    // image_right.Class = "start_gate_pole";
    // image_right.probability = 1.0;
    // image_right.xmin = 575;
    // image_right.ymin = 100;
    // image_right.xmax = 640;
    // image_right.ymax = 275;
    // vector<darknet_ros_msgs::BoundingBox> bbs;
    // bbs.push_back(image_left);
    // bbs.push_back(image_right);

    std_msgs::Header header;
    cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image);
    sensor_msgs::Image test_image;
    img_bridge.toImageMsg(test_image);

    geometry_msgs::Pose pose;
    string class_id;
    if(sgh.generatePose(test_image, bbs, pose, class_id))
    {
        cout << "localized pose!" <<endl;
    }

    // line(image0, Point(0,10), Point(752,10), Scalar(0,0,255));
    // draw_on_image(image);
    rectangle(image, Point(image_left.xmin,image_left.ymin), Point(image_left.xmax, image_left.ymax), Scalar(0,0,255));
    rectangle(image, Point(image_right.xmin,image_right.ymin), Point(image_right.xmax, image_right.ymax), Scalar(0,0,255));
    rectangle(image, Point(image_middle.xmin,image_middle.ymin), Point(image_middle.xmax, image_middle.ymax), Scalar(0,0,255));
    namedWindow("Display Window", WINDOW_AUTOSIZE);
    imshow("Display Window", image);
    waitKey(0);
    
    // really load the images & display them
    // pass them into generate pose
    return 0;
}

// Add to generatePose to visualize dots

// ADD to getPoints to visualize single leg
