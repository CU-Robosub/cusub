#include <localizer/jiangshi_watershed.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

 /*
 Load and display images, find bounding boxes
 Apply getPoints algorithm and see how far we can get
 Add generatePose method call

  */

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

int main(int argc, char ** argv)
{
    cout << "Starting Jiangshi Watershed Testing" << endl;
    pose_generator::JiangshiWatershed jw;
    // string image_name = "/home/luke/ros/robosub_ws/src/cusub/cusub_perception/localizer/unit_tests/images/jiangshi_close.jpg";
    // string image_name = "/home/luke/ros/robosub_ws/src/cusub/cusub_perception/localizer/unit_tests/images/jiangshi_far.jpg";
    string image_name = "/home/luke/ros/robosub_ws/src/cusub/cusub_perception/localizer/unit_tests/images/jiangshi_real_far.jpg";
    // string image_name = "/home/luke/ros/robosub_ws/src/cusub/cusub_perception/localizer/unit_tests/images/jiangshi_real_really_far.jpg";
    // string image_name = "/home/luke/ros/robosub_ws/src/cusub/cusub_perception/localizer/unit_tests/images/jiangshi_real_close.jpg";
    Mat image = imread(image_name, CV_LOAD_IMAGE_COLOR);
    // draw_on_image(image);
    // imshow("Jiangshi Window", image);
    // waitKey(0);
    // return 0;

    darknet_ros_msgs::BoundingBox box;
    box.Class = "jiangshi";
    box.probability = 1.0;
    // box.xmin = 560;
    // box.ymin = 100;
    // box.xmax = 720;
    // box.ymax = 370;
    // -----------
    // box.xmin = 420;
    // box.ymin = 150;
    // box.xmax = 500;
    // box.ymax = 300;
    // -----------
    box.xmin = 600;
    box.ymin = 110;
    box.xmax = 690;
    box.ymax = 270;
    vector<darknet_ros_msgs::BoundingBox> bbs;
    bbs.push_back(box);

    std_msgs::Header header;
    cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image);
    sensor_msgs::Image test_image;
    img_bridge.toImageMsg(test_image);

    geometry_msgs::Pose pose;
    string class_id;
    if(jw.generatePose(test_image, bbs, pose, class_id))
    {
        cout << "localized pose!" <<endl;
    }
    
    // rectangle(image, Point(box.xmin,box.ymin), Point(box.xmax, box.ymax), Scalar(0,0,255));
    // namedWindow("Display Window", WINDOW_AUTOSIZE);
    // imshow("Jiangshi Window", image);
    // waitKey(0);
}