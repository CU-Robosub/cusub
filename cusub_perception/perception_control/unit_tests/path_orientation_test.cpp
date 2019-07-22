/*
    Path Orientation Server:
    Input: image w/ bounding box
    Output: yaw setpoint adjustment
*/
#include <perception_control/path_orient.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <thread>
#include <actionlib/client/simple_action_client.h>
#include <perception_control/PathOrientAction.h>
#include <actionlib/client/terminal_state.h>


using namespace cv;
using namespace std;

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

void publish_bb(bool * stop)
{
    string image_name = "/home/luke/ros/robosub_ws/src/cusub/cusub_perception/perception_control/unit_tests/images/path_pair_sim.jpg";
    Mat image = imread(image_name, CV_LOAD_IMAGE_COLOR);
    // draw_on_image(image);
    // imshow("Path Window", image);
    // waitKey(0);
    // return 0;

    darknet_ros_msgs::BoundingBoxes bbs;
    darknet_ros_msgs::BoundingBox pathbox1, pathbox2;
    pathbox1.Class = "path";
    pathbox1.probability = 1.0;
    pathbox1.xmin = 700;
    pathbox1.ymin = 25;
    pathbox1.xmax = 775;
    pathbox1.ymax = 160;
    pathbox2.Class = "path";
    pathbox2.probability = 1.0;
    pathbox2.xmin = 490;
    pathbox2.ymin = 680;
    pathbox2.xmax = 590;
    pathbox2.ymax = 790;
    bbs.bounding_boxes.push_back(pathbox1);
    cv_bridge::CvImage img_bridge = cv_bridge::CvImage(bbs.image_header, sensor_msgs::image_encodings::RGB8, image);
    img_bridge.toImageMsg(bbs.image);

    ros::NodeHandle nh;
    ros::Publisher darkPub = nh.advertise<darknet_ros_msgs::BoundingBoxes>("cusub_perception/darknet_ros/bounding_boxes", 1);
    ros::Publisher yawStatePub = nh.advertise<std_msgs::Float64>("cusub_common/motor_controllers/pid/yaw/state",1);
    std_msgs::Float64 f;
    f.data = 1.0;
    ros::Rate r(1);
    while (!*stop && ros::ok() )
    {
        darkPub.publish(bbs);
        yawStatePub.publish(f);
        r.sleep();
    }
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "path_orient_test");
    cout << "Starting Path Orientation Testing" << endl;
    
    // bbs.push_back(pathbox2);



    // geometry_msgs::Pose pose;
    // string class_id;
    // if(jw.generatePose(test_image, bbs, pose, class_id))
    // {
    //     cout << "localized pose!" <<endl;
    //     cout << pose << endl;
    // }
    
    // rectangle(image, Point(box.xmin,box.ymin), Point(box.xmax, box.ymax), Scalar(0,0,255));
    // namedWindow("Display Window", WINDOW_AUTOSIZE);
    // imshow("Jiangshi Window", image);
    // waitKey(0);

    // Essentially launch a thread that calls execute and another thread that publishes our target image
    // Join the threads and print the result
    
    ros::Time::init();
    bool stop = false;
    thread first( publish_bb, &stop);

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<perception_control::PathOrientAction> ac("path_orient");
    ROS_INFO("Waiting for server");
    ac.waitForServer(); //will wait for infinite time
    ROS_INFO("...found server");

    perception_control::PathOrientGoal goal;
    ac.sendGoal(goal);
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
    if (finished_before_timeout)
    {
        perception_control::PathOrientResultConstPtr res = ac.getResult();
        if (res->oriented == true)
        {
            cout << "True!" << endl;
        } else { cout << "not true" << endl;}
        
        ROS_INFO("Test Successful!!");
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    } else { cout << "timed out!" << endl; }
        
    stop = true;
    first.join();
    return 0;
}