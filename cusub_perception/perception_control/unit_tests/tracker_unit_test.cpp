#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "perception_control/Tracking.h"

#include <stdio.h>

int main(int argc, char ** argv)
{
    // std::string imageName = "/home/soroush/robosub/ws/src/cusub/cusub_perception/perception_control/unit_tests/images/jiangshi_real_close.jpg";
    // int xmin = 327;
    // int xmax = 526;
    // int ymin = 12;
    // int ymax = 330;
    // perception_control::BoundingBox bbox(xmin, ymin, xmax, ymax);
    // cv::Mat image = cv::imread(imageName);
    // cv::Mat imageCopy;
    // cv::cvtColor(image, imageCopy, CV_BGR2GRAY);
    // cv::imshow("Image", image);
    // cv::waitKey(0);
    // cv::imshow("Cropped", image(bbox.roiRect()));
    // cv::waitKey(0);
    std::string videoName = "/home/soroush/robosub/ws/src/cusub/cusub_perception/perception_control/unit_tests/videos/out.mp4";
    cv::VideoCapture cap(videoName);
    int xmin = 520;
    int xmax = 626;
    int ymin = 230;
    int ymax = 410;
    perception_control::BoundingBox bbox(xmin, ymin, xmax, ymax);

    if (!cap.isOpened())
    {
        std::cout << "Cannot open cap" << std::endl;
        return -1;
    }

    cv::Mat image;
    cap >> image;
    
    cv::rectangle(image, bbox.roiRect(), cv::Scalar(0,0,255));
    cv::imshow("initial", image);
    cv::waitKey(0);
    
    perception_control::KLTPointTracker * tracker = new perception_control::KLTPointTracker();
    tracker->initialize(image, bbox);

    perception_control::Tracking * tracking = new perception_control::Tracking();
    tracking->objectDetected("vampire_fathead", bbox, image);

    while(1)
    {
        cap >> image;
        
        if (image.empty())
        {
            break;
        }
        
        tracking->newImage(image);

        perception_control::BoundingBox bbox = tracking->getBox("vampire_fathead");

        cv::rectangle(image, bbox.roiRect(), cv::Scalar(0,0,255), 2);

        cv::imshow("video", image);
        cv::waitKey(0);
    }
    // std::vector<cv::Point2f> points = tracker->currentPoints();
    // for (cv::Point pt : points)
    // {
    //     cv::circle(image, pt, 2, cv::Scalar(0,0,255), -1);
    // }
    // cv::imshow("Shi-Tomasi Points", image);
    // cv::waitKey(0);

    // perception_control::PointTracker::Result result;
    // while(1)
    // {
    //     cap >> image;
        
    //     if (image.empty())
    //     {
    //         break;
    //     }
    //     result = tracker->trackPoints(image);

    //     result.transform.transformBox(bbox, image);
    //     bbox.fixBox(tracker->currentPoints());

    //     cv::rectangle(image, bbox.roiRect(), cv::Scalar(0,0,255), 2);

    //     std::vector<cv::Point2f> points = tracker->currentPoints();
    //     for (cv::Point pt : points)
    //     {
    //         cv::circle(image, pt, 2, cv::Scalar(0,255,0), -1);
    //     }

    //     cv::imshow("video", image);
    //     cv::waitKey(0);
    // }
    

    delete tracker;

    return 0;
}