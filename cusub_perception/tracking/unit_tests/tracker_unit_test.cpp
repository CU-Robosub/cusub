/*
    Note this unit test tests the tracking functionality, none of the ROS interface

 */


#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "tracking/Tracking.h"

#include <stdio.h>

std::shared_ptr<tracking::Tracking> trackingPtr;
int main(int argc, char ** argv)
{
    ros::init(std::vector<std::pair<std::string, std::string> >{}, "tracking_unit_test");

    std::string videoName = "/home/soroush/robosub/ws/src/cusub/cusub_perception/tracking/unit_tests/videos/out.mp4";
    int xmin = 520;
    int xmax = 626;
    int ymin = 230;
    int ymax = 410;

    // std::string videoName = "/home/soroush/robosub/ws/src/cusub/cusub_perception/tracking/unit_tests/videos/jiangshi_slay.mp4";
    // int xmin = 240;
    // int xmax = 305;
    // int ymin = 140;
    // int ymax = 270;

    tracking::BoundingBox bbox(xmin, ymin, xmax, ymax);
    cv::VideoCapture cap(videoName);

    if (!cap.isOpened())
    {
        std::cout << "Cannot open video" << std::endl;
        return -1;
    }

    cv::Mat image;
    tracking::ImageData imageData;
    tracking::KLTPointTracker * tracker = new tracking::KLTPointTracker();
    trackingPtr.reset(new tracking::Tracking());


    if (tracker == nullptr)
    {
        return 2;
    }


    bool first;
    tracking::PointTracker::Result result;
    std::vector<cv::Point2f> bboxDebugPoints;

    bool break_pending = false;
    while (!break_pending)
    {
        cap = cv::VideoCapture(videoName); // reset cap
        bbox = tracking::BoundingBox(xmin, ymin, xmax, ymax);
        cap >> image;
        // cv::cvtColor(image, image, CV_BGR2GRAY);
        imageData.setImage(image);
        
        // initializes new tracker
        tracker->initialize(image, bbox);
        // trackingPtr->objectDetected("vampire_fathead", bbox, imageData);
        bboxDebugPoints = bbox.cornerPoints();
        first = true;
        while(!break_pending)
        {
            if (first == false)
            {
                std::vector<cv::Point2f> oldPoints = tracker->currentPoints();
                for (cv::Point2f pt : oldPoints)
                {
                    cv::circle(image, pt, 2, cv::Scalar(255,255,255), -1);
                }
                result.transform.transformPoints(oldPoints);
                for (cv::Point2f pt : oldPoints)
                {
                    cv::circle(image, pt, 2, cv::Scalar(0,0,255), -1);
                }
    
                result.transform.transformPoints(bboxDebugPoints);
                for (cv::Point2f pt : bboxDebugPoints)
                {
                    cv::circle(image, pt, 5, cv::Scalar(0,0,255), -1);
                }

                cv::imshow("Tracking box", image);
                // cv::waitKey(0);
                if (cv::waitKey(20) == 27) break_pending = true;
            }
            first = false;

            // next image in video
            cap >> image;
    
            if (image.empty())
            {
                break;
            }

            cv::cvtColor(image, image, CV_BGR2GRAY);
            result = tracker->trackPoints(image);

            bbox.setTransform(result.transform);
            cv::rectangle(image, bbox.roiRect(), cv::Scalar(0,0,255), 2);
        }
    }
    
    delete tracker;
    return 0;
}