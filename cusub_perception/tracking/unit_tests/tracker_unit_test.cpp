#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "tracking/Tracking.h"

#include <stdio.h>

std::shared_ptr<tracking::Tracking> trackingPtr;
int main(int argc, char ** argv)
{
    ros::init(std::vector<std::pair<std::string, std::string> >{}, "unit_test");

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
        std::cout << "Cannot open cap" << std::endl;
        return -1;
    }

    cv::Mat image;
    cap >> image;
    
    cv::rectangle(image, bbox.roiRect(), cv::Scalar(0,0,255));
    cv::imshow("initial", image);
    cv::waitKey(0);
    
    tracking::ImageData imageData;

    tracking::KLTPointTracker * tracker = new tracking::KLTPointTracker();
    tracker->initialize(image, bbox);

    trackingPtr.reset(new tracking::Tracking());
    imageData.setImage(image);
    trackingPtr->objectDetected("vampire_fathead", bbox, imageData);

    // full test with tracking object
    if (trackingPtr != nullptr)
    {
        while(1)
        {
            cap >> image;
            
            if (image.empty())
            {
                break;
            }
            
            imageData.setImage(image);
            trackingPtr->newImage(imageData);

            std::vector<tracking::BoundingBox> bboxs = trackingPtr->getBoxes(imageData.frameId());

            for (tracking::BoundingBox box : bboxs)
            {
                cv::rectangle(image, box.roiRect(), cv::Scalar(0,0,255), 2);
            } 

            cv::imshow("Tracked Object", image);
            cv::waitKey(0);
        }
    }

    // more in depth debugging shown
    if (tracker != nullptr)
    {
        cap = cv::VideoCapture(videoName); // reset cap
        cap >> image;

        std::vector<cv::Point2f> points = tracker->currentPoints();
        for (cv::Point pt : points)
        {
            cv::circle(image, pt, 2, cv::Scalar(0,0,255), -1);
        }
        cv::imshow("Shi-Tomasi Points", image);
        cv::waitKey(0);

        std::vector<cv::Point2f> bboxDebugPoints = bbox.cornerPoints();

        bool first = true;
        tracking::PointTracker::Result result;
        while(1)
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

                cv::imshow("Transformed points", image);
                cv::waitKey(0);
            }
            first = false;

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