#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "tracking/Tracking.h"

#include <stdio.h>

std::shared_ptr<perception_control::Tracking> tracking;
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

    perception_control::BoundingBox bbox(xmin, ymin, xmax, ymax);
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
    
    perception_control::ImageData imageData;

    perception_control::KLTPointTracker * tracker = new perception_control::KLTPointTracker();
    tracker->initialize(image, bbox);

    tracking.reset(new perception_control::Tracking());
    imageData.setImage(image);
    tracking->objectDetected("vampire_fathead", bbox, imageData);

    // full test with tracking object
    if (tracking != nullptr)
    {
        while(1)
        {
            cap >> image;
            
            if (image.empty())
            {
                break;
            }
            
            imageData.setImage(image);
            tracking->newImage(imageData);

            perception_control::BoundingBox bbox = tracking->getBox("vampire_fathead");

            cv::rectangle(image, bbox.roiRect(), cv::Scalar(0,0,255), 2);

            cv::imshow("Tracked Object", image);
            cv::waitKey(0);
        }
    }

    // more in depth debugging shown
    if (tracker != nullptr)
    {
        cap = cv::VideoCapture(videoName);
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
        perception_control::PointTracker::Result result;
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

            result = tracker->trackPoints(image);

            bbox.setTransform(result.transform);

            cv::rectangle(image, bbox.roiRect(), cv::Scalar(0,0,255), 2);
        }
    }
    

    delete tracker;

    return 0;
}