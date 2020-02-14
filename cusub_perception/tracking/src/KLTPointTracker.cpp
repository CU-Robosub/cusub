#include "tracking/KLTPointTracker.h"

using namespace tracking;

#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>

const int MIN_POINTS_FOUND = 4;

KLTPointTracker::KLTPointTracker()
{}

PointTracker::Result KLTPointTracker::initialize(const cv::Mat &image, const BoundingBox &roi)
{
    cv::Mat imageCopy;
    cv::cvtColor(image, imageCopy, CV_BGR2GRAY);
    Result result;

    cv::Rect roiRect = roi.roiRect();

    if (!ImageTools::checkRoi(image, roiRect))
    {
        result.message = "Invalid ROI";
        result.status = STATUS::Fail;

        return result;
    }

    cv::Mat croppedImage = imageCopy(roiRect);

    // extract shi-tomasi image features
    cv::goodFeaturesToTrack(croppedImage, m_currentPoints, 100, 0.3, 7, cv::Mat(), 7, false, 0.04);

    // shift back to full image
    for (cv::Point2f &pt : m_currentPoints)
    {
        pt.x += roi.xmin();
        pt.y += roi.ymin();
    }

    m_currentImg = imageCopy;

    if (m_currentPoints.size() > 2)
    {
        result.message = "Successfuly initialized KLT Point Tracker";
        result.status = STATUS::Success;
    }
    else
    {
        result.message = "Not enough points to initialize KLT Point Tracker: " + std::to_string(m_currentPoints.size());
        result.status = STATUS::Fail;
    }
    
    return result;
}

PointTracker::Result KLTPointTracker::trackPoints(const cv::Mat &imageGray)
{
    std::vector<cv::Point2f> newPoints, foundNewPoints, foundOldPoints;
    std::vector<uchar> pointStatus;
    std::vector<float> err;

    cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.03);

    cv::calcOpticalFlowPyrLK(m_currentImg, imageGray, m_currentPoints, newPoints, pointStatus, err, cv::Size(15,15), 2, criteria);

    for(uint i = 0; i < m_currentPoints.size(); i++)
    {
        // Select good points
        if(pointStatus[i] == 1)
        {
            foundNewPoints.push_back(newPoints[i]);
            foundOldPoints.push_back(m_currentPoints[i]);
        }
    }
    

    Result result;
    if (foundNewPoints.size() > MIN_POINTS_FOUND && (foundNewPoints.size() == foundOldPoints.size()))
    {
        // claculate the transform
        bool success;
        AffineTransform transform = AffineTransform(foundOldPoints, foundNewPoints, success);

        if (success)
        {

            // Now update the previous frame and previous points
            m_currentImg = imageGray.clone();
            m_currentPoints = foundNewPoints;

            result.status = STATUS::Success;
            result.transform = transform;
            result.message = "Succeeded with " + std::to_string(foundNewPoints.size()) + " points";
        }
        else
        {
            result.status = STATUS::Fail;
            result.message = "RANSAC failed to find a match";
        }
    }
    else
    {
        result.status = STATUS::Fail;
        result.message = "Not enough points found";
    }
    
    return result;
}

void KLTPointTracker::reset()
{

}