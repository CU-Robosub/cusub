/**
 * @file AffineTransform.h
 * @author Soroush Khadem (soroush.khadem@colorado.edu)
 * @brief Handles transforms between points
 * 
 */
#ifndef AFFINETRANSFORM_H
#define AFFINETRANSFORM_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <opencv2/calib3d.hpp>

namespace tracking
{

class AffineTransform
{
public:
    AffineTransform();
    AffineTransform(const AffineTransform &other);
    AffineTransform(const std::vector<cv::Point2f> &fromPts, const std::vector<cv::Point2f> &toPts, bool &success);
    
    void transformPoints(std::vector<cv::Point2f> &pts) const;

private:
    cv::Mat m_transform;

};

}; // namespace tracking

#endif // AFFINETRANSFORM_H