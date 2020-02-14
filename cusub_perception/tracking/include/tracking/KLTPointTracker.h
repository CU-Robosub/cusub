/**
 * @file KLTPointTracker.h
 * @author Soroush Khadem (soroush.khadem@colorado.edu)
 * @brief Implements a Kandas-Lucas-Tomasi tracking algorithm
 * 
 */
#ifndef KLTPOINTTRACKER_H
#define KLTPOINTTRACKER_H

#include "PointTracker.h"
#include "ImageTools.h"
#include <opencv2/imgproc.hpp>

namespace tracking
{
class KLTPointTracker : public PointTracker
{
public:
    KLTPointTracker();
    virtual Result initialize(const cv::Mat &image, const BoundingBox &roi);
    virtual Result trackPoints(const cv::Mat &image);
    virtual void reset();

};

}; // namespace tracking

#endif // KLTPOINTTRACKER_H