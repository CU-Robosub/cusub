#ifndef KLTPOINTTRACKER_H
#define KLTPOINTTRACKER_H

#include "PointTracker.h"
#include <opencv2/imgproc.hpp>

namespace perception_control
{
class KLTPointTracker : public PointTracker
{
public:
    KLTPointTracker();
    virtual Result initialize(const cv::Mat &image, const BoundingBox &roi);
    virtual Result trackPoints(const cv::Mat &image);
    virtual void reset();

};

}; // namespace perception_control

#endif // KLTPOINTTRACKER_H