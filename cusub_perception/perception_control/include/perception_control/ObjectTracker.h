#ifndef OBJECTTRACKER_H
#define OBJECTTRACKER_H

#include "BoundingBox.h"
#include "KLTPointTracker.h"

namespace perception_control
{

class ObjectTracker
{
public:
    ObjectTracker(BoundingBox &box, const cv::Mat &image);

    void initialize(BoundingBox &box, const cv::Mat &image);
    void updateImage(const cv::Mat &image);

    BoundingBox getCurrentBox() const;
    bool isValid();

private:
    PointTracker * m_pointTracker;
    BoundingBox m_boundingBox;

    bool m_valid;
};

}; // namespace perception_control

#endif // OBJECTTRACKER_H