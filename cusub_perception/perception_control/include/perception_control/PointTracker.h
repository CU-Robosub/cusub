#ifndef POINTTRACKER
#define POINTTRACKER

#include <opencv2/core.hpp>
#include "AffineTransform.h"
#include "BoundingBox.h"

namespace perception_control
{
class PointTracker
{
public:
    enum class STATUS
    {
        Success,
        Fail
    };

    struct Result
    {
        STATUS status;
        AffineTransform transform;
        std::string message;

        Result() {};
        Result(STATUS _status, std::string _message) :
            status(_status),
            transform(AffineTransform()),
            message(_message) {};
        Result(STATUS _status, AffineTransform _transform, std::string _message) :
            status(_status),
            transform(_transform),
            message(_message) {};
    };

    bool isValid();
    std::vector<cv::Point2f> currentPoints() const;
    cv::Mat currentImage() const;


    virtual Result initialize(const cv::Mat &image, const BoundingBox &roi) = 0;
    virtual Result trackPoints(const cv::Mat &image) = 0;
    virtual void reset() = 0;

protected:
    std::vector<cv::Point2f> m_currentPoints;
    cv::Mat m_currentImg;

};

}; // namespace perception_control

#endif // POINTTRACKER