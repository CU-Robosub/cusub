/**
 * @file BoundingBox.h
 * @author Soroush Khadem (soroush.khadem@colorado.edu)
 * @brief Holds a bounding box object
 * 
 */
#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#include <opencv2/core.hpp>
#include "AffineTransform.h"

namespace tracking
{

class BoundingBox
{

public:
    BoundingBox();
    BoundingBox(const int &xmin, const int &ymin, const int &xmax, const int &ymax);
    BoundingBox(const int &xmin, const int &ymin, const int &xmax, const int &ymax, const float &probability);

    void setTransform(const AffineTransform &tform);
    
    int xmin() const;
    int ymin() const;
    int xmax() const;
    int ymax() const;
    float probability() const;
    bool isValid() const;
    void extendBox(const int &amnt);
    
    std::vector<cv::Point2f> cornerPoints() const;
    cv::Rect roiRect() const;
    int area() const;
    cv::Point2f center() const;
    bool doesOverlap(const BoundingBox &other) const;
    int overlapArea(const BoundingBox &other) const;

private:
    static const int LOW_THRESHOLD;
    static const int HIGH_THRESHOLD;
    static const int AREA_THRESHOLD;

    std::vector<cv::Point2f> m_points;
    bool m_valid;

    int m_xmin;
    int m_xmax;
    int m_ymin;
    int m_ymax;
    float m_probability; 

    bool checkBox();
};
    
}; // namespace tracking


#endif // BOUNDINGBOX_H