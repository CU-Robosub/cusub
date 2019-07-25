/**
 * @file ObjectTracker.h
 * @author Soroush Khadem (soroush.khadem@colorado.edu)
 * @brief Tracks objects
 * 
 */
#ifndef OBJECTTRACKER_H
#define OBJECTTRACKER_H

#include "BoundingBox.h"
#include "KLTPointTracker.h"
#include "ImageData.h"

namespace perception_control
{

class ObjectTracker
{
public:
    ObjectTracker(BoundingBox &box, const ImageData &image);
    ~ObjectTracker();

    void initialize(BoundingBox &box, const ImageData &image);
    void updateImage(const ImageData &image);

    BoundingBox currentBox() const;
    ImageData currentImage() const;
    bool isValid();

private:
    PointTracker * m_pointTracker;
    BoundingBox m_boundingBox;

    bool m_valid;
    ImageData m_currentImage;
};

}; // namespace perception_control

#endif // OBJECTTRACKER_H