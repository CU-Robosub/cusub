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
#include "ImageTools.h"

namespace tracking
{

class ObjectTracker
{
public:
    ObjectTracker(BoundingBox &box, const ImageData &image, const std::string &classname);
    ~ObjectTracker();

    void initialize(BoundingBox &box, const ImageData &image);
    void updateImage(const ImageData &image);

    std::string classname() const;
    BoundingBox currentBox() const;
    cv::Mat currentImage() const;
    ImageData currentImageData() const;
    bool isValid();

private:
    PointTracker * m_pointTracker;
    BoundingBox m_boundingBox;

    std::string m_classname;
    bool m_valid;
    ImageData m_currentImage;
    ImageTools::Preprocessing m_preprocessSteps;
    
};

}; // namespace tracking

#endif // OBJECTTRACKER_H