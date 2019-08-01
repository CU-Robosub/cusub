#include "tracking/ObjectTracker.h"

#include <iostream>

using namespace tracking;

const int MOVEMENT_THRESH = 20000; // in pixels
const int AREA_CHANGE_THRESH = 20000; // in pixels

ObjectTracker::ObjectTracker(BoundingBox &box, const ImageData &image, const std::string &classname) :
    m_pointTracker(new KLTPointTracker()),
    m_boundingBox(box),
    m_classname(classname),
    m_valid(false)
{
    initialize(box, image);
}

ObjectTracker::~ObjectTracker()
{
    delete m_pointTracker;
}

void ObjectTracker::initialize(BoundingBox &box, const ImageData &image)
{
    PointTracker::Result result = m_pointTracker->initialize(image.cvImage(), box);
    m_valid = result.status == PointTracker::STATUS::Success;
    m_boundingBox = box;
    m_currentImage = image;

    ImageTools::Actions actions;
    actions.RESIZE = true;
    actions.CVT_GRAY = true;
    m_preprocessSteps = ImageTools::Preprocessing(actions, image.cvImage().size());
}

std::string ObjectTracker::classname() const
{
    return m_classname;
}

BoundingBox ObjectTracker::currentBox() const
{
    return m_boundingBox;
}

cv::Mat ObjectTracker::currentImage() const
{
    cv::Mat imgOut;
    ImageTools::processImage(m_currentImage.cvImage(), imgOut, m_preprocessSteps);
    return imgOut;
}

ImageData ObjectTracker::currentImageData() const
{
    return m_currentImage;
}

bool ObjectTracker::isValid()
{
    return m_valid;
}

void ObjectTracker::updateImage(const ImageData &image)
{
    m_currentImage = image;
    cv::Mat processedImage;
    if (m_valid)
    {
        ImageTools::processImage(image.cvImage(), processedImage, m_preprocessSteps);
        PointTracker::Result result = m_pointTracker->trackPoints(processedImage);

        if (result.status == PointTracker::STATUS::Success)
        {
            cv::Point2f oldCenter = m_boundingBox.center();
            int oldArea = m_boundingBox.area();
            m_boundingBox.setTransform(result.transform);
            cv::Point2f newCenter = m_boundingBox.center();
            int newArea = m_boundingBox.area();


            if (cv::norm(oldCenter - newCenter) > MOVEMENT_THRESH)
            {
                m_valid = false;
                std::cout << "Too much movement: " << std::to_string(cv::norm(oldCenter - newCenter)) << std::endl;
            }
            else if (abs(oldArea - newArea) > AREA_CHANGE_THRESH)
            {
                m_valid = false;
                std::cout << "Too much area change: " << std::to_string(abs(oldArea - newArea)) << std::endl;
            }
            else
            {
                m_valid = m_boundingBox.isValid();
            }
        }
        else
        {
            m_valid = false;
        }
    }
}
