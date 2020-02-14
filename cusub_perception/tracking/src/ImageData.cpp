#include "tracking/ImageData.h"

#include <iostream>

#include <opencv2/highgui.hpp>

using namespace tracking;

ImageData::ImageData() :
    m_empty(true)
{}

ImageData::ImageData(const sensor_msgs::Image::ConstPtr &image) :
    m_empty(false)
{
    m_image = image;
    m_frameId = m_image->header.frame_id;
}

ImageData::ImageData(const sensor_msgs::Image &image) :
    m_image(sensor_msgs::ImageConstPtr(new sensor_msgs::Image(image))),
    m_empty(false)
{
    m_frameId = m_image->header.frame_id;
}

ImageData::ImageData(const ImageData &other) :
    m_image(other.m_image),
    m_empty(other.m_empty),
    m_frameId(other.m_frameId)
{}

void ImageData::setImage(const cv::Mat &image)
{
    std_msgs::Header header;
    header.seq = 0;
    header.stamp = ros::Time();
    header.frame_id = "cv";
    // setup bridge
    cv_bridge::CvImage imgBridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image);
    
    // convert
    sensor_msgs::Image imgMsg;
    imgBridge.toImageMsg(imgMsg);
    m_image = sensor_msgs::ImageConstPtr(new sensor_msgs::Image(imgMsg));
    m_frameId = m_image->header.frame_id;
    m_empty = false;
}

void ImageData::setFrameId(const std::string &id)
{
    m_frameId = id;
}

std::string ImageData::frameId() const
{
    if (m_empty)
    {
        return "empty";
    }
    return m_frameId;
}

bool ImageData::empty()
{
    return m_empty;
}

cv::Mat ImageData::cvImage() const
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(m_image, sensor_msgs::image_encodings::RGB8);        
    return cv_ptr->image;
}

sensor_msgs::Image::ConstPtr ImageData::rosImage() const
{
    return m_image;
}

cv::Mat ImageData::rosImageToCV(const sensor_msgs::Image &image)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);
    return cv_ptr->image;
}

sensor_msgs::Image ImageData::cvImagetoROS(const cv::Mat &image, std_msgs::Header * header)
{
    // get header
    if (header == nullptr)
    {
        header = new std_msgs::Header();
        header->stamp = ros::Time::now();
    }

    // setup bridge
    cv_bridge::CvImage imgBridge = cv_bridge::CvImage(*header, sensor_msgs::image_encodings::BGR8, image);
    
    // convert
    sensor_msgs::Image imgMsg;
    imgBridge.toImageMsg(imgMsg);

    return imgMsg;
}