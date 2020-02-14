/**
 * @file ImageData.h
 * @author Soroush Khadem (soroush.khadem@colorado.edu)
 * @brief Holds an image message and can handle various conversions
 * 
 */
#ifndef IMAGEDATA_H
#define IMAGEDATA_H

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>

namespace tracking
{

class ImageData
{
public:
    ImageData();
    ImageData(const sensor_msgs::Image::ConstPtr &image);
    ImageData(const sensor_msgs::Image &image);
    ImageData(const ImageData &other);
    
    void setImage(const cv::Mat &image);
    void setFrameId(const std::string &id);

    std::string frameId() const;
    bool empty();
    cv::Mat cvImage() const;
    sensor_msgs::Image::ConstPtr rosImage() const;

    static sensor_msgs::Image cvImagetoROS(const cv::Mat &image, std_msgs::Header * header = nullptr);

private:
    bool m_empty;
    sensor_msgs::Image::ConstPtr m_image;
    std::string m_frameId;

    cv::Mat rosImageToCV(const sensor_msgs::Image &image);
    // sensor_msgs::Image cvImagetoROS(const cv::Mat &image);

};

}; // tracking

#endif // IMAGEDATA_H