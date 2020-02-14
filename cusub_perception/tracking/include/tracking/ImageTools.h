#ifndef IMAGETOOLS_H
#define IMAGETOOLS_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace tracking
{

class ImageTools
{
public:
    struct Actions
    {
        bool CVT_GRAY    = false;
        bool RESIZE      = false;
        bool CVT_RGB2BGR = false;
        bool CVT_BGR2RGB = false;
    };
    struct Preprocessing
    {
        Actions actions;
        cv::Size size;
        
        Preprocessing() {};
        Preprocessing(Actions _actions) :
            actions(_actions) {};
        Preprocessing(Actions _actions, cv::Size _size) :
            actions(_actions), size(_size) {};
    };

    static void processImage (const cv::Mat &in, cv::Mat &out, const Preprocessing &steps);
    static bool checkRoi(const cv::Mat &image, const cv::Rect &roi);

};

};

#endif // IMAGETOOLS_H