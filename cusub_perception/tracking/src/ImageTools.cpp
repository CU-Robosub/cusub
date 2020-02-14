#include "tracking/ImageTools.h"

using namespace tracking;

void ImageTools::processImage(const cv::Mat &in, cv::Mat &out, const ImageTools::Preprocessing &steps)
{
    out = in;
    if (steps.actions.RESIZE)
    {
        cv::resize(out, out, steps.size);
    }

    if (steps.actions.CVT_GRAY)
    {
        cv::cvtColor(out, out, CV_BGR2GRAY);
    }

    if (steps.actions.CVT_RGB2BGR)
    {
        cv::cvtColor(out, out, cv::COLOR_RGB2BGR);
    }

    if (steps.actions.CVT_BGR2RGB)
    {
        cv::cvtColor(out, out, cv::COLOR_BGR2RGB);
    }
}

/**
 * @brief Checks if it is safe to crop the image to the roi
 * 
 * @param image 
 * @param roi 
 * @return true 
 * @return false 
 */
bool ImageTools::checkRoi(const cv::Mat &image, const cv::Rect &roi)
{
    return (
        roi.x > 0 && roi.y > 0 &&
        roi.width > 0 && roi.height > 0 &&
        roi.x + roi.width < image.cols &&
        roi.y + roi.height < image.rows
    );
}