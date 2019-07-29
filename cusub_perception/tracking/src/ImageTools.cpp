#include "tracking/ImageTools.h"

using namespace tracking;

void ImageTools::processImage(const cv::Mat &in, cv::Mat &out, const ImageTools::Preprocessing &steps)
{
    if (steps.actions.RESIZE)
    {
        cv::resize(in, out, steps.size);
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
