#include "icu_detector_base.h"

ICU_Detector_Base::ICU_Detector_Base(const cv::Mat &frame, int ID):
    ID(ID),
    width(frame.cols),
    height(frame.rows)
{
}

void ICU_Detector_Base::displaydetections(cv::Mat& frame, const int thickness, const cv::Scalar color, const bool showCenter)
{
    const unsigned int size = detections.size();
    for (unsigned int i=0; i<size; i++)
    {
        const Observation& det = detections.at(i);
        cv::rectangle(frame, det.getBoundingBox(), color, thickness);
        if (showCenter)
            cv::circle(frame, cv::Point(det.getCenter().x, det.getCenter().y), 2, color, thickness);
    }
}
