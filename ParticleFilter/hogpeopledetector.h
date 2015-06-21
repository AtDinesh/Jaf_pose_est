#ifndef HOGPEOPLEDETECTOR_H
#define HOGPEOPLEDETECTOR_H

#include "icu_detector_base.h"
#include <opencv2/gpu/gpu.hpp>

/**
 * @brief This class implements a basic HOG people detector.
 */
class HOGPeopleDetector : public ICU_Detector_Base
{
    public:
        /**
         * @brief Constructor.
         */
        HOGPeopleDetector();
        /**
         * @brief Constructor.
         * @param frame : The input frame. This frame will help to initialize the detector thanks to its width and height.
         * @param ID : The identity of the detector. By default, it is set to -1.
         */
        HOGPeopleDetector(const cv::Mat& frame, const int ID=-1);

        /**
         * @brief This method process a people detections from a given image.
         * @param frame : The image used to process the detection.
         * @return A gaussian mixture that corresponds to the detections.
         */
        virtual GaussianMixture detect(const cv::Mat& frame);

    private:
        /**
         * @brief The gpu hog descriptor of opencv.
         */
        cv::gpu::HOGDescriptor HOGdesc;
};

#endif // HOGPEOPLEDETECTOR_H
