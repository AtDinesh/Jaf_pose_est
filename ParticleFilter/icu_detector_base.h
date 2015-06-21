#ifndef ICU_DETECTOR_BASE_H
#define ICU_DETECTOR_BASE_H

#include <opencv2/opencv.hpp>
#include "observation.h"
#include "gaussianmixture.h"

/**
 * @brief The ICU_Detector_Base class implements an abstract two dimentional detector.
 * @author Elie MOUSSY
 * @date 2015
 */
class ICU_Detector_Base
{
    public:
        /**
         * @brief Constructor.
         */
        ICU_Detector_Base() : ID(-1) {}
        /**
         * @brief Constructor.
         * @param frame : The input frame. This frame will help to initialize the detector thanks to its width and height.
         * @param ID : The identity of the detector. By default, it is set to -1.
         */
        ICU_Detector_Base(const cv::Mat& frame, int ID=-1);

        /**
         * @brief This method displays the last detections on a given frame. These detections will be drawn with a bounding box
         * whith a given thickness and a given color. It also allows to draw the center of the detections by setting showCenter to true.
         * @param frame : The frame on which the detections will be drawn.
         * @param thickness : The thickness of the bounding boxes representing the detections (1 by default).
         * @param color : The color of the bounding boxes representing the detections (black by default).
         * @param showCenter : If this parameter is set to true, the method will draw the centers of the detections (false by default).
         */
        void displaydetections(cv::Mat& frame, const int thickness=1, const cv::Scalar color=cv::Scalar(0,0,0), const bool showCenter=false);
        /**
         * @brief This method process the detections from a given image.
         * @param frame : The image used to process the detection.
         * @return A gaussian mixture that corresponds to the detections.
         */
        virtual GaussianMixture detect(const cv::Mat& frame)=0;
        /**
         * @brief This method returns a vector containing all the detections.
         * @return A vector containing all the detections.
         */
        std::vector<Observation> getDetections() const { return detections; }
        /**
         * @brief This method changes the ID of the detector.
         * @param newID : The new ID of the detector.
         */
        void setID(int newID) { ID = newID; }
        /**
         * @brief This method returns the ID of the detector.
         * @return The ID of the detector.
         */
        int getID() const { return ID; }
        /**
         * @brief This method returns the width of the frame of the detector.
         * @return The width of the frame of the detector.
         */
        int getWidth() const { return width; }
        /**
         * @brief This method returns the height of the frame of the detector.
         * @return The height of the frame of the detector.
         */
        int getHeight() const { return height; }

    protected:
        /**
         * @brief This method sets the width of the frame of the detector to newWidth.
         * @param newWidth : The new width of the frame of the detector.
         */
        void setWidth(int newWidth) { width = newWidth; }
        /**
         * @brief This method sets the height of the frame of the detector to newHeight.
         * @param newHeight : The new height of the frame of the detector.
         */
        void setHeight(int newHeight) { height = newHeight; }
        /**
         * @brief This method sets the detections to newDetections.
         * @param newDetections : The new detections of the detector.
         */
        void setDetections(std::vector<Observation> newDetections)
        {
            detections.clear();
            detections = newDetections;
        }
        /**
         * @brief This method clears all the detectections.
         */
        void clearDetections() { detections.clear(); }

    private:
        /**
         * @brief The detector ID.
         */
        int ID;
        /**
         * @brief The width of the frame in which the detections will be computed.
         */
        int width;
        /**
         * @brief The height of the frame in which the detections will be computed.
         */
        int height;
        /**
         * @brief The vector of computed detections.
         */
        std::vector<Observation> detections;
};

#endif // ICU_DETECTOR_BASE_H
