#ifndef MOTIONDISTRIBUTION_H
#define MOTIONDISTRIBUTION_H

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

/**
 * @brief The MotionDistribution class implements the motion distribution of a particle in the obervation model of the particle filter.
 */
class MotionDistribution
{
    public:
        /**
         * @brief Constructor.
         * @param nbOfBins : The number of bins.
         * @param nbOfParts : The number of parts in the model.
         * @param width : The width of the model.
         * @param height : The height of the model.
         */
        MotionDistribution(const int nbOfBins, const int nbOfParts, const int width, const int height);
        /**
         * @brief Destructor.
         */
        ~MotionDistribution() {}

        /**
         * @brief This method computes a mask for a given histogram.
         * @param img : The input image.
         * @param rec : The bounding box in which the particle is located.
         * @param histNum : The index of the histogram.
         * @param increment : The increment step of the mask.
         */
        void calcFromRect(const cv::Mat& img, const cv::Rect &rec, const int histNum, const double increment);
        /**
         * @brief This method computes a mask for all the histograms.
         * @param img : The input image.
         * @param rec : The bounding box in which the particle is located.
         * @param nbPix : The increment step of the mask for each histogram.
         */
        void calcFromRectList(const cv::Mat& img, const cv::Rect &rec, const Eigen::VectorXi &nbPix);

        /**
         * @brief This method computes the bhattacharaya distance between all histograms.
         * @param sigma : The deviation of the distance.
         * @return : The bhattacharaya distance.
         */
        double bhattacharayaDistance(const double sigma);

    private:
        /**
         * @brief The histograms data.
         */
        Eigen::MatrixXd data;
        /**
         * @brief A reference histogram.
         */
        Eigen::VectorXd uniformHist;
        /**
         * @brief The mask of the bins of the frame.
         */
        Eigen::MatrixXi binsMask;
};

#endif // MOTIONDISTRIBUTION_H
