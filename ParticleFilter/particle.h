#ifndef PARTICLE_H
#define PARTICLE_H

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "motiondistribution.h"

/**
  \def CST1
  \brief Constant helpfull to compute the movement of a particle.
  */
#define CST1 2.0
/**
  \def CST2
  \brief Constant helpfull to compute the movement of a particle.
  */
#define CST2 -1.0

/**
 * @brief The Particle class contains the data of a particle in a particle filter.
 * A particle has three continuous parameters : The position of the particle on
 * the x axis of the image, the position of the particle on the y axis and the scale of the particle.
 * @author Elie MOUSSY
 * @date 2015
 */
class Particle
{
    public:
        /**
         * @brief Constructor
         * @param nbrOfContParams : The number of continuous parameters.
         */
        Particle(const cv::Mat &frame);
        /**
          * @brief Destructor.
          */
        ~Particle() {}

        /**
         * @brief This operator handles the affectation of a particle to another one.
         * @param other : The particle to be affected.
         * @return The newly affected particle.
         */
        inline Particle& operator=(const Particle& other)
        {
            continuousParams.noalias() = other.getContinuousParameters();
            previousContinuousParams.noalias() = other.getPreviousContinuousParameters();
            continuousParamsRange.noalias() = other.getContinuousParamRange();
            weight = other.getWeight();
            prevWeight = other.getPrevWeight();
            hist = other.getHistogram();
            hist_initialized = other.isHistogramInitialized();
            return *this;
        }
        /**
         * @brief This method resets the continous parameters.
         */
        void resetContinuousParameters();
        /**
         * @brief This method initializes a particle.
         * @param continuousParams : The initial continuous parameters.
         * @param continuousParamsRange : The continuous parameters range.
         */
        void init(const Eigen::VectorXd& continuousParams, const Eigen::MatrixXd& continuousParamsRange, const double weight=0.);

        /**
         * @brief This method returns the continuous parameters.
         * @return The continuous parameters.
         */
        inline Eigen::VectorXd getContinuousParameters() const { return continuousParams; }
        /**
         * @brief This method sets the continuous parameters to contParams.
         * @param contParams : The new values of the continuous parameters. They will not be set if they are not within the range.
         * @param width : The maximum width of the frame.
         * @param height : The maximum height of the frame.
         * @return True if the new continuous parameters are within the range, false otherwise.
         */
        inline bool setContinuousParameters(const Eigen::VectorXd& contParams, const double width, const double height)
        {
            const int size = contParams.size();
            for (int i=0; i<size; i++)
                if(!(contParams(i)>continuousParamsRange(i,0) && contParams(i)<continuousParamsRange(i,1)))
                    return false;
            if (size == 3)
            {
                if (contParams(0)+contParams(2)*64. > width || contParams(1)+contParams(2)*128.> height)
                    return false;
            } else {
                if (contParams(0)+contParams(2) > width || contParams(1)+contParams(3) > height)
                    return false;
            }
            continuousParams.noalias() = contParams;
            return true;
        }
        /**
         * @brief This method returns the previous continuous parameters.
         * @return The previous continuous parameters.
         */
        inline Eigen::VectorXd getPreviousContinuousParameters() const { return previousContinuousParams; }
        /**
         * @brief This method sets the previous continuous parameters to prevContParams.
         * @param prevContParams : The new values of the previous continuous parameters.
         */
        inline void setPreviousContinuousParameters(const Eigen::VectorXd& prevContParams) { previousContinuousParams = prevContParams; }
        /**
         * @brief This method returns the continuous parameters range.
         * @return The continuous parameters range.
         */
        inline Eigen::MatrixXd getContinuousParamRange() const { return continuousParamsRange; }
        /**
         * @brief This method sets the continuous parameters range to paramsRange.
         * @param paramsRange : The new values of the continuous parameters range.
         */
        inline void setContinuousParametersRange(const Eigen::MatrixXd& paramsRange) { continuousParamsRange = paramsRange; }
        /**
         * @brief This method returns the weight of the particle.
         * @return The weight of the particle.
         */
        inline double getWeight() const { return weight; }
        /**
         * @brief This method sets the weight of the particle to w.
         * @param w : The new weight of the particle.
         * @param updatePrevious : If true, the previous weight will be updated.
         */
        inline void setWeight(double w, bool updatePrevious = true)
        {
            if (updatePrevious)
                prevWeight = weight;
            weight = w;
        }
        /**
         * @brief This method returns the previousWeight of the particle.
         * @return The previous weight of the particle.
         */
        inline double getPrevWeight() const { return prevWeight; }
        /**
         * @brief This method returns the number of continuous parameters.
         * @return The number of continuous parameters.
         */
        inline int getNumberOfContParams() const { return continuousParams.rows(); }
        /**
         * @brief This method computes an HSV histogram from a GBR frame and a region in it.
         * @param frame : The given frame.
         * @param region : The area in the frame from which the histogram will be computed.
         * @return The computed histogram.
         */
        cv::MatND computeHistogram(const cv::Mat& frame, const cv::Rect& region);
        /**
         * @brief This method returns the histogram of the particle.
         * @return The histogram of the particle.
         */
        inline cv::MatND getHistogram() const { return hist; }
        /**
         * @brief This method returns whether the histogram is initialized or not.
         * @return True if the histogram is initialized, false otherwise.
         */
        inline bool isHistogramInitialized() const { return  hist_initialized; }
        /**
         * @brief This method returns the bounding box of the particle.
         * @return The bouning box of the particle.
         */
        inline cv::Rect getBoundingBox() const
        {
            if (continuousParams.rows() == 3)
                return cv::Rect(continuousParams(0), continuousParams(1), continuousParams(2)*64., continuousParams(2)*128.);
            return cv::Rect(continuousParams(0), continuousParams(1), continuousParams(2), continuousParams(3));
        }
        /**
         * @brief This method returns the bhattacharyya distance between the particle histogram and another one.
         * @param otherHist : The other histogram.
         * @return The bhattacharyya distance between the particle histogram and another one.
         */
        double bhattacharyyaDistance(const cv::MatND& otherHist);
        /**
         * @brief This method computes a translation of the particle according to its velocity and a random vector.
         * @param width : The width of the frame.
         * @param height : The height of the frame.
         */
        void moveParticle(const double width, const double height);

    private:
        /**
         * @brief The continuous parameters
         */
        Eigen::VectorXd continuousParams;
        /**
         * @brief The previous continuous parameters
         */
        Eigen::VectorXd previousContinuousParams;
        /**
         * @brief The first continuous parameters of the particle.
         */
        Eigen::VectorXd firstContParams;
        /**
         * @brief The range of the continuous parameters.
         */
        Eigen::MatrixXd continuousParamsRange;
        /**
         * @brief The weight of the particle.
         */
        double weight;
        /**
         * @brief The previous weight of the particle.
         */
        double prevWeight;
        /**
         * @brief The histogram of the particle.
         */
        cv::MatND hist;
        /**
         * @brief Flag that indicates whether the histogram is initialized or not.
         */
        bool hist_initialized;
        /**
         * @brief The motion distribution model.
         */
        MotionDistribution md;
};

#endif // PARTICLE_H
