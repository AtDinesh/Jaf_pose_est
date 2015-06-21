#ifndef OBSERVATION_H
#define OBSERVATION_H

#include <opencv2/opencv.hpp>

/**
 * @brief The Observation class implements an observation model that represents a detection or a track.
 * @author Elie MOUSSY
 * @date 2015
 */
class Observation
{
    public:
        /**
         * @brief Constructor.
         */
        Observation() { hist_initialized = false; }
        /**
         * @brief Constructor.
         * @param frame : The frame used for the observation model.
         * @param boundingBox : The bounding box of the observation model.
         */
        Observation(const cv::Mat& frame, const cv::Rect& boundingBox);
        /**
         * @brief This method returns the bounding box of the observation model.
         * @return The bounding box of the observation model.
         */
        inline cv::Rect getBoundingBox() const { return bb; }
        /**
         * @brief This method returns the histogram of the observation model.
         * @return The histogram of the observation model.
         */
        inline cv::MatND getHistogram() const { return hist; }

        /**
         * @brief This method computes an HSV histogram from a GBR frame and a region in it.
         * @param frame : The given frame.
         * @param region : The area in the frame from which the histogram will be computed.
         * @return The computed histogram.
         */
        cv::MatND computeHistogram(const cv::Mat& frame, const cv::Rect& region);
        /**
         * @brief This method returns the bhattacharyya distance between the observation model histogram and another one.
         * @param otherHist : The other histogram.
         * @return The bhattacharyya distance between the observation model histogram and another one.
         */
        double bhattacharyyaDistance(const cv::MatND& otherHist) const;
        /**
         * @brief This method returns the center of the observation.
         * @return The center of the observation.
         */
        inline cv::Point2d getCenter() const { return cv::Point2d((double)bb.x+(double)bb.width/2., (double)bb.y+(double)bb.height/2.); }
        /**
         * @brief This method returns the scale of the observation.
         * @return The scale of the observation.
         */
        inline double getScale() const { return (double)bb.width/64.; }
        /**
          * @brief This method returns the gaussian distance between two points in one dimension.
          * @param x1 : Coordinate of the first point.
          * @param x2 : Coordinate of the second point.
          * @param sig : A coefficient.
          * @return The following results : \f$ exp(-0.5*(\frac{x1-x2}{sig})^2) \f$.
          */
        inline double gaussian1DDistance(const double x1, const double x2, const double sig) const
        {
            const double tmp = (x1-x2)/sig;
            return std::exp(-0.5*tmp*tmp);
        }
        /**
          * @brief This method returns the gaussian distance between two points in two dimensions.
          * @param x1 : Coordinate of the first point on the x axis.
          * @param y1 : Coordinate of the first point on the y axis.
          * @param x2 : Coordinate of the second point on the x axis.
          * @param y2 : Coordinate of the second point on the y axis.
          * @param sig1 : A coefficient.
          * @param sig2 : A coefficient.
          * @return The following results : \f$ exp(-0.5*(\frac{x1-x2}{sig})^2) \f$.
          */
        inline double gaussian2DDistance(const double x1, const double y1, const double x2, const double y2, const double sig1, const double sig2) const
        {
            const double tmpx = (x1-x2)/sig1;
            const double tmpy = (y1-y2)/sig2;
            return std::exp(-0.5*(tmpx*tmpx + tmpy*tmpy));
        }
        /**
         * @brief This method returns whether the histogram is initialized or not.
         * @return True if the histogram is initilized, false otherwise.
         */
        inline bool isHistogramInitialized() const { return hist_initialized; }
        /**
         * @brief This operator handles the compare operator== with another Observation.
         * @param other : The other Observation.
         * @return True if *this = other, false otherwise.
         */
        inline bool operator ==(const Observation& other)
        {
            return (bb == other.getBoundingBox());
        }
        /**
         * @brief This operator handles the affectation of an Observation object to another one.
         * @param other : The Observation object that will be affected.
         * @return The newly affected Observation object.
         */
        inline Observation& operator =(const Observation& other)
        {
            bb = other.getBoundingBox();
            hist = other.getHistogram();
            hist_initialized = other.isHistogramInitialized();
            return *this;
        }
        /**
         * @brief This method returns the frame in which the histogram is computed.
         * @return The frame in which the histogram is computed.
         */
        inline cv::Mat getFrame() const { return frame; }
        /**
         * @brief This method sets the coordinate on the x axis of the bounding box.
         * @param x : The new value of the coordinate.
         */
        inline void setX(int x) { bb.x = x; }
        /**
         * @brief This method sets the coordinate on the y axis of the bounding box.
         * @param y : The new value of the coordinate.
         */
        inline void setY(int y) { bb.y = y; }
        /**
         * @brief This method sets the width of the bounding box.
         * @param w : The new value of the width.
         */
        inline void setWidth(int w) { bb.width = w; }
        /**
         * @brief This method sets the height of the bounding box.
         * @param h : The new value of the height.
         */
        inline void setHeight(int h) { bb.height = h; }
        /**
         * @brief This method sets the observation frame to f.
         * @param f : The new observation frame.
         */
        inline void setFrame(cv::Mat f) { frame = f; }

    private:
        /**
         * @brief The observation bounding box.
         */
        cv::Rect bb;
        /**
         * @brief The histogram of the bounding box.
         */
        cv::MatND hist;
        /**
         * @brief Flag that indicates whether the histogram is initialized or not.
         */
        bool hist_initialized;
        /**
         * @brief The frame in which the histogram is computed.
         */
        cv::Mat frame;
};

#endif // OBSERVATION_H
