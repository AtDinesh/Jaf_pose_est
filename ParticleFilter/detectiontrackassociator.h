#ifndef DETECTIONTRACKASSOCIATOR_H
#define DETECTIONTRACKASSOCIATOR_H

#include <vector>
#include <opencv.hpp>
#include "observation.h"

/**
 * @brief The DetectionTrackAssociator class handles the association between two observations using the greedy or the hungarian algorithm.
 */
class DetectionTrackAssociator
{
    public:
        /**
         * @brief The Mode of the algorithm to perform for the association.
         */
        enum Mode {
            GREEDY,
            HUNGARIAN
        };
        /**
         * @brief Constructor.
         * @param m : The mode of the algorithm to run for the association.
         * @param threshold : The threshold below which the score will be ignored.
         */
        DetectionTrackAssociator(Mode m, const double threshold=0.3);
        /**
         * @brief This method runs the algorithm of association.
         * @param tracks : The list of the tracks to be associated.
         * @param detections : The list of the detections to be associated.
         */
        void run(const std::vector<Observation>& tracks, const std::vector<Observation>& detections);
        /**
         * @brief This method returns the algorithm used for the association.
         * @return The algorithm used for the association.
         */
        inline Mode getMode() const { return m; }
        /**
         * @brief This method returns the threshold below which the score will be ignored.
         * @return The threshold below which the score will be ignored.
         */
        inline double getThreshold() const { return thresh; }
        /**
         * @brief This method returns the matched associators.
         * @return The matched associators.
         */
        inline std::vector< std::pair< Observation, Observation > > getMatches() const { return matches; }
        /**
         * @brief This method returns the unmatched tracks.
         * @return The unmatched tracks.
         */
        inline std::vector< Observation > getUnmatchedTracks() const { return unmatchedTracks; }
        /**
         * @brief This method returns the unmatched detections.
         * @return The unmatched detections.
         */
        inline std::vector< Observation > getUnmatchedDetects() const { return unmatchedDetecs; }
        /**
         * @brief This method will set the algorithm of the association to m.
         * @param m : The new algorithm of the association.
         */
        inline void setMode(Mode m) { this->m = m; }
        /**
         * @brief This method sets the threshold below which the score will be ignored to threshold.
         * @param threshold : The new value of the threshold.
         */
        inline void setThreshold(double threshold) { thresh = threshold; }

    private:
        /**
         * @brief The mode of the association. (GREEDY or HUNGARIAN)
         */
        Mode m;
        /**
         * @brief The threshold below which the score will be ignored.
         */
        double thresh;
        /**
         * @brief A vector of a pair of cv::Rect that corresponds to the matched associators.
         */
        std::vector< std::pair< Observation, Observation > > matches;
        /**
         * @brief A vector of the unmatched tracks.
         */
        std::vector< Observation > unmatchedTracks;
        /**
         * @brief A vector of the unmatched detections.
         */
        std::vector< Observation > unmatchedDetecs;
};

#endif // DETECTIONTRACKASSOCIATOR_H
