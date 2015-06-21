#ifndef PARTICLEFILTERDEMONSTRATOR_H
#define PARTICLEFILTERDEMONSTRATOR_H

#include <opencv2/opencv.hpp>
#include <vector>
#include "particlefilter.h"
#include "observation.h"

/**
 * @brief The ParticleFilterDemonstrator class implements a demonstrator in order to handle the particle filter in Single Object Tracking (SOT)
 * or in Multi-Object Tracking (MOT).
 * @author Elie MOUSSY
 * @date 2015
 */
class ParticleFilterDemonstrator
{
    public:
        /**
         * @brief The tracking mode of the particle filter. It can be a Single Object Tracking (SOT) or
         * a Multi Object Tracking (MOT).
         */
        enum Mode {
            SOT,
            MOT
        };
        /**
         * @brief Constructor.
         * @param m : The tracking mode (MOT or SOT).
         * @param trackCreationThreshold : The number of associated detections in order to create a track (only in MOT).
         * @param trackKillingThreshold : The number of frames without associating a detection to a track before killing it (only in MOT).
         * @param detectionKillingThreshold : The number of frames without associating a detection to another one in order to kill it.
         */
        ParticleFilterDemonstrator(Mode m=MOT, int trackCreationThreshold=3, int trackKillingThreshold=10, int detectionKillingThreshold=3);
        /**
         * @brief This method computes the necessary for the next (given) frame.
         * @param frame : The given frame.
         * @param detections : The detections made on the frame.
         */
        void step(const cv::Mat& frame, const std::vector<Observation> &detections);
        /**
         * @brief This method returns a vector of the bounding boxes of the activated tracks.
         * @return A vector of the bounding boxes of the activated tracks.
         */
        inline std::vector<ParticleFilter> getTracks() const
        {
            std::vector<ParticleFilter> res;
            const unsigned int size = pfList.size();
            for (unsigned int i=0; i<size; i++)
                if (pfList.at(i).isActivated())
                    res.push_back(pfList.at(i));
            return res;
        }
        /**
         * @brief This method draws all the active tracks to a given frame.
         * @param frame : The frame on which the active tracks will be drawn.
         */
        inline void showActiveTracks(cv::Mat& frame) const
        {
            const unsigned int size = pfList.size();
            for (unsigned int i=0; i<size; i++)
                if (pfList.at(i).isActivated())
                {
                    const cv::Rect rec = pfList.at(i).getBoundingBox();
                    cv::rectangle(frame, rec, cv::Scalar(255, 0, 0));
                    std::stringstream ss;
                    ss << pfList[i].getID();
                    cv::putText(frame, ss.str(), cv::Point(rec.x+5, rec.y+5), cv::FONT_HERSHEY_PLAIN, 1,
                                cv::Scalar(255,0,0));
                }
        }

    private:
        /**
         * @brief The tracking mode (SOT or MOT).
         */
        Mode m;
        /**
         * @brief The list of all the tracks (just a single one in SOT mode).
         */
        std::vector<ParticleFilter> pfList;
        /**
         * @brief A counter to deactivate a particle filter.
         */
        std::vector<int> killList;
        /**
         * @brief A counter to deactivate a detection on the waiting list.
         */
        std::vector<int> killWaitingList;
        /**
         * @brief Some detections waiting to create a track.
         */
        std::vector<std::pair<Observation, int> > waitingList;
        /**
         * @brief Number of matched detections in order to create a track.
         */
        int trackCreationThresh;
        /**
         * @brief Number of unmatched track-detection before deactivating a track.
         */
        int trackKillingThresh;
        /**
         * @brief Number of unmatched detection-detection before removing a detection from the waiting list.
         */
        int detectionKillingThresh;
        /**
         * @brief The next particle filter ID.
         */
        int ID;
};

#endif // PARTICLEFILTERDEMONSTRATOR_H
