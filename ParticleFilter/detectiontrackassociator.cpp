#include "detectiontrackassociator.h"
#include "munkres/munkres.h"
#include <eigen3/Eigen/Dense>
#include <iostream>

DetectionTrackAssociator::DetectionTrackAssociator(Mode m, const double threshold):
    m(m),
    thresh(threshold)
{
}

template<typename t> t minimum(t a, t b)
{
    return (a < b) ? a : b;
}

void DetectionTrackAssociator::run(const std::vector<Observation> &tracks, const std::vector<Observation> &detections)
{
    // Initialization
    matches.clear();
    unmatchedDetecs.clear();
    unmatchedTracks.clear();
    const unsigned int detectionSize = detections.size();
    const unsigned int tracksSize = tracks.size();

    // Case where there is no detection
    if (detectionSize == 0)
    {
        for (unsigned int i=0; i<tracksSize; i++)
            unmatchedTracks.push_back(tracks[i]);
        return;
    }

    // Case where there is no tracks
    if (tracksSize == 0)
    {
        for (unsigned int i=0; i<detectionSize; i++)
            unmatchedDetecs.push_back(detections[i]);
        return;
    }

    // Creation of the score matrix
    Eigen::MatrixXd scoreMat(tracksSize, detectionSize);

    /**
                  __det__
                 |       |
               tr|       |
                 |_______|
                             **/

    // Fill the score matrix
    for (unsigned int i=0; i<tracksSize; i++)
    {
        for(unsigned int j=0; j<detectionSize; j++)
        {
            const Observation& detec = detections.at(j);
            const Observation& track = tracks.at(i);
            // Histograms
            double h_score = track.bhattacharyyaDistance(detec.getHistogram());

            // Centers distance
            const cv::Point2d trackCenter = track.getCenter();
            const cv::Point2d detecCenter = detec.getCenter();
            double d_score = track.gaussian2DDistance(trackCenter.x, trackCenter.y,
                                                      detecCenter.x, detecCenter.y, 60., 40.);

            // Scale
            double s_score = track.gaussian1DDistance(track.getScale(), detec.getScale(), 0.4);

            // Update score matrix
            scoreMat(i,j) = h_score*d_score*s_score;
        }
    }
    std::cout << "score matrix = \n" << scoreMat << std::endl;

    // Run the algorithm
    if (m == GREEDY)
    {
        std::vector<int> matchedTracks;
        std::vector<int> matchedDetecs;

        // Get the maximum coefficient of the score matrix
        const unsigned int tmp = minimum(detectionSize, tracksSize);
        for (unsigned int i=0; i<tmp; i++)
        {
            int max_row, max_col;   // The location of the maximum coefficient
            if (scoreMat.maxCoeff(&max_row,&max_col) >= thresh)
            {
                if (max_col>=0 && max_col < (int)detectionSize && max_row>=0 && max_row < (int)tracksSize)
                {
                    // Update the matched vector
                    std::pair< Observation, Observation > p(tracks[max_row],detections[max_col]);
                    matches.push_back(p);
                    scoreMat.row(max_row).setZero();
                    scoreMat.col(max_col).setZero();
                    matchedDetecs.push_back((int)max_col);
                    matchedTracks.push_back((int)max_row);
                }
            } else {
                break;
            }
        }
        // Fill the unmatched tracks.
        const int scoreMatRows = scoreMat.rows();
        const unsigned int matchedTracksSize = matchedTracks.size();
        for (int i=0; i<scoreMatRows; i++)
        {
            bool cont = true;
            for (unsigned int j=0; j < matchedTracksSize && cont; j++)
            {
                const int trackNbr = matchedTracks[j];
                if (i == trackNbr)
                    cont = false;
                if (i < trackNbr)
                {
                    unmatchedTracks.push_back(tracks.at((unsigned int)i));
                    cont = false;
                }
            }
        }

        // Fill the unmatched detections
        const int scoreMatCols = scoreMat.cols();
        const unsigned int matchedDetecsSize = matchedDetecs.size();
        for (int i=0; i<scoreMatCols; i++)
        {
            bool cont = true;
            for (unsigned int j=0; j < matchedDetecsSize && cont; j++)
            {
                const int detecNbr = matchedDetecs[j];
                if (i == detecNbr)
                    cont = false;
                if (i < detecNbr)
                {
                    unmatchedDetecs.push_back(detections.at((unsigned int)i));
                    cont = false;
                }
            }
        }
    } else {    // HUNGARIAN
        Eigen::MatrixXd assignments;
        Munkres(scoreMat,assignments);

        std::vector<int> matchedTracks;
        std::vector<int> matchedDetecs;

        // Get the maximum coefficient of the score matrix
        const unsigned tmp = minimum(detectionSize, tracksSize);
        for (unsigned int i=0; i<tmp; i++)
        {
            int max_row, max_col;   // The location of the maximum coefficient
            if (assignments.maxCoeff(&max_row,&max_col) >= thresh)
            {
                if (max_col>=0 && max_col < (int)detectionSize && max_row>=0 && max_row < (int)tracksSize)
                {
                    // Update the matched vector
                    std::pair< Observation, Observation > p(tracks[max_row],detections[max_col]);
                    matches.push_back(p);
                    assignments(max_row,max_col) = 0;
                    matchedDetecs.push_back((int)max_col);
                    matchedTracks.push_back((int)max_row);
                }
            } else {
                break;
            }
        }
        // Fill the unmatched tracks.
        const int assignmentsRows = assignments.rows();
        const unsigned int matchedTracksSize = matchedTracks.size();
        for (int i=0; i<assignmentsRows; i++)
        {
            bool cont = true;
            for (unsigned int j=0; j < matchedTracksSize && cont; j++)
            {
                const int trackNbr = matchedTracks[j];
                if (i == trackNbr)
                    cont = false;
                if (i < trackNbr)
                {
                    unmatchedTracks.push_back(tracks.at((unsigned int)i));
                    cont = false;
                }
            }
        }

        // Fill the unmatched detections
        const int assignmentsCols = assignments.cols();
        const unsigned int matchedDetecsSize = matchedDetecs.size();
        for (int i=0; i<assignmentsCols; i++)
        {
            bool cont = true;
            for (unsigned int j=0; j < matchedDetecsSize && cont; j++)
            {
                const int detecNbr = matchedDetecs[j];
                if (i == detecNbr)
                    cont = false;
                if (i < detecNbr)
                {
                    unmatchedDetecs.push_back(detections.at((unsigned int)i));
                    cont = false;
                }
            }
        }
    }
}
