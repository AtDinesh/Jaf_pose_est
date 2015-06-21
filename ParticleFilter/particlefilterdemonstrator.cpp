#include "particlefilterdemonstrator.h"
#include "detectiontrackassociator.h"

ParticleFilterDemonstrator::ParticleFilterDemonstrator(Mode m, int trackCreationThreshold, int trackKillingThreshold, int detectionKillingThreshold) :
    m(m),
    trackCreationThresh(trackCreationThreshold),
    trackKillingThresh(trackKillingThreshold),
    detectionKillingThresh(detectionKillingThreshold),
    ID(1)
{
}

void ParticleFilterDemonstrator::step(const cv::Mat &frame, const std::vector<Observation> &detections)
{
    if (m == SOT)
    {
        if (pfList.size()==0)
        {
            ParticleFilter pf(frame, 50);
            if (detections.size())
                pf.initParticles(frame, detections[0].getBoundingBox(), 4, true);
            pfList.push_back(pf);
        } else {
            pfList[0].step(frame, detections);
        }
    } else {    // MOT
        // Get the activated tracks observation
        std::vector<Observation> activeTracks;
        const unsigned int pfListSize = pfList.size();
        for(unsigned int i=0; i<pfListSize; i++)
        {
            if (pfList[i].isActivated())
                activeTracks.push_back(pfList[i].getObservation());
        }

        // Try to associate new detections with existing active tracks.
        DetectionTrackAssociator associator(DetectionTrackAssociator::HUNGARIAN,0.3);
        associator.run(activeTracks, detections);

        // Get the results of the association
        std::vector< std::pair<Observation, Observation> > matches = associator.getMatches();
        std::vector< Observation > unmatchedTracks = associator.getUnmatchedTracks();
        std::vector< Observation > unmatchedDetecs = associator.getUnmatchedDetects();

        // Update existing tracks with matched detections
        const unsigned int matchesSize = matches.size();
        for(unsigned int i=0; i<matchesSize; i++)
        {
            const Observation& trackObs = matches.at(i).first;
            const Observation& detecObs = matches.at(i).second;

            // Get the index of the track
            bool cont = true;
            for(unsigned int j=0; j<pfListSize && cont; j++)
            {
                if (pfList[j].isActivated() && pfList[j].getObservation() == trackObs )
                {
                    // Do the step for the particle filter.
                    std::vector<Observation> newDet;
                    newDet.push_back(detecObs);
                    pfList[j].step(frame, newDet);

                    // Update the kill list
                    killList[j]--;
                    if (killList[j]<0)
                        killList[j]=0;
                    cont = false;
                }
            }
        }

        // Manage death of unmatched tracks
        const unsigned int unmatchedTracksSize = unmatchedTracks.size();
        for(unsigned int i=0; i<unmatchedTracksSize; i++)
        {
            // Get the index of the track
            bool cont = true;
            for(unsigned int j=0; j<pfListSize && cont; j++)
            {
                if (pfList[j].isActivated() && pfList[j].getObservation() == unmatchedTracks[i])
                {
                    killList[j]++;
                    std::vector<Observation> tmp;
                    pfList[j].step(frame, tmp);
                    if (killList[j] >= trackKillingThresh)
                        pfList[j].activateTrack(false);
                    cont = false;
                }
            }
        }

        // Try to associate new detections with old ones.
        std::vector< Observation > oldDetecs;
        const unsigned int waitingListSize = waitingList.size();
        for (unsigned int i=0; i<waitingListSize; i++)
            oldDetecs.push_back(waitingList[i].first);

        associator.setThreshold(0.3);
        associator.run(oldDetecs, unmatchedDetecs);
        matches = associator.getMatches();
        unmatchedTracks = associator.getUnmatchedTracks();
        unmatchedDetecs = associator.getUnmatchedDetects();

        // Manage the creation of tracks
        std::vector<int> updatedDetecs;
        const unsigned int matchesSize1 = matches.size();
        for(unsigned int i=0; i<matchesSize1; i++)
        {
            const Observation& trackObs = matches.at(i).first;
            const Observation& detecObs = matches.at(i).second;

            // Get the index of the track
            bool cont = true;
            for(unsigned int j=0; j<waitingList.size() && cont; j++)
            {
                if (waitingList[j].first == trackObs)
                {
                    waitingList[j].second++;
                    killWaitingList[j]=0;
                    if (waitingList[j].second >= trackCreationThresh)   // Create a new track
                    {
                        ParticleFilter pf(frame);
                        pf.initParticles(frame, detecObs.getBoundingBox(), 4, true);
                        pf.setID(ID);
                        ID++;
                        pfList.push_back(pf);
                        killList.push_back(0);

                        waitingList.erase(waitingList.begin()+j);
                        killWaitingList.erase(killWaitingList.begin()+j);
                    } else  // Update the waiting list
                        waitingList[j].first.computeHistogram(detecObs.getFrame(), detecObs.getBoundingBox());
                    updatedDetecs.push_back((int)j);
                    cont = false;
                }
            }
        }

        // Manage the kill of old detecs
        for(unsigned int i=0; i<waitingList.size(); i++)
        {
            bool cont = true;
            const unsigned int updatedDetecsSize = updatedDetecs.size();
            for (unsigned int j=0; j<updatedDetecsSize && cont; j++)
            {
                if (i < (unsigned int)updatedDetecs[j])
                {
                    killWaitingList[j]++;
                    if (killWaitingList[j]>=detectionKillingThresh)
                    {
                        waitingList.erase(waitingList.begin()+j);
                        killWaitingList.erase(killWaitingList.begin()+j);
                    }
                    cont = false;
                }
            }
        }

        // Try to associate new detections with deactivated tracks.
        std::vector< Observation > deactiveTracks;
        const unsigned int pfListSize1 = pfList.size();
        for(unsigned int i=0; i<pfListSize1; i++)
            if (!pfList[i].isActivated())
                deactiveTracks.push_back(pfList[i].getObservation());

        associator.setThreshold(0.3);
        associator.run(deactiveTracks, unmatchedDetecs);
        matches = associator.getMatches();
        unmatchedDetecs = associator.getUnmatchedDetects();

        // Handle the reactivation of a track
        const unsigned int matchesSize2 = matches.size();
        for(unsigned int i=0; i<matchesSize2; i++)
        {
            const Observation& trackObs = matches.at(i).first;
            const Observation& detecObs = matches.at(i).second;

            // Get the index of the track
            bool cont = true;
            for(unsigned int j=0; j<pfListSize1 && cont; j++)
            {
                ParticleFilter& pf = pfList[j];
                if (!pf.isActivated() && pf.getObservation() == trackObs)
                {
                    pf.setReactivateCounter(pf.getReactivateCounter()+1);
                    if (pf.getReactivateCounter() == trackCreationThresh)
                    {
                        pf.setReactivateCounter(0);
                        pf.activateTrack(true);
                        std::vector<Observation> tmp;
                        tmp.push_back(detecObs);
                        pf.step(frame, tmp);
                    }
                    cont = false;
                }
            }
        }

        // Add the unassociated detecs to the waiting list
        const unsigned int unmatchedDetecsSize = unmatchedDetecs.size();
        for (unsigned int i=0; i<unmatchedDetecsSize; i++)
        {
            std::pair< Observation, int > p(unmatchedDetecs[i], 0);
            waitingList.push_back(p);
            killWaitingList.push_back(0);
        }
    }
}
