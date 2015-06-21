#ifndef TPSO_WRAPPER_H
#define TPSO_WRAPPER_H

#include <boost/shared_ptr.hpp>

#include <_state.h>
#include <_particle.h>
#include <_trackerpso.h>
#include <_observation.h>
#include <_dynamics.h>

#include "riddle/TpsoState.h"
#include "riddle/HeadShoulderPoseList.h"

#include "HeadShoulderState.h"

class TpsoWrapper
{
    typedef tpso::_Particle<HeadShoulderState> Particle;

public:
    TpsoWrapper();
    ~TpsoWrapper();

    //void headShoulderPoseReader(const riddle::HeadShoulderPoseList & msg);
    bool initTpsoTracker();
    bool stepTpsoTracker();
    bool endTpsoTracker();

    void minorReset()
    {
        if(!this->is_tpso_init)
            return;

        this->tpsoPtr->disperse();
        this->new_obs_available = false;
    }

    //bool resetTpsoTracker();

    //void readVadTopic(const std_msgs::Int8 & msg)
    //{
    //    this->vad = msg.data;
    //}
    bool ok() { return this->is_ok; }
    int getVad()            {   return this->vad;   }
    void setVad(const int data)   {   this->vad = data;   }

    const std::vector<double> & get_map() {    return this->map_modified;   }
    const std::vector<double> & get_mmse() {    return this->mmse_modified; }

    const bool  newobs_available()  {   return this->new_obs_available;     }
    const bool  reset_obs()         {   this->new_obs_available = false;    }

    const riddle::HeadShoulderPoseList & getTrackedHsPoseList() {   return this->trackedHsPoseList; }
    void setHeadShoulderPoseList(const riddle::HeadShoulderPoseList & hsPoseList)
    {
        this->headShoulderPoseList = hsPoseList;

        if ((hsPoseList.headPoseLists.size() > 0) ||
                (hsPoseList.headSkeletonLists.size() > 0) ||
                (hsPoseList.shoulderPanLists.size() > 0)    )
        {
            /*printf("[%lf %lf %lf %lf %lf %lf] \n",
                   hsPoseList.headPoseLists[0].pose[0],
                   hsPoseList.headPoseLists[0].pose[1],
                    hsPoseList.headPoseLists[0].pose[2],
                    hsPoseList.headPoseLists[0].pose[3],
                    hsPoseList.headPoseLists[0].pose[4],
                     hsPoseList.headPoseLists[0].pose[5]);
            fflush(stdout);
            */
            this->new_obs_available = true;
        }
        else
            this->new_obs_available = false;
    }

protected:
    riddle::HeadShoulderPoseList headShoulderPoseList;
    riddle::HeadShoulderPoseList trackedHsPoseList;

    boost::shared_ptr<_Setting> setPtr; //setting pointer
    boost::shared_ptr< _Observation<Particle> > obsPtr; //observation pointer
    boost::shared_ptr< _Dynamics<Particle> > dynmPtr;   //dynamics pointer
    boost::shared_ptr< tpso::_TrackerPSO<Particle> > tpsoPtr; //tracker pointer

private:
    /*
    ros::NodeHandle _nh;
    ros::Subscriber _sub_hpose;
    ros::Subscriber _sub_vad;

    ros::Publisher _pub_filtered;
    std::string _head_shoulder_pose_topic;
    std::string _filtered_hs_pose_topic;
*/

    boost::mt19937 rng;

    unsigned int dimension;
    unsigned int nbParticles;
    double observNoise;

    std::vector<int> discObs;
    std::vector<double> contObs;

    std::vector<double> map_modified;
    std::vector<double> mmse_modified;

    bool new_obs_available;
    bool obs_read;

    bool is_tpso_init;
    bool is_ok;

    int vad;
};

#endif
