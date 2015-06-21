#include "tpsowrapper.h"


#include <RandomWalk.h>
#include <WaveFormObs.h>
#include <HeadShoulderObs.h>
#include <HeadShoulderState.h>

#include "ros/ros.h"

TpsoWrapper::TpsoWrapper()
    :is_tpso_init(false), is_ok(false), vad(0)
{   }


TpsoWrapper::~TpsoWrapper()
{
    if(this->is_tpso_init)
        this->endTpsoTracker();
}


bool TpsoWrapper::initTpsoTracker()
{
    if(this->is_tpso_init)  return true;

    this->dimension = 11;       this->nbParticles = 500;    this->observNoise = 0.05;


    this->setPtr = boost::shared_ptr<_Setting>(new _Setting()); //setting pointer
    std::vector<double> mint(this->dimension, -100.0);
    std::vector<double> maxt(this->dimension, 100.0);
    setPtr->init(mint, maxt);

    this->obsPtr = boost::shared_ptr< _Observation<Particle> >
            (new HeadShoulderObs<Particle>()); //observation pointer
    std::vector<int> discObsNoise;
    std::vector<double> contObsNoise(this->dimension, this->observNoise);
    obsPtr->setNoise(discObsNoise, contObsNoise);


    std::vector<double> processNoise(this->dimension);

    processNoise[0] = 30.;      processNoise[1] = 30.;      processNoise[2] = 30.;
    processNoise[3] = 0.08;     processNoise[4] = 0.08;     processNoise[5] = 0.08;
    processNoise[6] = 0.08;     processNoise[7] = 0.08;     processNoise[8] = 0.08;
    processNoise[9] = 0.08;     processNoise[10] = 0.08;


    this->dynmPtr = boost::shared_ptr< _Dynamics<Particle> >
            ( new RandomWalk<Particle>(processNoise) );   //dynamics pointer


    this->tpsoPtr = boost::shared_ptr< tpso::_TrackerPSO<Particle> >
            (new tpso::_TrackerPSO<Particle>()); //tracker point

    this->tpsoPtr->init(nbParticles, setPtr, obsPtr, dynmPtr);

    this->tpsoPtr->setParamW(0.9);
    this->tpsoPtr->setParamPhig(1.0);
    this->tpsoPtr->setParamPhip(0.8);

    this->contObs.resize(this->dimension, 0.);


    this->is_tpso_init = true;
    this->is_ok = true;

    this->new_obs_available = false;

    return this->is_tpso_init;
}


bool TpsoWrapper::stepTpsoTracker()
{
    if(!this->is_tpso_init)
        return false;

    this->map_modified.resize(7, 0.);
    this->mmse_modified.resize(7, 0.);

    /* Check if observation is available */
    if(this->new_obs_available)
    {
        this->new_obs_available = false;

        this->contObs[0] = 0;

        if(this->headShoulderPoseList.headPoseLists.size() > 0)
        {
            this->contObs[1] = this->headShoulderPoseList.headPoseLists[0].pose[0];/* x1 */
            this->contObs[2] = this->headShoulderPoseList.headPoseLists[0].pose[1];/* y1 */
            this->contObs[3] = this->headShoulderPoseList.headPoseLists[0].pose[2];/* z1 */

            this->contObs[7] = this->headShoulderPoseList.headPoseLists[0].pose[3];/* tetha */
            this->contObs[8] = this->headShoulderPoseList.headPoseLists[0].pose[4];/* phi */
            this->contObs[9] = this->headShoulderPoseList.headPoseLists[0].pose[5];/* psi */
        }

        if(this->headShoulderPoseList.shoulderPanLists.size() > 0)
            this->contObs[10] = this->headShoulderPoseList.shoulderPanLists[0];/* tetha_hs */

        if(this->headShoulderPoseList.headSkeletonLists.size() > 0)
        {
            //include coordinate coversion
            this->contObs[4] = -this->headShoulderPoseList.headSkeletonLists[0].pose[1]*1000.;/* x2 */
            this->contObs[5] = -this->headShoulderPoseList.headSkeletonLists[0].pose[2]*1000.;/* y2 */
            this->contObs[6] =  this->headShoulderPoseList.headSkeletonLists[0].pose[0]*1000.;/* z2 */
        }

        obsPtr->setObs(discObs, contObs);

        printf("With measurement - !\n");
        fflush(stdout);

    }
    else //no new observation available
    {
        std::vector<int> _discObs;
        std::vector<double> _contObs;
        _discObs.clear();   _contObs.clear();

        obsPtr->setObs(discObs, contObs);

        printf("No measurement - update only!\n");
        fflush(stdout);
    }

    /* Step the tracker */
    this->tpsoPtr->step();

    Particle & map = this->tpsoPtr->getMapEstimate();
    const std::vector<double> & vmap = map.getStateVector()->getContParams();

    Particle & mmse = this->tpsoPtr->getMmseEstimate();
    const std::vector<double> & vmmse = mmse.getStateVector()->getContParams();

    /*      data post processing to extract only the 7 states       */
    //vmap and vmmse (double vectors)
    this->map_modified[0]  = vmap[0];       this->map_modified[1]  = vmap[1];
    this->map_modified[2]  = vmap[2];
    this->mmse_modified[0] = vmmse[0];      this->mmse_modified[1] = vmmse[1];
    this->mmse_modified[2] = vmmse[2];

    double temp;
    temp = acos(vmap[3])*180.0/M_PI;    if (vmap[4] < 0)    temp = -temp;   this->map_modified[3] = temp;
    temp = acos(vmap[5])*180.0/M_PI;    if (vmap[6] < 0)    temp = -temp;   this->map_modified[4] = temp;
    temp = acos(vmap[7])*180.0/M_PI;    if (vmap[8] < 0)    temp = -temp;   this->map_modified[5] = temp;
    temp = acos(vmap[9])*180.0/M_PI;    if (vmap[10] < 0)   temp = -temp;   this->map_modified[6] = temp;

    temp = acos(vmmse[3])*180.0/M_PI;    if (vmmse[4] < 0)    temp = -temp;   this->mmse_modified[3] = temp;
    temp = acos(vmmse[5])*180.0/M_PI;    if (vmmse[6] < 0)    temp = -temp;   this->mmse_modified[4] = temp;
    temp = acos(vmmse[7])*180.0/M_PI;    if (vmmse[8] < 0)    temp = -temp;   this->mmse_modified[5] = temp;
    temp = acos(vmmse[9])*180.0/M_PI;    if (vmmse[10] < 0)   temp = -temp;   this->mmse_modified[6] = temp;

    this->trackedHsPoseList.headPoseLists.resize(1);
    this->trackedHsPoseList.headPoseLists[0].pose[0] = this->mmse_modified[0];
    this->trackedHsPoseList.headPoseLists[0].pose[1] = this->mmse_modified[1];
    this->trackedHsPoseList.headPoseLists[0].pose[2] = this->mmse_modified[2];
    this->trackedHsPoseList.headPoseLists[0].pose[3] = this->mmse_modified[3];
    this->trackedHsPoseList.headPoseLists[0].pose[4] = this->mmse_modified[4];
    this->trackedHsPoseList.headPoseLists[0].pose[5] = this->mmse_modified[5];

    this->trackedHsPoseList.shoulderPanLists.clear();
    this->trackedHsPoseList.shoulderPanLists.push_back(this->mmse_modified[6]);

    this->trackedHsPoseList.header.stamp = ros::Time::now();

    this->is_ok = true;
    //continue with deallocation and file closure
}




bool TpsoWrapper::endTpsoTracker()
{
    this->is_tpso_init = false;
    this->is_ok = false;
    this->vad = 0;
    //continue with other deallocation
}
