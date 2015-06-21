/*
Notes:

Tracker
-------
Members
- Raw Sensor Data
- Detector
- Observation (detector + likelihood)
- Particle
    - State Vector

Methods
- SetData();
- Step(); or Process()

- Visualization tools()
- Data looging/saving options()

Particle
--------
Members
- bounds (static)
- State vector

State vector
-----------
- variable dimension storage

*/

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <ctime>

#include <boost/random/uniform_real.hpp>


#include <_state.h>
#include <_particle.h>
#include <_trackerpso.h>
#include <_observation.h>
#include <_dynamics.h>

#include <RandomWalk.h>
#include <WaveFormObs.h>
#include <HeadShoulderState.h>
#include <HeadShoulderObs.h>

using namespace std;



//#define NO_VAD

//#define NO_RGB_DATA

/*
function [newParticles] = dynamics(oldParticles, option)
nbParticles = size(oldParticles);
nbParticles = nbParticles(2);

newParticles = oldParticles + (option.ProcessNoise*ones(1,nbParticles)).*(randn(size(oldParticles)));
*/
using namespace tpso;

int setup_and_run_tpso_wavform()
{
    /*
     *      Setup (intialize + configure) and run TPSO tracker for a tracking sample wavform
     *      read from a text file. Write the result onto specified text files.
     */

    typedef tpso::_Particle<tpso::State2C> Particle;
    boost::mt19937 rng;

    unsigned int dimension = 4;
    unsigned int nbParticles = 500;

    /* setup Setting */
    boost::shared_ptr<_Setting> setPtr = boost::shared_ptr<_Setting>(new _Setting());
    std::vector<double> mint, maxt;
    mint.push_back(-10.0);  mint.push_back(-10.0);
    maxt.push_back(10.0);   maxt.push_back(10.0);

    setPtr->init(mint, maxt);


    /* setup ObservationModel */
    boost::shared_ptr< _Observation<Particle> > obsPtr = boost::shared_ptr< _Observation<Particle> >
            (new WaveformObs<Particle>());
    //noise needs to be set
    std::vector<int> discObsNoise;
    std::vector<double> contObsNoise(2, 0.05);

    obsPtr->setNoise(discObsNoise, contObsNoise);

    /* setup DynamicsModel  */

    double processNoise = 0.05; //the same for each state variable component

    boost::shared_ptr< _Dynamics<Particle> > dynmPtr = boost::shared_ptr< _Dynamics<Particle> >
            ( new RandomWalk<Particle>(processNoise, dimension) );

    /* initialize PsoTracker */
    tpso::_TrackerPSO<Particle> psoTracker;
    psoTracker.init(nbParticles, setPtr, obsPtr, dynmPtr);

    psoTracker.setParamW(0.9);
    psoTracker.setParamPhig(1.5);
    psoTracker.setParamPhip(0.8);


    std::vector<int> discObs;
    std::vector<double> contObs;

    std::string true_signal_filename = "/home/aamekonn/Documents/christophe/PSO/filtering/data/true_signal.txt";
    std::string obsv_signal_filename = "/home/aamekonn/Documents/christophe/PSO/filtering/data/observed_signal.txt";
    std::string pso_signal_filename = "/home/aamekonn/Documents/christophe/PSO/filtering/data/tracked_signal.txt";
    std::string pso_mmse_filename =  "/home/aamekonn/Documents/christophe/PSO/filtering/data/trmmse_signal.txt";

    ifstream true_signal_stream;
    ifstream obsv_signal_stream;
    ofstream psot_signal_stream;
    ofstream mmse_signal_stream;


    true_signal_stream.open(true_signal_filename.c_str(), ios::in);
    obsv_signal_stream.open(obsv_signal_filename.c_str(), ios::in);

    psot_signal_stream.open(pso_signal_filename.c_str(), ios::out);
    mmse_signal_stream.open(pso_mmse_filename.c_str(), ios::out);

    if(!obsv_signal_stream.is_open())
    {
        std::cerr<<"Error opening file : "<<obsv_signal_stream<<std::endl;
        std::cerr<<"Exiting..."<<std::endl;
        return -1;
    }

    contObs.resize(2, 0.);

    std::clock_t start;
    double duration;
    start = std::clock();

    unsigned int t = 0;

    do {
        //sample discObs
        //sample contObs
        obsv_signal_stream>>contObs[0]>>contObs[1];

        obsPtr->setObs(discObs, contObs);
        psoTracker.step();

        Particle & map = psoTracker.getMapEstimate();
        const std::vector<double> & vmap = map.getStateVector()->getContParams();

        Particle & mmse = psoTracker.getMmseEstimate();
        const std::vector<double> & vmmse = mmse.getStateVector()->getContParams();

        for (size_t i = 0; i < vmap.size(); i++)
            psot_signal_stream<<vmap[i]<<" ";
        psot_signal_stream<<std::endl;

        for (size_t i = 0; i < vmmse.size(); i++)
            mmse_signal_stream<<vmmse[i]<<" ";
        mmse_signal_stream<<std::endl;


        //std::cout<<t<<":["<<contObs[0]<<","<<contObs[1]<<"] ";
        std::cout<<t++<<" "<<std::flush;

    } while(obsv_signal_stream.good());

    psot_signal_stream.close();
    obsv_signal_stream.close();
    true_signal_stream.close();
    mmse_signal_stream.close();

    std::cout<<std::endl;

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

    std::cout<<" fps : ["<<1000./duration<<"] "<<std::endl;

    return 0;
}



int setup_and_run_tpso_hstracker()
{
    /*
     *      Setup (intialize + configure) and run TPSO tracker for a tracking head shoulder
     *      pose (x, y, z position and yaw, pitch, roll) with observation data
     *      read from a text file. The result is also written onto a text file.
     */
    //return  setup_and_run_tpso_wavform();

    typedef tpso::_Particle<HeadShoulderState> Particle;
    boost::mt19937 rng;

    unsigned int dimension = 11;
    unsigned int nbParticles = 500;
    double observNoise = 0.05;

    /*
     *      setup Setting
     */
    boost::shared_ptr<_Setting> setPtr = boost::shared_ptr<_Setting>(new _Setting());
    std::vector<double> mint(dimension, -100.0);
    std::vector<double> maxt(dimension, 100.0);
    setPtr->init(mint, maxt);


    /*
     *      setup ObservationModel
     *
     */
    boost::shared_ptr< _Observation<Particle> > obsPtr = boost::shared_ptr< _Observation<Particle> >
            (new HeadShoulderObs<Particle>());
    //noise needs to be set
    std::vector<int> discObsNoise;
    std::vector<double> contObsNoise(dimension, observNoise);
    obsPtr->setNoise(discObsNoise, contObsNoise);

    /*
     *      setup DynamicsModel
     */
    std::vector<double> processNoise(dimension);

    processNoise[0] = 30.;      processNoise[1] = 30.;      processNoise[2] = 30.;
    processNoise[3] = 0.08;     processNoise[4] = 0.08;     processNoise[5] = 0.08;
    processNoise[6] = 0.08;     processNoise[7] = 0.08;     processNoise[8] = 0.08;
    processNoise[9] = 0.08;     processNoise[10] = 0.08;

    boost::shared_ptr< _Dynamics<Particle> > dynmPtr = boost::shared_ptr< _Dynamics<Particle> >
            ( new RandomWalk<Particle>(processNoise) );

    /* initialize PsoTracker */
    tpso::_TrackerPSO<Particle> psoTracker;
    psoTracker.init(nbParticles, setPtr, obsPtr, dynmPtr);

    psoTracker.setParamW(0.9);
    psoTracker.setParamPhig(1.0);
    psoTracker.setParamPhip(0.8);


    std::vector<int> discObs;
    std::vector<double> contObs;

    /* input */
    std::string obsv_signal_filename = "/home/aamekonn/Documents/christophe/PSO/filtering/datahs/observed_signal_hs.txt";

    /* ouput */
    std::string pso_map_filename = "/home/aamekonn/Documents/christophe/PSO/filtering/datahs/trmap_signal_hs.txt";
    std::string pso_mmse_filename =  "/home/aamekonn/Documents/christophe/PSO/filtering/datahs/trmmse_signal_hs.txt";
    std::string postpso_map_filename = "/home/aamekonn/Documents/christophe/PSO/filtering/datahs/posttrmap_signal_hs.txt";
    std::string postpso_mmse_filename =  "/home/aamekonn/Documents/christophe/PSO/filtering/datahs/posttrmmse_signal_hs.txt";

    ifstream obsv_signal_stream;
    ofstream map_signal_stream;
    ofstream mmse_signal_stream;
    ofstream postmap_signal_stream;
    ofstream postmmse_signal_stream;


    obsv_signal_stream.open(obsv_signal_filename.c_str(), ios::in);

    map_signal_stream.open(pso_map_filename.c_str(), ios::out);
    mmse_signal_stream.open(pso_mmse_filename.c_str(), ios::out);

    postmap_signal_stream.open(postpso_map_filename.c_str(), ios::out);
    postmmse_signal_stream.open(postpso_mmse_filename.c_str(), ios::out);

    if(!obsv_signal_stream.is_open())
    {
        std::cerr<<"Error opening file : "<<obsv_signal_stream<<std::endl;
        std::cerr<<"Exiting..."<<std::endl;
        return -1;
    }

    contObs.resize(dimension, 0.);

    std::clock_t start;
    double duration;
    start = std::clock();

    std::vector<double> map_modified(7, 0.);
    std::vector<double> mmse_modified(7, 0.);

    unsigned int t = 0;

    do {
        //sample discObs
        //sample contObs
        for (unsigned int d = 0; d < dimension; d++)
            obsv_signal_stream>>contObs[d];

        obsPtr->setObs(discObs, contObs);
        psoTracker.step();

        Particle & map = psoTracker.getMapEstimate();
        const std::vector<double> & vmap = map.getStateVector()->getContParams();

        Particle & mmse = psoTracker.getMmseEstimate();
        const std::vector<double> & vmmse = mmse.getStateVector()->getContParams();

        for (size_t i = 0; i < vmap.size(); i++)
            map_signal_stream<<vmap[i]<<" ";
        map_signal_stream<<std::endl;

        for (size_t i = 0; i < vmmse.size(); i++)
            mmse_signal_stream<<vmmse[i]<<" ";
        mmse_signal_stream<<std::endl;


        /*      data post processing to extract only the 7 states       */
        //vmap and vmmse (double vectors)
        map_modified[0]  = vmap[0];     map_modified[1]  = vmap[1];     map_modified[2]  = vmap[2];
        mmse_modified[0] = vmmse[0];    mmse_modified[1] = vmmse[1];    mmse_modified[2] = vmmse[2];

        double temp;
        temp = acos(vmap[3])*180.0/M_PI;    if (vmap[4] < 0)    temp = -temp;   map_modified[3] = temp;
        temp = acos(vmap[5])*180.0/M_PI;    if (vmap[6] < 0)    temp = -temp;   map_modified[4] = temp;
        temp = acos(vmap[7])*180.0/M_PI;    if (vmap[8] < 0)    temp = -temp;   map_modified[5] = temp;
        temp = acos(vmap[9])*180.0/M_PI;    if (vmap[10] < 0)   temp = -temp;   map_modified[6] = temp;

        temp = acos(vmmse[3])*180.0/M_PI;    if (vmmse[4] < 0)    temp = -temp;   mmse_modified[3] = temp;
        temp = acos(vmmse[5])*180.0/M_PI;    if (vmmse[6] < 0)    temp = -temp;   mmse_modified[4] = temp;
        temp = acos(vmmse[7])*180.0/M_PI;    if (vmmse[8] < 0)    temp = -temp;   mmse_modified[5] = temp;
        temp = acos(vmmse[9])*180.0/M_PI;    if (vmmse[10] < 0)   temp = -temp;   mmse_modified[6] = temp;

        for (size_t i = 0; i < map_modified.size(); i++)
            postmap_signal_stream<<map_modified[i]<<" ";
        postmap_signal_stream<<std::endl;

        for (size_t i = 0; i < mmse_modified.size(); i++)
            postmmse_signal_stream<<mmse_modified[i]<<" ";
        postmmse_signal_stream<<std::endl;

        //std::cout<<t<<":["<<contObs[0]<<","<<contObs[1]<<"] ";
        std::cout<<t++<<" "<<std::flush;

    } while(obsv_signal_stream.good());

    map_signal_stream.close();
    obsv_signal_stream.close();
    mmse_signal_stream.close();
    postmap_signal_stream.close();
    postmmse_signal_stream.close();

    std::cout<<std::endl;

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

    std::cout<<" fps : ["<<1000./duration<<"] "<<std::endl;

    return 0;
}

#include "ros/ros.h"
#include "riddle/TpsoState.h"

#include "riddle/HeadShoulderPoseList.h"
#include "riddle/TimeFlag.h"
#include <std_msgs/Int8.h>

class TpsoRosWrapper
{
    typedef tpso::_Particle<HeadShoulderState> Particle;

public:
    TpsoRosWrapper();
    ~TpsoRosWrapper();

    void headShoulderPoseReader(const riddle::HeadShoulderPoseList & msg);
    bool initTpsoTracker();
    bool stepTpsoTracker();

    void readVadTopic(const std_msgs::Int8 & msg)
    {
        this->vad = msg.data;
    }

    bool endTpsoTracker();

    bool ok() { return this->is_ok; }

    const std::vector<double> & get_map() {
        return this->map_modified;
    }
    const std::vector<double> & get_mmse() {
        return this->mmse_modified;
    }

    const bool  is_new_obs_avail()  {   return this->new_obs_available;     }
    const bool  reset_obs()         {   this->new_obs_available = false;    }


    int vad;

protected:
    riddle::HeadShoulderPoseList headShoulderPoseList;
    riddle::HeadShoulderPoseList trackedHsPoseList;

    boost::shared_ptr<_Setting> setPtr; //setting pointer
    boost::shared_ptr< _Observation<Particle> > obsPtr; //observation pointer
    boost::shared_ptr< _Dynamics<Particle> > dynmPtr;   //dynamics pointer
    boost::shared_ptr< tpso::_TrackerPSO<Particle> > tpsoPtr; //tracker pointer

private:
    ros::NodeHandle _nh;
    ros::Subscriber _sub_hpose;
    ros::Subscriber _sub_vad;

    ros::Publisher _pub_filtered;
    std::string _head_shoulder_pose_topic;
    std::string _filtered_hs_pose_topic;
    std::string _vad_topic;


    boost::mt19937 rng;

    unsigned int dimension;
    unsigned int nbParticles;
    double observNoise;

    std::vector<int> discObs;
    std::vector<double> contObs;

    std::string obsv_signal_filename;
    std::string pso_map_filename;
    std::string pso_mmse_filename;
    std::string postpso_map_filename;
    std::string postpso_mmse_filename;

    ifstream obsv_signal_stream;
    ofstream map_signal_stream;
    ofstream mmse_signal_stream;
    ofstream postmap_signal_stream;
    ofstream postmmse_signal_stream;

    std::vector<double> map_modified;//(7, 0.);
    std::vector<double> mmse_modified;//(7, 0.);


    bool new_obs_available;
    bool obs_read;

    bool is_tpso_init;
    bool is_ok;
};

TpsoRosWrapper::TpsoRosWrapper()
    :_head_shoulder_pose_topic("/head_shoulder_pose_list"), _filtered_hs_pose_topic("/filtered_hs_pose"),
     _vad_topic("/vad_state"), is_tpso_init(false), is_ok(false)
{

    _sub_hpose = _nh.subscribe(_head_shoulder_pose_topic, 100,
                               &TpsoRosWrapper::headShoulderPoseReader, this);
    _pub_filtered = _nh.advertise<riddle::HeadShoulderPoseList>(_filtered_hs_pose_topic, 10, true);

    _sub_vad = _nh.subscribe(_vad_topic, 100,
                             &TpsoRosWrapper::readVadTopic, this);

    this->vad = 0;
}

TpsoRosWrapper::~TpsoRosWrapper()
{
    if(this->is_tpso_init)
        endTpsoTracker();
}

void TpsoRosWrapper::headShoulderPoseReader(const riddle::HeadShoulderPoseList & msg)
{
    /* msg  - std_msgs/Header header
     *      - riddle/HeadPose[] headPoseLists - pose[6] x y z (in mm) teta phi psi (in radian)
     *      - riddle/HeadPose[] headSkeletonLists - pose[6] x y z (in m) teta phi psi (in radian)
     *      - float64[] shoulderPanLists
     */
    //read poses and draw ontop of this

    this->headShoulderPoseList = msg;

    /* msg  - std_msgs/Header header
     *      - riddle/HeadPose[] headPoseLists - pose[6] x y z (in mm) teta phi psi (in radian)
     *      - riddle/HeadPose[] headSkeletonLists - pose[6] x y z (in m) teta phi psi (in radian)
     *      - float64[] shoulderPanLists
     */
    printf("Meast --> headpose[%d] headSkeleton[%d] shoulderPan[%d] ",
           static_cast<int>(msg.headPoseLists.size()),
           static_cast<int>(msg.headSkeletonLists.size()),
           static_cast<int>(msg.shoulderPanLists.size()));

    if((msg.headPoseLists.size() > 0) ||
            (msg.headSkeletonLists.size() > 0) ||
            (msg.shoulderPanLists.size() > 0)       )
        this->new_obs_available = true;
    else
    {
        this->new_obs_available = false;
    }
}

int annotation;

void setAnnotation(const riddle::TimeFlag & msg)
{
    annotation = msg.intFlag;
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "tpso_tracker");
    //ros::NodeHandle n;
    //ros::Publisher chatter_pub = n.advertise<tpso::TpsoState>("tpso_state", 1000);
    
    //ros::Subscriber _sub_annotation = n.subscribe("/vad/intent_annotation", 5, &setAnnotation);

    /* subscribe to */
    // riddle::HeadShoulderPoseList
    /* msg  - std_msgs/Header header
     *      - riddle/HeadPose[] headPoseLists - pose[6] x y z (in mm) teta phi psi (in radian)
     *      - riddle/HeadPose[] headSkeletonLists - pose[6] x y z (in m) teta phi psi (in radian)
     *      - float64[] shoulderPanLists
     */

    /* q11(0) q12(2)
     * q21(1) q22(3)
     */
    std::vector<double> Q(4, 0.);
    Q[0] = 0.99;     Q[2] = 0.017;
    Q[1] = 0.01;     Q[3] = 0.983;

    std::vector<double> vadLikelihood(4, 0.);
    /* xt = intent  */                          /* xt = Not intent  */
    vadLikelihood[0] = 0.3; /* zt = vad */         vadLikelihood[2] = 0.75; /* zt = vad */
    vadLikelihood[1] = 0.7; /* zt = not vad */     vadLikelihood[3] = 0.25;/* zt = not vad */

    //#ifdef NO_VAD
    //    vadLikelihood[0] = 1.0;
    //    vadLikelihood[1] = 1.0;
    //    vadLikelihood[2] = 1.0;
    //    vadLikelihood[3] = 1.0;
    //#endif
    std::vector<double> intentbel(2,0);
    /* prior distribution p(x0)*/
    intentbel[0] = 1.0;
    intentbel[1] = 0.0;

    annotation = 0;

    ros::Rate loop_rate(5);

    /* setup and launch the trial tracker */
    //int r = setup_and_run_tpso_hstracker();
    TpsoRosWrapper tpsoRosTracker;

    printf("Initializing wrapper!...");

    bool b = tpsoRosTracker.initTpsoTracker();

    printf("init complete [%d]...", static_cast<int>(b));
    fflush(stdout);


    riddle::TpsoState tpsoState;

    while(ros::ok() &&
          !tpsoRosTracker.is_new_obs_avail())
    {
        printf("No head and shoulder pos message!\n");
        fflush(stdout);
        ros::spinOnce();

        loop_rate.sleep();
    }

    tpsoRosTracker.reset_obs();

    int count = 0;
    ofstream likelihood_out;
    likelihood_out.open("/home/aamekonn/catkin_ws/tmp/icme_output/likelihood.txt", ios::out);

    bool vad_detected = true;

    while (ros::ok() &&
           tpsoRosTracker.ok())
    {
        tpsoRosTracker.stepTpsoTracker();
        tpsoRosTracker.reset_obs();

        const std::vector<double> & mmse =
                tpsoRosTracker.get_mmse();

        printf("(%04d)[ ", count);
        for (int i = 0; i < static_cast<int>(mmse.size()); i++)
            printf(" %.3f ", mmse[i]);
        printf("] \n");
        fflush(stdout);


        {
            /*
             *      Proceed with FUSION here
             */
            double dth, dphi,dhs, eqdist;
            double likelihood, unlikelihood;
            std::vector<double> bel(2,0);

            //standard deviation of 10
            double norm_fact = (2.0*M_PI*sqrt(2*M_PI)*10.*10.*10.);
            dth  = -0.5*(mmse[3]*mmse[3])/100.;
            dphi = -0.5*(mmse[4]*mmse[4])/100.;
            dhs  = -0.5*(mmse[6]*mmse[6])/100.;

            likelihood = exp(dth)*exp(dphi)*exp(dhs)/(norm_fact);

            dth  = (mmse[3]*mmse[3]);
            dphi = (mmse[4]*mmse[4]);
            dhs  = (mmse[6]*mmse[6]);

            eqdist = sqrt(dth + dphi + dhs);

            if (eqdist < 30.)
                unlikelihood = 10.*std::numeric_limits<float>::epsilon();
            else
                unlikelihood = 1.0/(M_PI*(eqdist*eqdist - 30.*30.));


            //#ifdef NO_RGB_DATA
            //            likelihood = 1.;
            //            unlikelihood = 1.;
            //#endif

            if (tpsoRosTracker.vad == 1) /* vad */
            {
                bel[0] = likelihood*vadLikelihood[0]*( Q[0]*intentbel[0]  +  Q[2]*intentbel[1]);
                bel[1] = unlikelihood*vadLikelihood[2]*( Q[3]*intentbel[0]  +  Q[4]*intentbel[1]);
            }
            else /* no vad */
            {
                bel[0] = likelihood*vadLikelihood[1]*( Q[0]*intentbel[0]  +  Q[2]*intentbel[1]);
                bel[1] = unlikelihood*vadLikelihood[3]*( Q[3]*intentbel[0]  +  Q[4]*intentbel[1]);
            }

            intentbel[0] = bel[0] /(bel[0] + bel[1]);
            intentbel[1] = bel[1] /(bel[0] + bel[1]);

            likelihood_out<<likelihood<<" "<<unlikelihood<<" "<<intentbel[0]<<" "<<intentbel[1]<<" "<<annotation<<" "<<tpsoRosTracker.vad<<std::endl;

            printf("[ intent %.5f  nointent %.5f vad %d ] ", intentbel[0], intentbel[1], tpsoRosTracker.vad);
            fflush(stdout);
            //#ifdef NO_RGB_DATA
            //            if(vad_detected)
            //            {
            //                vad_detected = false;
            //                getchar();
            //            }
            //#endif
        }


        /* publish demo data */
        //        tpsoState.header.stamp = ros::Time::now();
        //        tpsoState.mmse[0] = 0.;	tpsoState.cov[0] = 0.;
        //        tpsoState.mmse[1] = 0.;	tpsoState.cov[1] = 0.;
        //        tpsoState.mmse[2] = 0.;	tpsoState.cov[2] = 0.;
        //        tpsoState.mmse[3] = 0.;	tpsoState.cov[3] = 0.;
        //        tpsoState.mmse[4] = 0.;	tpsoState.cov[4] = 0.;
        //        tpsoState.mmse[5] = 0.;	tpsoState.cov[5] = 0.;
        //        tpsoState.mmse[6] = 0.;	tpsoState.cov[6] = 0.;

        //        chatter_pub.publish(tpsoState);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    likelihood_out.close();

    return 0;
    //return r;
}
bool TpsoRosWrapper::endTpsoTracker()
{
    this->is_tpso_init = false;

    if(map_signal_stream.is_open())
        map_signal_stream.close();

    if(obsv_signal_stream.is_open())
        obsv_signal_stream.close();

    if(mmse_signal_stream.is_open())
        mmse_signal_stream.close();

    if(postmap_signal_stream.is_open())
        postmap_signal_stream.close();

    if(postmmse_signal_stream.is_open())
        postmmse_signal_stream.close();

    this->is_ok = false;
    //continue with other deallocation
}

bool TpsoRosWrapper::stepTpsoTracker()
{
    if(!this->is_tpso_init)
        return false;

    std::clock_t start;
    double duration;
    start = std::clock();

    map_modified.resize(7, 0.);
    mmse_modified.resize(7, 0.);
    //std::vector<double> mmse_modified(7, 0.);

    //unsigned int t = 0;

    //do {
    //sample discObs
    //sample contObs

    /* Check if observation is available
     *
     */
    if(this->new_obs_available)
    {
        this->new_obs_available = false;
        //use headShoulderPoseList
        //for (unsigned int d = 0; d < dimension; d++)
        //    obsv_signal_stream>>contObs[d];

        // riddle::HeadShoulderPoseList
        /* msg  - std_msgs/Header header
         *      - riddle/HeadPose[] headPoseLists - pose[6] x y z (in mm) teta phi psi (in radian)
         *      - riddle/HeadPose[] headSkeletonLists - pose[6] x y z (in m) teta phi psi (in radian)
         *      - float64[] shoulderPanLists
         */
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
    tpsoPtr->step();

    Particle & map = tpsoPtr->getMapEstimate();
    const std::vector<double> & vmap = map.getStateVector()->getContParams();

    Particle & mmse = tpsoPtr->getMmseEstimate();
    const std::vector<double> & vmmse = mmse.getStateVector()->getContParams();

    for (size_t i = 0; i < vmap.size(); i++)
        map_signal_stream<<vmap[i]<<" ";
    map_signal_stream<<std::endl;

    for (size_t i = 0; i < vmmse.size(); i++)
        mmse_signal_stream<<vmmse[i]<<" ";
    mmse_signal_stream<<std::endl;


    /*      data post processing to extract only the 7 states       */
    //vmap and vmmse (double vectors)
    map_modified[0]  = vmap[0];     map_modified[1]  = vmap[1];     map_modified[2]  = vmap[2];
    mmse_modified[0] = vmmse[0];    mmse_modified[1] = vmmse[1];    mmse_modified[2] = vmmse[2];

    double temp;
    temp = acos(vmap[3])*180.0/M_PI;    if (vmap[4] < 0)    temp = -temp;   map_modified[3] = temp;
    temp = acos(vmap[5])*180.0/M_PI;    if (vmap[6] < 0)    temp = -temp;   map_modified[4] = temp;
    temp = acos(vmap[7])*180.0/M_PI;    if (vmap[8] < 0)    temp = -temp;   map_modified[5] = temp;
    temp = acos(vmap[9])*180.0/M_PI;    if (vmap[10] < 0)   temp = -temp;   map_modified[6] = temp;

    temp = acos(vmmse[3])*180.0/M_PI;    if (vmmse[4] < 0)    temp = -temp;   mmse_modified[3] = temp;
    temp = acos(vmmse[5])*180.0/M_PI;    if (vmmse[6] < 0)    temp = -temp;   mmse_modified[4] = temp;
    temp = acos(vmmse[7])*180.0/M_PI;    if (vmmse[8] < 0)    temp = -temp;   mmse_modified[5] = temp;
    temp = acos(vmmse[9])*180.0/M_PI;    if (vmmse[10] < 0)   temp = -temp;   mmse_modified[6] = temp;

    /*
    for (size_t i = 0; i < map_modified.size(); i++)
        postmap_signal_stream<<map_modified[i]<<" ";
    postmap_signal_stream<<std::endl;

    for (size_t i = 0; i < mmse_modified.size(); i++)
        postmmse_signal_stream<<mmse_modified[i]<<" ";
    postmmse_signal_stream<<std::endl;
    */
    this->trackedHsPoseList.headPoseLists.resize(1);
    this->trackedHsPoseList.headPoseLists[0].pose[0] = mmse_modified[0];
    this->trackedHsPoseList.headPoseLists[0].pose[1] = mmse_modified[1];
    this->trackedHsPoseList.headPoseLists[0].pose[2] = mmse_modified[2];
    this->trackedHsPoseList.headPoseLists[0].pose[3] = mmse_modified[3];
    this->trackedHsPoseList.headPoseLists[0].pose[4] = mmse_modified[4];
    this->trackedHsPoseList.headPoseLists[0].pose[5] = mmse_modified[5];

    this->trackedHsPoseList.shoulderPanLists.clear();
    this->trackedHsPoseList.shoulderPanLists.push_back(mmse_modified[6]);

    this->_pub_filtered.publish(this->trackedHsPoseList);



    //std::cout<<t<<":["<<contObs[0]<<","<<contObs[1]<<"] ";
    //std::cout<<t++<<" "<<std::flush;
    this->is_ok = true;


    /*
    if(!this->obsv_signal_stream.good())
    {
        map_signal_stream.close();
        obsv_signal_stream.close();
        mmse_signal_stream.close();
        postmap_signal_stream.close();
        postmmse_signal_stream.close();

        this->is_ok = false;
    }
    */

    //continue with deallocation and file closure
}

bool TpsoRosWrapper::initTpsoTracker()
{
    if(this->is_tpso_init)
        return true;

    this->dimension = 11;
    this->nbParticles = 500;
    this->observNoise = 0.05;


    this->setPtr = boost::shared_ptr<_Setting>(new _Setting()); //setting pointer
    std::vector<double> mint(dimension, -100.0);
    std::vector<double> maxt(dimension, 100.0);
    setPtr->init(mint, maxt);

    this->obsPtr = boost::shared_ptr< _Observation<Particle> >
            (new HeadShoulderObs<Particle>()); //observation pointer
    std::vector<int> discObsNoise;
    std::vector<double> contObsNoise(dimension, observNoise);
    obsPtr->setNoise(discObsNoise, contObsNoise);


    std::vector<double> processNoise(dimension);

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
    /*
    this->obsv_signal_filename = "/home/aamekonn/Documents/christophe/PSO/filtering/datahs/observed_signal_hs.txt";


    this->pso_map_filename = "/home/aamekonn/Documents/christophe/PSO/filtering/datahs/trmap_signal_hs.txt";
    this->pso_mmse_filename =  "/home/aamekonn/Documents/christophe/PSO/filtering/datahs/trmmse_signal_hs.txt";
    this->postpso_map_filename = "/home/aamekonn/Documents/christophe/PSO/filtering/datahs/posttrmap_signal_hs.txt";
    this->postpso_mmse_filename =  "/home/aamekonn/Documents/christophe/PSO/filtering/datahs/posttrmmse_signal_hs.txt";

    this->obsv_signal_stream.open(this->obsv_signal_filename.c_str(), ios::in);

    this->map_signal_stream.open(this->pso_map_filename.c_str(), ios::out);
    this->mmse_signal_stream.open(this->pso_mmse_filename.c_str(), ios::out);

    this->postmap_signal_stream.open(this->postpso_map_filename.c_str(), ios::out);
    this->postmmse_signal_stream.open(this->postpso_mmse_filename.c_str(), ios::out);

    if(!this->obsv_signal_stream.is_open())
    {
        std::cerr<<"Error opening file : "<<this->obsv_signal_stream<<std::endl;
        std::cerr<<"Exiting..."<<std::endl;
        return -1;
    }
    */

    this->contObs.resize(dimension, 0.);


    this->is_tpso_init = true;
    this->is_ok = true;

    this->new_obs_available = false;

    return this->is_tpso_init;
}

//namespace tpso {
//template <class P>
//class WaveformTPSO : public _TrackerPSO<P>
//{
//public:
//    WaveformTPSO():_TrackerPSO<P>() { }

//    virtual
//    void init(unsigned int nbrP,
//              SettingPtr & _setPtr,
//              tpso::_TrackerPSO<P>::ObservPtr & _obsPtr,
//              tpso::_TrackerPSO<P>::DynamicsPtr & _dynamics)
//    {
//        this->nbParticles = nbrP;

//        this->setting   =   _setPtr;
//        this->observ    =   _obsPtr;
//        this->dynmxPtr  =   _dynamics;

//        this->w = 0.;
//        this->phig = 0.;
//        this->phip = 0.;

//        this->sg = boost::shared_ptr<P>(new P());
//        this->sg_t_1 = boost::shared_ptr<P>(new P());

//        this->sx.clear();
//        this->sp.clear();
//        this->sv.clear();

//        this->sx_t_1.clear();
//        this->sp_t_1.clear();
//        this->sv_t_1.clear();

//        //initialize particles
//        for (unsigned int i = 0; i < this->nbParticles; i++)
//        {
//            this->sx.push_back(boost::shared_ptr<P>(new P(i)));
//            this->sp.push_back(boost::shared_ptr<P>(new P(i)));
//            this->sv.push_back(boost::shared_ptr<P>(new P(i)));

//            this->sx_t_1.push_back(boost::shared_ptr<P>(new P(i)));
//            this->sp_t_1.push_back(boost::shared_ptr<P>(new P(i)));
//            this->sv_t_1.push_back(boost::shared_ptr<P>(new P(i)));
//        }

//        std::cout<<"TPSO Tracker initialized!"<<std::endl;
//        this->initialized =  true;
//    }
//};
//}

/*
std::vector<double> v;
v.resize(2);
v[0] = 1;
v[1] = 2;

tpso::State2C state2C;
state2C.setContParams(v);

tpso::State2C s2, s3;

s2 = state2C;

std::cout<<"\n state2C : ";
state2C.printInfo();
std::cout<<"\n      s2 : ";
s2.printInfo();

s3 = (state2C + s2)*0.2555;
tpso::State2C s4(s3);
s4 = s4 - s3;
std::cout<<"\n      s3 : ";
s3.printInfo();
std::cout<<"\n      s4 : ";
s4.printInfo();

std::cout<<std::endl;

Particle particle, particle2;

particle.setStateVector(state2C);
particle.setNo(0);
particle.setWeight(0.5);
particle.printInfo();

particle2.setStateVector(s3);
particle2.setNo(1);
particle2.setWeight(0.25);
particle2.printInfo();

std::cout<<" particle1 * 0.3 "<<std::endl;
particle *= 0.3;
particle.printInfo();

std::cout<<" p3 = p1 + p2 "<<std::endl;
Particle p3 = particle + particle2;

p3.printInfo();


std::cout<<" p3 = p1 - p2 "<<std::endl;
Particle p4 = particle - particle2;

p4.printInfo();

boost::shared_ptr<tpso::State2C> & s = particle.getMutStateVector();
std::vector<double> & x = s->getMutContParams();
x[0] = 5;

//    particle.printInfo();

//    particle /= 5.0;

//    particle.printInfo();

std::vector<double> vNoise;
vNoise.resize(2, 0.05);
RandomWalk<Particle> rwalk(vNoise);
rwalk.setRndGenerator(rng);

rwalk.apply(particle2);
std::cout<<"\n pert1 p2 : ";
particle2.printInfo();

rwalk.apply(particle2);
std::cout<<"\n pert2 p2 : ";
particle2.printInfo();

rwalk.apply(particle2);
std::cout<<"\n pert2 p2 : ";
particle2.printInfo();
*/

/*
 *  Initialization
 *      Setting->initialize()
 *      Observation->initialize() -- no need
 *      Dynamics->initialize() -- set randomiser (for the moment)
 *
 *      PsoTracker->init(...)   //no. particles
 *      PsoTracker->setSetting(...)
 *      PsoTracker->setObservationModel(...)
 *      PsoTracker->setDynamicsModel(...)
 *      PsoTracker->setParam{W,phig,phip}(...)
 *
 *
 *  Tracking Routine
 *
 *  (1) - read current time observation (from sensor)
 *  (2) Observation->setObs(discrete, continous)
 *  (3) PsoTracker->step()
 *  (4) PsoTracker->{getMmseEstimate(), getMapEstimate()}
 *
 */


//    boost::shared_ptr< boost::uniform_real<> > ptrUniDist;

//    ptrUniDist =    boost::shared_ptr<  boost::uniform_real<>   >
//            (new boost::uniform_real<>(0,1));

//    //boost::uniform_real<> uni_dist(0,1);

//    for(int i = 0; i < 10; i++)
//    {
//        std::cout<<ptrUniDist->operator ()(rng)<<" ";
//    }



//state2C.printInfo();    std::cout<<std::endl;


//tpso::_TrackerPSO<Particle> PsoTracker;
