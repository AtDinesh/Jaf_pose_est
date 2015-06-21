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

#include <vector>
#include <queue>

#include "tpsowrapper.h"

#include <boost/random/uniform_real.hpp>
#include <boost/shared_ptr.hpp>

#include "ros/ros.h"
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>

#include "objectsMap.hpp"

#include "riddle/VAD.h"
#include "riddle/Intention.h"
#include "riddle/Utterance.h"   /* reco utterance message */
#include "riddle/ObjectPosition.h"

/* action related headers */
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
#include <riddle/EstimateIntentAction.h>

typedef actionlib::SimpleActionServer<riddle::EstimateIntentAction> IntentActionServer;


using namespace std;
using namespace tpso;

class IntentForInteraction {
public:
    IntentForInteraction();
    ~IntentForInteraction();

    bool init();
    bool fuse();
    virtual void execute();

    void reset()
    {

        this->poseList.headPoseLists.clear();
        this->poseList.shoulderPanLists.clear();
        this->poseList.headSkeletonLists.clear();

        this->intentBel[0] = 1.0;
        this->intentBel[1] = 0.;

        //ros::NodeHandle nh_t("~");
        _speachDetected = false;
        //_currIntDetected = false;
        this->head_pose_available = false;

        this->tpsoWrapper->minorReset();
    }

    /* message reading functions */
    void readVadTopic(const riddle::VAD & msg)
    {
        if (!msg.state.compare("start") || !msg.state.compare("process"))
            _speachDetected = true;
        else
            _speachDetected = false;

        this->tpsoWrapper->setVad(static_cast<int>(_speachDetected));
    }

    void headShoulderPoseReader(const riddle::HeadShoulderPoseList & msg)
    {   //
        //this->tpsoWrapper->setHeadShoulderPoseList(msg);
        this->poseList = msg;

        this->head_pose_available = true;
    }

    void readUtteranceTopic(const riddle::Utterance & msg);
    bool chooseNBest(const riddle::Utterance & msg);
    bool parseReco(std::string sentence, std::string& objFound, std::vector<double>& objPos, std::string& objLocationName);

protected:
    boost::shared_ptr<TpsoWrapper> tpsoWrapper;

    std::vector<double> Q;
    std::vector<double> vadLikelihood;
    std::vector<double> intentBel;

    /* ROS related variables */
    ros::NodeHandle _nh;
    ros::Subscriber _sub_hpose;
    ros::Subscriber _sub_vad;

    ros::Publisher _pub_filtered;
    ros::Publisher _pub_intent;
    ros::Publisher _pub_intent_conf;

    std::string hpose_topic;
    std::string vad_topic;
    std::string filtered_hpose_topic;
    std::string intent_topic;
    std::string intent_conf_topic;

    /* utterance and object related vars */
    bool _speachDetected;
    double _currIntention;
    double _currIntDetected;

    riddle::HeadShoulderPoseList poseList;
    bool head_pose_available;

    //Objects Map
    ObjectsMap* _objMap;

    ros::Subscriber _sub_utt;
    ros::Publisher _pub_object;

    ros::Publisher _pub_intentionnality;
    riddle::Intention _intention_msg;

    riddle::ObjectPosition _objPos_msg;
    std::string _utterance_topic_name;
    std::string _objmaps_file_name;

    //reco
    std::vector<double> _objPos;
    std::string _objName;
    //utterance list
    std::vector<std::string> _utterancesNBest;
    std::vector<int> _utterancesScores;
    int _currUttId;
    std::vector<double> _currPose;
    std::string _currObject;
    std::string _currObjectLocationName;
};

IntentForInteraction::IntentForInteraction()
    :hpose_topic("/head_shoulder_pose_list"), vad_topic("/vad"),
      filtered_hpose_topic("/filtered_hs_pose"),
      intent_topic("/discr_intent"), intent_conf_topic("/cont_intent"),
      _utterance_topic_name("utterance")
{
    this->_sub_hpose = this->_nh.subscribe(this->hpose_topic, 100,
                                           &IntentForInteraction::headShoulderPoseReader, this);
    this->_sub_vad = this->_nh.subscribe(this->vad_topic, 100,
                                         &IntentForInteraction::readVadTopic, this);

    this->_pub_filtered = this->_nh.advertise<riddle::HeadShoulderPoseList>
            (this->filtered_hpose_topic, 10, true);

    this->_pub_intent = this->_nh.advertise<std_msgs::Int8>
            (this->intent_topic, 10, true);

    this->_pub_intent_conf = this->_nh.advertise<std_msgs::Float32>
            (this->intent_conf_topic, 10, true);

    this->tpsoWrapper = boost::shared_ptr<TpsoWrapper>(new TpsoWrapper());

    /*
    _sub_utt = _nh.subscribe(_utterance_topic_name, 10,
                             &IntentForInteraction::readUtteranceTopic, this);
    */
    _pub_intentionnality = _nh.advertise<riddle::Intention>("intention", 10, true);

    this->head_pose_available = false;

    /*_pub_object = _nh.advertise<riddle::ObjectPosition>("ObjectPosition", 10, true);*/
}

IntentForInteraction::~IntentForInteraction()
{
    if (_objMap) delete _objMap;
}

bool IntentForInteraction::init()
{
    this->Q.resize(4, 0.);
    this->Q[0] = 0.99;     this->Q[2] = 0.017;
    this->Q[1] = 0.01;     this->Q[3] = 0.983;

    this->vadLikelihood.resize(4, 0.);
    /* xt = intent  */                          /* xt = Not intent  */
    this->vadLikelihood[0] = 0.3; /* zt = vad */
    this->vadLikelihood[2] = 0.75; /* zt = vad */
    this->vadLikelihood[1] = 0.7; /* zt = not vad */
    this->vadLikelihood[3] = 0.25;/* zt = not vad */

    this->intentBel.resize(2, 0.);
    this->intentBel[0] = 1.0;
    this->intentBel[1] = 0.;

    ros::NodeHandle nh_t("~");
    nh_t.param("objectmap_path", _objmaps_file_name, std::string("../data/objects/objectsMap"));
    _speachDetected = false;
    _currIntDetected = false;

    _objMap = new ObjectsMap(_objmaps_file_name);

    this->head_pose_available = false;

    return this->tpsoWrapper->initTpsoTracker();
}

void IntentForInteraction::readUtteranceTopic(const riddle::Utterance & msg)
{
    for (int i=0 ; i<msg.length ; i++)
    {
        ROS_INFO("Utterance[-] : %s", msg.utterances[i].c_str());
    }
    if(this->chooseNBest(msg))
    {
        ROS_INFO("Object found[-] : : %s", _currObject.c_str());
        _objPos_msg.header.stamp = ros::Time::now();
        _objPos_msg.objectName = _currObject;
        _objPos_msg.position[0] = _currPose[0];
        _objPos_msg.position[1] = _currPose[1];
        _objPos_msg.position[2] = _currPose[2];
        _objPos_msg.locationName = _currObjectLocationName;
        _objPos_msg.isThereObject = true;
        _pub_object.publish(_objPos_msg);
    }
    else
    {
        _objPos_msg.isThereObject = false;
    }
}

bool IntentForInteraction::chooseNBest(const riddle::Utterance& msg)
{
    std::map<std::string, int> vote;
    std::map<std::string, std::vector<double> > pos;
    std::map<std::string, std::string> location;

    if (_currUttId != msg.utt_id)
    {
        _currUttId = msg.utt_id;
        _utterancesNBest.clear();
        _utterancesScores.clear();
        vote.clear();
        pos.clear();
    }

    for (int i=0 ; i<msg.length ; i++)
    {
        vote[""]=0;
        std::string res, loc;
        std::vector<double> resPos;
        _utterancesNBest.push_back(msg.utterances[i]);
        _utterancesScores.push_back(msg.scores[i]);
        if (this->parseReco(msg.utterances[i], res, resPos, loc))
        {
            if (vote.count(res)==0)
            {
                vote[res] = 1;
                pos[res] = resPos;
                location[res] = loc;
            }
            else
                vote[res]+=1;
        }
        else
        {
            vote[""] += 1;
        }
    }

    std::string maxKey="";
    int maxValue = 0;
    std::multimap<std::string, int>::iterator it;

    for (it = vote.begin() ; it != vote.end() ; it++)
    {
        if (it->second > maxValue && it->first.compare(""))
        {
            maxValue = it->second;
            maxKey = it->first;
        }
    }

    if (!maxKey.compare("")) return false;

    _currPose = pos[maxKey];
    _currObject = maxKey;
    _currObjectLocationName = location[maxKey];

    return true;
}

bool IntentForInteraction::parseReco(std::string sentence, std::string& objFound, std::vector<double>& objPos, std::string& objLocationName)
{
    std::vector<std::string> words;
    size_t start=0, end=0;
    while (end != std::string::npos)
    {
        end = sentence.find(" ", start);
        words.push_back(sentence.substr(start, (end == std::string::npos) ? std::string::npos : end - start));
        start = (   ( end > (std::string::npos -1) )
                    ?  std::string::npos  :  end +1);
        if(_objMap->isInMap(words.back()))
        {
            ROS_INFO("word %s is in map !!!\n", words.back().c_str());
            objFound = words.back();
            objPos = _objMap->getObjectPosition(words.back());
            objLocationName = _objMap->getObjectLocationName(words.back());
            return true;
        }
    }

    objFound = "";
    objPos.clear();

    return false;
}

void IntentForInteraction::execute()
{
    std_msgs::Int8 discr_msg;
    std_msgs::Float32 cont_msg;

    ros::Rate loop_rate(5);

    while (ros::ok() &&
           !this->tpsoWrapper->newobs_available())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    this->tpsoWrapper->reset_obs();

    while (ros::ok() &&
           this->tpsoWrapper->ok())
    {
        this->tpsoWrapper->stepTpsoTracker();
        this->tpsoWrapper->reset_obs();

        if (!this->fuse())
            break;

        cont_msg.data = static_cast<float>(this->intentBel[0]);
        discr_msg.data = static_cast<short int>(this->intentBel[0]>=0.50);

        this->_pub_intent.publish(discr_msg);
        this->_pub_intent_conf.publish(cont_msg);
        this->_pub_filtered.publish(this->tpsoWrapper->getTrackedHsPoseList());

        this->_intention_msg.header.stamp = ros::Time::now();
        this->_intention_msg.intention = this->intentBel[0];
        this->_intention_msg.buffSize = 1; //hmm don't know what to set it
        this->_intention_msg.detected = static_cast<short int>(this->intentBel[0]>=0.50);

        _pub_intentionnality.publish(this->_intention_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

bool IntentForInteraction::fuse()
{
    const std::vector<double> & mmse =
            this->tpsoWrapper->get_mmse();

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


    if (this->tpsoWrapper->getVad() == 1) /* vad */
    {
        bel[0] = likelihood*this->vadLikelihood[0]*( this->Q[0]*this->intentBel[0]  +  this->Q[2]*this->intentBel[1]);
        bel[1] = unlikelihood*this->vadLikelihood[2]*( this->Q[3]*this->intentBel[0]  +  this->Q[4]*this->intentBel[1]);
    }
    else /* no vad */
    {
        bel[0] = likelihood*this->vadLikelihood[1]*( this->Q[0]*this->intentBel[0]  +  this->Q[2]*this->intentBel[1]);
        bel[1] = unlikelihood*this->vadLikelihood[3]*( this->Q[3]*this->intentBel[0]  +  this->Q[4]*this->intentBel[1]);
    }

    this->intentBel[0] = bel[0] /(bel[0] + bel[1]);
    this->intentBel[1] = bel[1] /(bel[0] + bel[1]);
    return true;
}

class Intent2InteractAction: public IntentForInteraction
{
public:
    Intent2InteractAction()
        : IntentForInteraction(),
          _intent_server(_nh,"estimate_intent",
                         boost::bind(&Intent2InteractAction::executeEstimateIntent, this, _1),
                         false),
          action_name_("estimate_intent")
    {

    }

    virtual void execute()
    {
        //do stuff
        this->_intent_server.start();
    }

    void executeEstimateIntent(const riddle::EstimateIntentGoalConstPtr &goal)
    {

        std::cout<<" Function executeEstimateIntent called..."<<std::endl;
        this->intentOccuranceCntr = 0;

        bool success = true;

        std_msgs::Int8 discr_msg;
        std_msgs::Float32 cont_msg;

        ros::Rate loop_rate(5);

        while (ros::ok() &&
               !this->tpsoWrapper->newobs_available())
        {
            this->tpsoWrapper->setHeadShoulderPoseList(this->poseList);

            ros::spinOnce();
            loop_rate.sleep();
        }

        this->tpsoWrapper->reset_obs();

        while (ros::ok() &&
               this->tpsoWrapper->ok())
        {
            this->tpsoWrapper->setHeadShoulderPoseList(this->poseList);

            this->tpsoWrapper->stepTpsoTracker();
            this->tpsoWrapper->reset_obs();

            if (!this->fuse())
                break;

            cont_msg.data = static_cast<float>(this->intentBel[0]);
            discr_msg.data = static_cast<short int>(this->intentBel[0]>=0.50);

            this->_pub_intent.publish(discr_msg);
            this->_pub_intent_conf.publish(cont_msg);
            this->_pub_filtered.publish(this->tpsoWrapper->getTrackedHsPoseList());

            this->_intention_msg.header.stamp = ros::Time::now();
            this->_intention_msg.intention = this->intentBel[0];
            this->_intention_msg.buffSize = 1; //hmm don't know what to set it

            int detected = static_cast<short int>(this->intentBel[0]>= goal->threshold);
            this->_intention_msg.detected = detected;
            _pub_intentionnality.publish(this->_intention_msg);

            if (detected)
                this->intentOccuranceCntr++;

            if (this->_intent_server.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                // set the action state to preempted
                _intent_server.setPreempted();
                success = false;
                break;
            }

            if (this->intentOccuranceCntr > goal->requiredCnt)
            {
                success = true;
                break;
            }


            this->poseList.headPoseLists.clear();
            this->poseList.shoulderPanLists.clear();
            this->poseList.headSkeletonLists.clear();


            ros::spinOnce();
            loop_rate.sleep();
        }

        //feedback_.currentCnt =
        if(success)
        {
            result_.confidence = this->intentBel[0];

            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            this->_intent_server.setSucceeded(result_);
        }

        this->reset();
    }

private:
    IntentActionServer _intent_server;

    riddle::EstimateIntentFeedback feedback_;
    riddle::EstimateIntentResult result_;

    std::string action_name_;
    int intentOccuranceCntr;
};



int main(int argc, char * argv[])
{
    ros::init(argc, argv, "multimodal_intent");

    //IntentForInteraction intentForInteraction;
    Intent2InteractAction intent;

    if (!intent.init())
    {
        std::cerr<<"Error initializing intentForInteraction...exiting!"<<std::endl;
        exit(-1);
    }

    //    /* goto into execution -- blocking code */
    std::cout<<"\n\nStarting actions server!!\n\n"<<std::endl;
    intent.execute();


    ros::spin();

    return 0;
}
