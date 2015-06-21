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
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>

#include <geometry_msgs/PoseStamped.h>

#include "riddle/VAD.h"
#include "riddle/Intention.h"


#include "riddle/HeadShoulderPoseList.h"


/* action related headers */
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
#include <riddle/EstimateIntentAction.h>

typedef actionlib::SimpleActionServer<riddle::EstimateIntentAction> IntentActionServer;

using namespace std;
using namespace tpso;

class IntentForInteraction {
public:
    IntentForInteraction(ros::NodeHandle * hndlNode, tf::TransformListener * hndlTransformListener);
    ~IntentForInteraction();

    bool init();
    bool fuse();
    virtual void execute();

    void reset()
    {

        this->poseList.headPoseLists.clear();
        this->poseList.shoulderPanLists.clear();
        this->poseList.headSkeletonLists.clear();

        this->intentBel[0] = 0.0;
        this->intentBel[1] = 1.0;

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


    void readHeadPoseTopic(const geometry_msgs::PoseStamped & msg)
    {
        printf("In readHeadPoseTopic... \n"); fflush(stdout);

        //this->head_pose_stamp = msg;

        riddle::HeadPose headSkeleton;
        tf::StampedTransform transform;

        this->poseList.headPoseLists.clear();

        {
            //printf(":");
            std::ostringstream oss;
            oss << "head_origin";
            std::string childFrame = oss.str();

            //oss1 << "/left_shoulder_" << i;       //oss2 << "/right_shoulder_" << i;
            //tf::Vector3 leftShoulder, rightShoulder;
            tfScalar yaw, pitch, roll;


            if (_listener->frameExists(childFrame))
            {
                printf("got childframe [%s] \n", childFrame.c_str());

                try{
                    _listener->lookupTransform("/camera_depth_frame", childFrame, ros::Time::now(), transform);
                    transform.getBasis().getEulerYPR(yaw, pitch, roll);

                    tf::Vector3 v = transform.getOrigin();
                    double act_yaw = yaw*180./M_PI;

                    if (act_yaw < 0.)
                        act_yaw += 180.;
                    else
                        act_yaw -= 180.;

                    headSkeleton.pose[0] = v.getX();    headSkeleton.pose[3] = act_yaw;
                    headSkeleton.pose[1] = v.getY();    headSkeleton.pose[4] = pitch*180./M_PI;
                    headSkeleton.pose[2] = v.getZ();    headSkeleton.pose[5] = roll*180./M_PI;

                    printf("position [%f %f %f] ",  headSkeleton.pose[0],
                            headSkeleton.pose[1],
                            headSkeleton.pose[2]);

                    printf("pos [%f %f %f] ", headSkeleton.pose[3],
                            headSkeleton.pose[4],
                            headSkeleton.pose[5]);

                    this->poseList.headPoseLists.push_back(headSkeleton);

                    //also if head position exists
                }
                catch(tf::TransformException& ex)
                {
                    std::cout << "Failure at "<< ros::Time::now() << std::endl;
                    std::cout << "Exception thrown:" << ex.what()<< std::endl;
                    //std::cout << "The current list of frames is:" <<std::endl;
                    //std::cout << _listener->allFramesAsString()<<std::endl;
                }
                //printf(":");
            }
        }

        this->head_pose_available = true;
        //        printf(" : [%lf %lf %lf] : \n", msg.pose.position.x*1000.,
        //               msg.pose.position.y*1000., msg.pose.position.z*1000.);
        //        fflush(stdout);
    }

    const riddle::HeadShoulderPoseList & getHeadShoulderFromTf();
    //void headShoulderPoseReader(const riddle::HeadShoulderPoseList & msg)
    //{   this->tpsoWrapper->setHeadShoulderPoseList(msg);        }

protected:
    boost::shared_ptr<TpsoWrapper> tpsoWrapper;

    std::vector<double> Q;
    std::vector<double> vadLikelihood;
    std::vector<double> intentBel;

    /* ROS related variables */
    ros::NodeHandle *_nh;
    //ros::Subscriber _sub_hpose;
    ros::Subscriber _sub_vad;

    ros::Publisher _pub_filtered;
    ros::Publisher _pub_intent;
    ros::Publisher _pub_intent_conf;

    bool _speachDetected;

    std::string vad_topic;
    std::string filtered_hpose_topic;
    std::string intent_topic;
    std::string intent_conf_topic;

    ros::Publisher _pub_intentionnality;
    riddle::Intention _intention_msg;

    tf::TransformListener *_listener;
    riddle::HeadShoulderPoseList poseList;

    std::string head_pose_topic;
    ros::Subscriber _sub_head_pose;
    geometry_msgs::PoseStamped head_pose_stamp;
    bool head_pose_available;
};

const riddle::HeadShoulderPoseList & IntentForInteraction::getHeadShoulderFromTf()
{
    //    riddle::HeadShoulderPoseList &dummy = this->poseList;

    // //        this->headShoulderPoseList.headPoseLists[0].pose[0];/* x1 */
    // //        this->headShoulderPoseList.headPoseLists[0].pose[1];/* y1 */
    // //        this->headShoulderPoseList.headPoseLists[0].pose[2];/* z1 */

    // //        this->headShoulderPoseList.headPoseLists[0].pose[3];/* tetha */
    // //        this->headShoulderPoseList.headPoseLists[0].pose[4];/* phi */
    // //        this->headShoulderPoseList.headPoseLists[0].pose[5];/* psi */

    // //        this->headShoulderPoseList.shoulderPanLists[0];/* tetha_hs */

    // //        //include coordinate coversion
    // //        -this->headShoulderPoseList.headSkeletonLists[0].pose[1]*1000.;/* x2 */
    // //        -this->headShoulderPoseList.headSkeletonLists[0].pose[2]*1000.;/* y2 */
    // //        this->headShoulderPoseList.headSkeletonLists[0].pose[0]*1000.;/* z2 */

    //    riddle::HeadPose headSkeleton;
    //    tf::StampedTransform transform;

    //    printf("In getHeadShoulderFromTf : ");

    //    dummy.shoulderPanLists.clear();
    //    dummy.headSkeletonLists.clear();

    //    for ( int i=0 ; i<4 ; i++) //limit to four
    //    {
    //        //printf(":");
    //        std::ostringstream oss, oss_h;
    //        oss << "neck_" << i;                    oss_h<<"head_" << i;
    //        std::string childFrame = oss.str();     std::string headFrame = oss_h.str();

    //        //oss1 << "/left_shoulder_" << i;       //oss2 << "/right_shoulder_" << i;
    //        //tf::Vector3 leftShoulder, rightShoulder;
    //        tfScalar yaw, pitch, roll;

    //        //printf(" %s [-%d-]  ", childFrame.c_str(), static_cast<int>(_listener->frameExists(childFrame)));

    //        if (_listener->frameExists(childFrame))
    //        {
    //            try{
    //                _listener->lookupTransform("/camera_depth_frame", childFrame, ros::Time(), transform);
    //                transform.getBasis().getEulerYPR(yaw, pitch, roll);

    //                yaw = (yaw*180/3.14) - 90;
    //                if (yaw < -180) yaw = 360 - yaw;

    //                dummy.shoulderPanLists.push_back(yaw);

    //                //also if head position exists
    //            }
    //            catch(tf::TransformException& ex)
    //            {
    //                std::cout << "Failure at "<< ros::Time::now() << std::endl;
    //                std::cout << "Exception thrown:" << ex.what()<< std::endl;
    //            }
    //            //printf(":");
    //        }

    //        if (_listener->frameExists(headFrame))
    //        {
    //            try{
    //                _listener->lookupTransform("/camera_depth_frame", headFrame, ros::Time(), transform);
    //                transform.getBasis().getEulerYPR(yaw, pitch, roll);

    //                tf::Vector3 v = transform.getOrigin();
    //                headSkeleton.pose[0] = v.getX();    headSkeleton.pose[3] = yaw;
    //                headSkeleton.pose[1] = v.getY();    headSkeleton.pose[4] = pitch;
    //                headSkeleton.pose[2] = v.getZ();    headSkeleton.pose[5] = roll;

    //                dummy.headSkeletonLists.push_back(headSkeleton);

    //                //also if head position exists
    //            }
    //            catch(tf::TransformException& ex)
    //            {
    //                std::cout << "Failure at "<< ros::Time::now() << std::endl;
    //                std::cout << "Exception thrown:" << ex.what()<< std::endl;
    //                //std::cout << "The current list of frames is:" <<std::endl;
    //                //std::cout << _listener->allFramesAsString()<<std::endl;
    //            }
    //            //printf(":");
    //        }

    //    }
    //    printf("\n");
    //    fflush(stdout);

    printf("Passing the following info: \n");

    for (int i = 0; i < static_cast<int>(this->poseList.headPoseLists.size()); i++)
    {
        printf("Head [%d] -- [%lf %lf %lf %lf %lf %lf] \n", i,
               this->poseList.headPoseLists[i].pose[0],
                this->poseList.headPoseLists[i].pose[1],
                this->poseList.headPoseLists[i].pose[2],
                this->poseList.headPoseLists[i].pose[3],
                this->poseList.headPoseLists[i].pose[4],
                this->poseList.headPoseLists[i].pose[5]);
    }


    for (int i = 0; i < static_cast<int>(this->poseList.shoulderPanLists.size()); i++)
        printf("Pan [%d] \n", i,this->poseList.shoulderPanLists[i]);

    for (int i = 0; i < static_cast<int>(this->poseList.headSkeletonLists.size()); i++)
    {
        printf("Skeleton [%d] -- [%lf %lf %lf %lf %lf %lf] \n", i,
               this->poseList.headSkeletonLists[i].pose[0],
                this->poseList.headSkeletonLists[i].pose[1],
                this->poseList.headSkeletonLists[i].pose[2],
                this->poseList.headSkeletonLists[i].pose[3],
                this->poseList.headSkeletonLists[i].pose[4],
                this->poseList.headSkeletonLists[i].pose[5]);
    }
    fflush(stdout);
    this->head_pose_available = false;

    return this->poseList;
}

IntentForInteraction::IntentForInteraction(ros::NodeHandle * hndlNode,
                                           tf::TransformListener *hndlTransformListener)
    : vad_topic("/vad"), filtered_hpose_topic("/filtered_hs_pose"),
      intent_topic("/discr_intent"), intent_conf_topic("/cont_intent"),
      head_pose_topic("/head_pose")
{
    this->_nh = hndlNode;
    this->_listener = hndlTransformListener;

    this->_sub_vad = this->_nh->subscribe(this->vad_topic, 5,
                                          &IntentForInteraction::readVadTopic, this);

    this->_sub_head_pose = this->_nh->subscribe(this->head_pose_topic, 1,
                                                &IntentForInteraction::readHeadPoseTopic, this);

    this->_pub_filtered = this->_nh->advertise<riddle::HeadShoulderPoseList>
            (this->filtered_hpose_topic, 10, true);

    this->_pub_intent = this->_nh->advertise<std_msgs::Int8>
            (this->intent_topic, 10, true);

    this->_pub_intent_conf = this->_nh->advertise<std_msgs::Float32>(this->intent_conf_topic, 10, true);

    this->tpsoWrapper = boost::shared_ptr<TpsoWrapper>(new TpsoWrapper());

    this->_pub_intentionnality = this->_nh->advertise<riddle::Intention>("intention", 10, true);

    this->head_pose_available = false;
}

IntentForInteraction::~IntentForInteraction()
{

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
    this->intentBel[0] = 0.7;
    this->intentBel[1] = 0.3;

    //ros::NodeHandle nh_t("~");
    _speachDetected = false;
    //_currIntDetected = false;

    this->head_pose_available = false;

    return this->tpsoWrapper->initTpsoTracker();

}



void IntentForInteraction::execute()
{
    std_msgs::Int8 discr_msg;
    std_msgs::Float32 cont_msg;

    ros::Rate loop_rate(5);

    while (ros::ok() &&
           !this->tpsoWrapper->newobs_available())
    {
        this->tpsoWrapper->setHeadShoulderPoseList(getHeadShoulderFromTf());
        ros::spinOnce();
        loop_rate.sleep();
    }

    this->tpsoWrapper->reset_obs();

    while (ros::ok() &&
           this->tpsoWrapper->ok())
    {
        //new! pass the head and shoulder frames
        this->tpsoWrapper->setHeadShoulderPoseList(//msg
                                                   getHeadShoulderFromTf()
                                                   );

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

        this->poseList.headPoseLists.clear();
        ros::spinOnce();
        loop_rate.sleep();
    }

    this->reset();
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
    Intent2InteractAction(ros::NodeHandle * hndlNode,
                          tf::TransformListener * hndlTransformListener)
        : IntentForInteraction(hndlNode, hndlTransformListener),
          /**_intent_server(hndlNode[0],"estimate_intent",
                            boost::bind(&Intent2InteractAction::executeEstimateIntent, this, _1),
                            false),**/
          action_name_("estimate_intent")
    {

    }

    virtual void execute()
    {
        //do stuff
        //this->_intent_server.start();
        riddle::EstimateIntentGoal goal;
        goal.requiredCnt = 2;
        goal.threshold = 0.5;
        //riddle::EstimateIntentGoalConstPtr
        executeEstimateIntent(goal);

    }

    //void executeEstimateIntent(const riddle::EstimateIntentGoalConstPtr &goal)
    void executeEstimateIntent(const riddle::EstimateIntentGoal &goal)
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
            this->tpsoWrapper->setHeadShoulderPoseList(getHeadShoulderFromTf());
            ros::spinOnce();
            loop_rate.sleep();
        }

        this->tpsoWrapper->reset_obs();

        int no_update_counter = 0;

        while (ros::ok() &&
               this->tpsoWrapper->ok())
        {

            //new! pass the head and shoulder frames
            this->tpsoWrapper->setHeadShoulderPoseList(//msg
                                                       getHeadShoulderFromTf()
                                                       );

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

            int detected = static_cast<short int>(this->intentBel[0]>= goal.threshold);
            this->_intention_msg.detected = detected;
            _pub_intentionnality.publish(this->_intention_msg);

            if (detected)
                this->intentOccuranceCntr++;
            /**
            if (this->_intent_server.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                // set the action state to preempted
                _intent_server.setPreempted();
                success = false;
                break;
            }
            **/
            /**
            if (this->intentOccuranceCntr > goal.requiredCnt)
            {
                success = true;
                break;
            }
            **/

            if (!this->tpsoWrapper->newobs_available())
                no_update_counter++;

            this->poseList.headPoseLists.clear();
            this->poseList.shoulderPanLists.clear();
            this->poseList.headSkeletonLists.clear();

            if (no_update_counter > 15)
            {
                printf(".....RESETING ESTIMATOR.....!");
                fflush(stdout);
                this->reset();
                no_update_counter = 0;

                this->intentOccuranceCntr = 0;

                ros::Rate loop_rate(5);

                while (ros::ok() &&
                       !this->tpsoWrapper->newobs_available())
                {
                    this->tpsoWrapper->setHeadShoulderPoseList(getHeadShoulderFromTf());

                    cont_msg.data = static_cast<float>(this->intentBel[0]);
                    discr_msg.data = static_cast<short int>(this->intentBel[0]>=0.50);

                    this->_pub_intent.publish(discr_msg);
                    this->_pub_intent_conf.publish(cont_msg);
                    this->_pub_filtered.publish(this->tpsoWrapper->getTrackedHsPoseList());

                    this->_intention_msg.header.stamp = ros::Time::now();
                    this->_intention_msg.intention = this->intentBel[0];
                    this->_intention_msg.buffSize = 1; //hmm don't know what to set it

                    int detected = static_cast<short int>(this->intentBel[0]>= goal.threshold);
                    this->_intention_msg.detected = detected;
                    _pub_intentionnality.publish(this->_intention_msg);


                    ros::spinOnce();
                    loop_rate.sleep();
                }
            }

            ros::spinOnce();
            loop_rate.sleep();
        }

        //feedback_.currentCnt =
        if(success)
        {
            result_.confidence = this->intentBel[0];

            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            /** this->_intent_server.setSucceeded(result_); **/
        }

        this->reset();
    }

private:
    /**IntentActionServer _intent_server;**/

    riddle::EstimateIntentFeedback feedback_;
    riddle::EstimateIntentResult result_;

    std::string action_name_;
    int intentOccuranceCntr;
};


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "multimodal_intent");

    ros::NodeHandle nh;
    tf::TransformListener listener;

    listener.waitForTransform("/camera_depth_frame",
                              "/camera_depth_optical_frame",
                              ros::Time(), ros::Duration(5.0));

    std::cout << "The current list of frames is:" <<std::endl;
    std::cout << listener.allFramesAsString()<<std::endl;

    std::cout<<"Constructing! intent4interaction "<<std::endl;

    //IntentForInteraction intentForInteraction(&nh, &listener);
    Intent2InteractAction intent(&nh, &listener);

    std::cout<<"...intent4interaction constructed! "<<std::endl;


    std::cout<<"intent4interaction initialization! "<<std::endl;

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
