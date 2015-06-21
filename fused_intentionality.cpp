//#define uint64 uint64_sphinx
//#define int64 int64_sphinx
//#include <audioProcessing.h>
//#undef uint64
//#undef int64
#include <ros/ros.h>

#include <string>
#include <vector>
#include <queue>
#include "head-pose-fanelli/CRForestEstimator.h"
#include "head-pose-fanelli/gl_camera.hpp"
#include "head-pose-fanelli/freeglut.h"

//#include "kinect-interface/kinectInterface.h"

#include "objectsMap.hpp"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "image_transport/image_transport.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

//message topic headers to subscribe
#include "riddle/HeadPoseList.h"
#include "riddle/HeadShoulderPoseList.h"  /* fanelli head pose list */
#include "riddle/VAD.h" /* vad message */
#include "riddle/Utterance.h"   /* reco utterance message */

//message topic headers to publish
#include "riddle/Intention.h"
#include "riddle/ObjectPosition.h"


//TODO :
// ajouter les fonctions publish (utterances et intention).
// ajouter les params serveurs (taille buffers en secondes)
// pb avec stamp.secs (non reconnu)


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

namespace riddle {

class Fusion
{
	public:
		Fusion(std::string ONIfile="");
		~Fusion();

		void init(int argc, char* argv[]);

		void readUtteranceTopic(const riddle::Utterance & msg);
		void readVadTopic(const riddle::VAD & msg);
		void readHeadShoulderPoseTopic(const riddle::HeadShoulderPoseList & msg);
		void readHeadPoseTopic(const riddle::HeadPoseList & msg);
		//void depthMapReader(const sensor_msgs::PointCloud2ConstPtr& msg);

	protected:

		void updateInt(double value, long long int timestamp, double weight=1);
		
		bool chooseNBest(const riddle::Utterance & msg);
		bool parseReco(std::string sentence, std::string& objFound, std::vector<double>& objPos, std::string& objLocationName);

	protected:
		//FanelliInt* _pFanelliDet;

		//shoulders
		std::vector<math_vector_3f> _leftShoulder, _rightShoulder;
		std::vector<double> _shoulderPan;
		//KinectStruct* _kinectData;

		int _frame;

		//security
		bool _isKinectRunning;

		//intentionnality
		double _lastInt;
		std::vector<double> _previousInt;
		int _intIndex;

		//Objects Map
		ObjectsMap* _objMap;

	private:
		//ros stuff
		ros::NodeHandle _nh;
		ros::Subscriber _sub_utt;
		ros::Subscriber _sub_vad;
		ros::Subscriber _sub_hspose;
		ros::Subscriber _sub_hpose;
		ros::Publisher _pub_object;
		ros::Publisher _pub_intentionnality;
		riddle::Intention _intention_msg;
		riddle::ObjectPosition _objPos_msg;

		std::string _utterance_topic_name;
		std::string _vad_topic_name;
		std::string _head_pose_topic_name;
		std::string _head_shoulder_pose_topic_name;
		std::string _depth_msg_name;

		std::string _objmaps_file_name;
		//end ros stuff
		
		//Fannelli head pose
		//this vector can be directly populated from topic message
		std::vector< cv::Vec<float,POSE_SIZE> > _vFanelliResults;
		//VAD
		bool _speachDetected;
		
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
		
		//Intention
		std::queue<double> _features;
		std::queue<double> _weights;
		std::queue<long long int> _timestamp;
		double _currIntention;
		double _currIntDetected;
};

Fusion* g_fusionInstance;
}

using namespace riddle;

Fusion::Fusion(std::string ONIfile)
    :_head_pose_topic_name("head_pose_list"), _utterance_topic_name("utterance"),
      _vad_topic_name("vad"), _head_shoulder_pose_topic_name("head_shoulder_pose_list")
{
    g_fusionInstance = this;

    /* parse this from parameter server */
    

    //subscribe to messages
    _sub_hspose = _nh.subscribe(_head_shoulder_pose_topic_name, 10,
                               &Fusion::readHeadShoulderPoseTopic, this) ;
    _sub_hpose = _nh.subscribe(_head_pose_topic_name, 10,
                               &Fusion::readHeadPoseTopic, this) ;
    _sub_utt = _nh.subscribe(_utterance_topic_name, 10,
                             &Fusion::readUtteranceTopic, this);
    _sub_vad = _nh.subscribe(_vad_topic_name, 10,
                             &Fusion::readVadTopic, this);
                             
	_pub_intentionnality = _nh.advertise<riddle::Intention>("intention", 10, true);
	
	_pub_object = _nh.advertise<riddle::ObjectPosition>("ObjectPosition", 10, true);
}

Fusion::~Fusion()
{
    if (_objMap) delete _objMap;
}

void Fusion::init(int argc, char* argv[])
{
	ros::NodeHandle nh_t("~");
    nh_t.param("objectmap_path", _objmaps_file_name, std::string("../data/objects/objectsMap"));
	
	_speachDetected = false;
	_currIntDetected = false;
    _objMap = new ObjectsMap(_objmaps_file_name);

}

void Fusion::readUtteranceTopic(const riddle::Utterance & msg)
{
	
	for (int i=0 ; i<msg.length ; i++)
	{
		ROS_INFO("Utterance : %s", msg.utterances[i].c_str());
	}
	if(this->chooseNBest(msg))
	{
		ROS_INFO("Object found : : %s", _currObject.c_str());
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

void Fusion::readVadTopic(const riddle::VAD & msg)
{
	if (!msg.state.compare("start") || !msg.state.compare("process"))
		_speachDetected = true;
	else _speachDetected = false;

	this->updateInt(1, msg.header.stamp.toSec(), 0.1);
}

void Fusion::readHeadPoseTopic(const riddle::HeadPoseList & msg)
{
    _vFanelliResults.clear();

    cv::Vec<float,POSE_SIZE> vpose;

    for (int i = 0; i < (int)msg.headPoseLists.size(); i++)
    {
        const riddle::HeadPose & p = msg.headPoseLists[i];
        vpose[0] = p.pose[0];   vpose[1] = p.pose[1];   vpose[2] = p.pose[2];
        vpose[3] = p.pose[3];   vpose[4] = p.pose[4];   vpose[5] = p.pose[5];

        _vFanelliResults.push_back(vpose);
        
        //head Tilt
        this->updateInt(exp(-(vpose[3]*vpose[3])/100.), msg.header.stamp.toSec(), 1);
        //head Pan
		this->updateInt(exp(-(vpose[4]*vpose[4])/100.), msg.header.stamp.toSec(), 1);
    }
}

void Fusion::readHeadShoulderPoseTopic(const riddle::HeadShoulderPoseList & msg)
{
    _vFanelliResults.clear();

    cv::Vec<float,POSE_SIZE> vpose;

    for (int i = 0; i < (int)msg.headPoseLists.size(); i++)
    {
        const riddle::HeadPose & p = msg.headPoseLists[i];
        vpose[0] = p.pose[0];   vpose[1] = p.pose[1];   vpose[2] = p.pose[2];
        vpose[3] = p.pose[3];   vpose[4] = p.pose[4];   vpose[5] = p.pose[5];

        _vFanelliResults.push_back(vpose);
        
        //head Tilt
        this->updateInt(exp(-(vpose[3]*vpose[3])/100.), msg.header.stamp.toSec(), 1);
        //head Pan
		this->updateInt(exp(-(vpose[4]*vpose[4])/100.), msg.header.stamp.toSec(), 1);
    }
    
    for (int i=0 ; i<(int)msg.shoulderPanLists.size(); i++)
    {
		double pan = msg.shoulderPanLists[i];
		this->updateInt(exp(-(pan*pan)/100.), msg.header.stamp.toSec(), 1);
	}
}

void Fusion::updateInt(double value, long long int timestamp, double weight)
{
	double div=0;
	_currIntention=0;
	std::queue<double> newQueue, newWeightsQueue;
	double tempIntValue=0;
	double tempWeightValue=0;
	int numFeatures=0;
	
	while (!_features.empty())
	{
		if (timestamp - _timestamp.front() > 1)
		{
			_timestamp.pop();
			_features.pop();
			_weights.pop();
			continue;
		}
		
		tempIntValue = _features.front();
		tempWeightValue = _weights.front();
		_features.pop();
		_weights.pop();
		newQueue.push(tempIntValue);
		newWeightsQueue.push(tempWeightValue);
		
		div += tempWeightValue;
		_currIntention += tempIntValue*tempWeightValue;
		numFeatures++;
	}
	_features = newQueue;
	_weights = newWeightsQueue;
	
	_timestamp.push(timestamp);
	_features.push(value);
	_weights.push(weight);
	div+=weight;
	_currIntention += value*weight;
	numFeatures++;
	
	_currIntention /= div;
	
	_intention_msg.header.stamp = ros::Time::now();
	_intention_msg.intention = _currIntention;
	_intention_msg.buffSize = numFeatures;
	if (_currIntention  > 0.5)
	{
		_currIntDetected = true;
		_intention_msg.detected = 2;
	}
	else if (_currIntention > 0.2 && _currIntDetected)
	{
		_intention_msg.detected = 1;
	}
	else
	{
		_currIntDetected = false;
		_intention_msg.detected = 0;
	}
	_pub_intentionnality.publish(_intention_msg);
	
}

bool Fusion::chooseNBest(const riddle::Utterance& msg)
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
bool Fusion::parseReco(std::string sentence, std::string& objFound, std::vector<double>& objPos, std::string& objLocationName)
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

int main(int argc, char* argv[])
{
    ROS_INFO("Initializing fused_intention...\n");

    ros::init(argc, argv, "fused_intention");

    Fusion fusion;
    fusion.init(argc, argv);

    ROS_INFO("...fused_intention initialized\n");

    ros::spin();

    return 0;
}
