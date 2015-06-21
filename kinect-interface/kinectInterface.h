#ifndef KINECTINTERFACE_H
#define KINECTINTERFACE_H

#include <ni/XnCppWrapper.h>
#include <vector>
#include <iostream>
#include <string>
#include <math.h>
#include "kinectException.h"

#define MAX_USERS 15

typedef struct
{
	int depthWidth;
	int depthHeight;
	int RGBWidth;
	int RGBHeight;
	XnUInt64 focalLength;
	XnDouble pixelSize;
	xn::DepthMetaData depthMetaData;
	xn::ImageMetaData imageMetaData;
	XnUInt64 timestamp;
	bool isEOF;

	std::vector<std::vector<XnSkeletonJointPosition> > jointPositions; //head, neck, shoulder_left, shoulder_right
	std::vector<double> shoulderPan;
	std::vector<bool> isUserTracked;


}KinectStruct;

class KinectInterface
{
	public:
		KinectInterface(std::string ONIfile="", bool trackSkel=true, std::string ONIfileToPlay="");
		~KinectInterface();
		void init();
		void updateAllMaps();
		void getKinectData(KinectStruct*& kinectData);

	protected:
		void initFromONIFile();
		void initFromKinect();
		static void XN_CALLBACK_TYPE newUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie);
		static void XN_CALLBACK_TYPE lostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie);
		static void XN_CALLBACK_TYPE exitUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie);
		static void XN_CALLBACK_TYPE reenterUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie);

	protected:
		XnStatus _nRetVal;
		xn::Context _context;
		xn::AudioGenerator _audio;
		xn::DepthGenerator _depth;
		xn::ImageGenerator _image;
		xn::UserGenerator _user;
		xn::Player _player;
		XnCallbackHandle _userHandler;

		KinectStruct* _kinectVar;

		//internal vars
		std::string _ONIFilePath;
		std::string _ONIFileToPlay;
		xn::Recorder _recorder;
		bool _isTrackingSkel;
		XnUInt32 _nbFrames;
		XnUInt32 _currentFrame;
};

#endif
