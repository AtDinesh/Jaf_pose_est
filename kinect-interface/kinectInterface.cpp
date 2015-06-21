#include "kinectInterface.h"

KinectInterface::KinectInterface(std::string ONIfile, bool trackSkel, std::string ONIFileToPlay)
{
    _ONIFilePath = ONIfile;
    _ONIFileToPlay = ONIFileToPlay;
    _isTrackingSkel = trackSkel;
    _kinectVar = new KinectStruct;
    _kinectVar->jointPositions.resize(MAX_USERS, std::vector<XnSkeletonJointPosition>(4));
    _kinectVar->isUserTracked.resize(MAX_USERS);
    _kinectVar->shoulderPan.resize(MAX_USERS);
    _currentFrame = 0;
}

KinectInterface::~KinectInterface()
{
    if (_kinectVar) delete _kinectVar;
}

void KinectInterface::init()
{
    if (_ONIFileToPlay.empty())
        this->initFromKinect();
    else
        this->initFromONIFile();
}

void KinectInterface::initFromONIFile()
{
    _nRetVal = _context.Init();
    if (_nRetVal != XN_STATUS_OK) throw KinectException("Error in kinect initialisation", xnGetStatusString(_nRetVal));

    _nRetVal = _context.OpenFileRecording(_ONIFileToPlay.c_str(), _player);
    if (_nRetVal != XN_STATUS_OK) throw KinectException("Error in kinect initialisation", xnGetStatusString(_nRetVal));

    //read only once
    _player.SetRepeat(false);

    xn::NodeInfoList list;
    _nRetVal = _context.EnumerateProductionTrees(XN_NODE_TYPE_DEPTH, NULL, list);
    if (_nRetVal != XN_STATUS_OK) throw KinectException("Error kinect not detected", xnGetStatusString(_nRetVal));

    //get number of frames
    _player.GetNumFrames((*list.Begin()).GetInstanceName(), _nbFrames);
    //std::cout << _nbFrames << std::endl;

    _nRetVal = _depth.Create(_context);
    if (_nRetVal != XN_STATUS_OK) throw KinectException("Error in depth generator", xnGetStatusString(_nRetVal));
    _depth.GetIntProperty ("ZPD", _kinectVar->focalLength);
    _depth.GetRealProperty ("ZPPS", _kinectVar->pixelSize);
    _kinectVar->pixelSize *= 2.f;

    _nRetVal = _image.Create(_context);
    if (_nRetVal != XN_STATUS_OK) throw KinectException("Error in RGB generator", xnGetStatusString(_nRetVal));

    _depth.GetAlternativeViewPointCap().SetViewPoint(_image);

    _nRetVal = _context.EnumerateExistingNodes(list, XN_NODE_TYPE_AUDIO);
    if (_nRetVal != XN_STATUS_OK) throw KinectException("Error kinect audio not detected", xnGetStatusString(_nRetVal));

    //For skeleton fitting
    if (_isTrackingSkel)
    {
        _nRetVal = _user.Create(_context);
        if (_nRetVal != XN_STATUS_OK) throw KinectException("Error in user generator", xnGetStatusString(_nRetVal));

        _user.RegisterUserCallbacks(newUser, lostUser, this, _userHandler);// to detect new user
        _user.RegisterToUserExit(exitUser, this, _userHandler);
        _user.RegisterToUserReEnter(reenterUser, this, _userHandler);

        _kinectVar->jointPositions.resize(4);
        xn::SkeletonCapability skel = _user.GetSkeletonCap();
        _nRetVal = skel.SetSkeletonProfile(XN_SKEL_PROFILE_UPPER);
        if (_nRetVal != XN_STATUS_OK) throw KinectException("Error in skeleton profile assignment generator", xnGetStatusString(_nRetVal));
    }

    _kinectVar->depthHeight=XN_VGA_Y_RES;
    _kinectVar->depthWidth=XN_VGA_X_RES;
    _kinectVar->RGBHeight=XN_VGA_Y_RES;
    _kinectVar->RGBWidth=XN_VGA_X_RES;

    _nRetVal = _context.StartGeneratingAll();
    if (_nRetVal != XN_STATUS_OK) throw KinectException("Error in generation start", xnGetStatusString(_nRetVal));

    this->updateAllMaps();

}

void KinectInterface::initFromKinect()
{
    _nRetVal = _context.Init();
    if (_nRetVal != XN_STATUS_OK) throw KinectException("Error in kinect initialisation", xnGetStatusString(_nRetVal));


    xn::NodeInfoList list;
    _nRetVal = _context.EnumerateProductionTrees(XN_NODE_TYPE_DEPTH, NULL, list);
    if (_nRetVal != XN_STATUS_OK) throw KinectException("Error kinect not detected", xnGetStatusString(_nRetVal));

    _nRetVal = _depth.Create(_context);
    if (_nRetVal != XN_STATUS_OK) throw KinectException("Error in depth generator", xnGetStatusString(_nRetVal));
    _depth.GetIntProperty ("ZPD", _kinectVar->focalLength);
    _depth.GetRealProperty ("ZPPS", _kinectVar->pixelSize);
    _kinectVar->pixelSize *= 2.f;

    _nRetVal = _image.Create(_context);
    if (_nRetVal != XN_STATUS_OK) throw KinectException("Error in RGB generator", xnGetStatusString(_nRetVal));

    _depth.GetAlternativeViewPointCap().SetViewPoint(_image);

    _nRetVal = _context.EnumerateExistingNodes(list, XN_NODE_TYPE_AUDIO);
    if (_nRetVal != XN_STATUS_OK) throw KinectException("Error kinect audio not detected", xnGetStatusString(_nRetVal));


    //For skeleton fitting
    if (_isTrackingSkel)
    {
        _nRetVal = _user.Create(_context);
        if (_nRetVal != XN_STATUS_OK) throw KinectException("Error in user generator", xnGetStatusString(_nRetVal));

        _user.RegisterUserCallbacks(newUser, lostUser, this, _userHandler);// to detect new user
        _user.RegisterToUserExit(exitUser, this, _userHandler);
        _user.RegisterToUserReEnter(reenterUser, this, _userHandler);

        _kinectVar->jointPositions.resize(4);
        xn::SkeletonCapability skel = _user.GetSkeletonCap();
        _nRetVal = skel.SetSkeletonProfile(XN_SKEL_PROFILE_UPPER);
        if (_nRetVal != XN_STATUS_OK) throw KinectException("Error in skeleton profile assignment generator", xnGetStatusString(_nRetVal));
    }


    _kinectVar->depthHeight=XN_VGA_Y_RES;
    _kinectVar->depthWidth=XN_VGA_X_RES;
    _kinectVar->RGBHeight=XN_VGA_Y_RES;
    _kinectVar->RGBWidth=XN_VGA_X_RES;

    //if there is a name for recording file, we create a recorder node
    if (!_ONIFilePath.empty())
    {
        try
        {
            _nRetVal = _recorder.Create(_context);
            if (_nRetVal != XN_STATUS_OK) throw KinectException("Unable to create a recorder from context", xnGetStatusString(_nRetVal));
            _nRetVal = _recorder.SetDestination(XN_RECORD_MEDIUM_FILE, _ONIFilePath.c_str());
            if (_nRetVal != XN_STATUS_OK) throw KinectException("Unable to set a proper destination for the recorded file", xnGetStatusString(_nRetVal));
            _nRetVal = _recorder.AddNodeToRecording(_depth);
            if (_nRetVal != XN_STATUS_OK) throw KinectException("Unable to add depth Map to recorder", xnGetStatusString(_nRetVal));
            _nRetVal = _recorder.AddNodeToRecording(_image);
            if (_nRetVal != XN_STATUS_OK) throw KinectException("Unable to add image Map to recorder", xnGetStatusString(_nRetVal));
            /*if (_isTrackingSkel)
            {
                _nRetVal = _recorder.AddNodeToRecording(_user);
                if (_nRetVal != XN_STATUS_OK) throw KinectException("Unable to add skel Map to recorder", xnGetStatusString(_nRetVal));
            }*/
        }
        catch(KinectException& e)
        {
            std::cout << e.what() << std::endl;
            std::cout << "Warning : Stream will not be recorded" << std::endl;
            _ONIFilePath="";
        }
    }

    //start generation
    _nRetVal = _context.StartGeneratingAll();
    if (_nRetVal != XN_STATUS_OK) throw KinectException("Error in generation start", xnGetStatusString(_nRetVal));

    this->updateAllMaps();
}

void KinectInterface::updateAllMaps()
{
    _nRetVal = _context.WaitAndUpdateAll();
    if (_nRetVal != XN_STATUS_OK) throw("Failed updating kinect", xnGetStatusString(_nRetVal));

    _depth.GetMetaData(_kinectVar->depthMetaData);
    _image.GetMetaData(_kinectVar->imageMetaData);

    if (_isTrackingSkel)
    {
        XnUserID aUsers[MAX_USERS];
        XnUInt16 nUsers = MAX_USERS;
        _user.GetUsers(aUsers, nUsers);

        for (unsigned int i=0 ; i < nUsers ; i++)
        {
            if (_user.GetSkeletonCap().IsTracking(aUsers[i]))
            {
                _kinectVar->isUserTracked[i] = true;
                _user.GetSkeletonCap().IsValid();
                _user.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_HEAD, _kinectVar->jointPositions[i][0]);
                _user.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_NECK, _kinectVar->jointPositions[i][1]);
                _user.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_LEFT_SHOULDER, _kinectVar->jointPositions[i][2]);
                _user.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_RIGHT_SHOULDER, _kinectVar->jointPositions[i][3]);

                /*double result = (_kinectVar->jointPositions[i][2].position.X+_kinectVar->jointPositions[i][3].position.X)/2;
                result -= _kinectVar->jointPositions[i][2].position.X;

                double op = (_kinectVar->jointPositions[i][2].position.Y+_kinectVar->jointPositions[i][3].position.Y)/2;
                op -= _kinectVar->jointPositions[i][2].position.Y;

                result = atan(op/result)*180./3.14159265359;
                if (result != result) result=0;*/

                double vecX = -(_kinectVar->jointPositions[i][2].position.X-_kinectVar->jointPositions[i][3].position.X);
                double vecZ = -(_kinectVar->jointPositions[i][2].position.Z-_kinectVar->jointPositions[i][3].position.Z);
                double norm = sqrt(vecX*vecX + vecZ*vecZ);

                double angle = acos(vecX/norm);
                if (acos(vecZ/norm) >  3.14159265359/2.) angle = -angle;

                _kinectVar->shoulderPan[i] = angle*180./3.14159265359;
            }
            else
            {
                _kinectVar->isUserTracked[i] = false;
                _kinectVar->shoulderPan[i] = 0;
            }
        }
    }

    _currentFrame++;
    //std::cout << _currentFrame << std::endl;
    //std::cout << _nbFrames << std::endl;

    if (!_ONIFileToPlay.empty())
    {
        _player.TellTimestamp(_kinectVar->timestamp);
        _kinectVar->isEOF = _player.IsEOF();
        if (_currentFrame == _nbFrames) _kinectVar->isEOF = true;
    }

    if (!_ONIFilePath.empty() && _ONIFileToPlay.empty()) _recorder.Record();
}

void KinectInterface::getKinectData(KinectStruct *&kinectData)
{
    kinectData->depthHeight = _kinectVar->depthHeight;
    kinectData->depthWidth = _kinectVar->depthWidth;
    kinectData->RGBHeight = _kinectVar->RGBHeight;
    kinectData->RGBWidth = _kinectVar->RGBWidth;
    kinectData->focalLength = _kinectVar->focalLength;
    kinectData->pixelSize = _kinectVar->pixelSize;
    kinectData->isUserTracked = _kinectVar->isUserTracked;
    kinectData->jointPositions = _kinectVar->jointPositions;
    kinectData->shoulderPan = _kinectVar->shoulderPan;
    kinectData->depthMetaData.CopyFrom(_kinectVar->depthMetaData);
    kinectData->imageMetaData.CopyFrom(_kinectVar->imageMetaData);
    kinectData->isEOF = _kinectVar->isEOF;
    kinectData->timestamp = _kinectVar->timestamp;
}

void XN_CALLBACK_TYPE KinectInterface::newUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
    KinectInterface* self = (KinectInterface*)pCookie;
    std::cout << "start tracking user : " << nId << std::endl;
    self->_user.GetSkeletonCap().StartTracking(nId);
}

void XN_CALLBACK_TYPE KinectInterface::lostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
    KinectInterface* self = (KinectInterface*)pCookie;
    std::cout << "stop tracking user : " << nId << std::endl;
    self->_user.GetSkeletonCap().StopTracking(nId);
}

void XN_CALLBACK_TYPE KinectInterface::exitUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
    std::cout << "user exit..." << nId << std::endl;
}

void XN_CALLBACK_TYPE KinectInterface::reenterUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
    std::cout << "user reenter..." << nId << std::endl;
}
