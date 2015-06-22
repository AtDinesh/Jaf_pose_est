#include "kinectgrabber.h"
#include <iostream>

KinectGrabber::KinectGrabber(void (*call_back_function)(const float *, const unsigned char *)):
    CallBackFunction(call_back_function), m_pDepth1(NULL), m_pImage(NULL)
{
    if(!init())
    {
        std::cout<<"SimpleViewer: Initialization failed."<<std::endl;
        exit(1);
    }
}

KinectGrabber::~KinectGrabber()
{
    delete[] m_pDepth1; delete[] m_pImage;

    //    if (m_streams != NULL)
    //    {
    //        delete []m_streams;
    //    }
}

unsigned int KinectGrabber::init()
{
    std::cout<<"Initializing device..."<<std::endl;

    is_running = false;

    _nRetVal = _context.Init();
    if (_nRetVal != XN_STATUS_OK) {
        std::cerr<<"Error in kinect initialisation"<<xnGetStatusString(_nRetVal)<<std::endl;
        return 0;
    }

    _nRetVal = _context.EnumerateProductionTrees(XN_NODE_TYPE_DEPTH, NULL, _list);

    if (_nRetVal != XN_STATUS_OK) {
        std::cerr<<"Error kinect not detected "<<xnGetStatusString(_nRetVal)<<std::endl;
        return 0;
    }

    _nRetVal = _depth.Create(_context);
    if (_nRetVal != XN_STATUS_OK) {
        std::cerr<<"Error in depth generator "<<xnGetStatusString(_nRetVal)<<std::endl;
        return 0;
    }

    _depth.GetIntProperty ("ZPD", _focalLength);
    _depth.GetRealProperty ("ZPPS", _pixelSize);
    _pixelSize *= 2.f;

    _nRetVal = _image.Create(_context);

    if (_nRetVal != XN_STATUS_OK) {
        std::cerr<<"Error in RGB generator "<<xnGetStatusString(_nRetVal)<<std::endl;
        return 0;
    }

    _depth.GetAlternativeViewPointCap().SetViewPoint(_image);

    _nRetVal = _context.EnumerateExistingNodes(_list, XN_NODE_TYPE_AUDIO);

    if (_nRetVal != XN_STATUS_OK) {
        std::cerr<<"Error kinect audio not detected "<<xnGetStatusString(_nRetVal)<<std::endl;
        return 0;
    }

    _depthHeight=XN_VGA_Y_RES;
    _depthWidth=XN_VGA_X_RES;
    _RGBHeight=XN_VGA_Y_RES;
    _RGBWidth=XN_VGA_X_RES;

    //start generation
    _nRetVal = _context.StartGeneratingAll();
    if (_nRetVal != XN_STATUS_OK) {
        std::cerr<<"Error in generation start "<<xnGetStatusString(_nRetVal)<<std::endl;
        return 0;
    }

    m_width = _depthWidth;
    m_height = _depthHeight;

    m_pDepth1 = new float[m_width*m_height];//*sizeof(float)]; //this has to be a bug, I don't think sizeof(float) is necessary
    m_pImage = new unsigned char[m_width*m_height*3]; //RGB channels

    //    img_rgb.create(m_height, m_width, CV_8UC3);
    //    img_depth.create(m_height, m_width, CV_32FC1);

    return 1;
}

void* KinectGrabber::thread_function(void* owner)
{
    KinectGrabber* _this = (KinectGrabber*)owner;
    float* pDepth=new float[_this->m_width*_this->m_height];//*sizeof(float)];
    unsigned char* pImage=new unsigned char[_this->m_width*_this->m_height*3];
    int ch1=-1,ch2=-1;
    while(_this->is_running)
    {
        //_this->grabb(pDepth, pImage,ch1);
        //_this->grabb(pDepth, pImage,ch2);
        _this->grabb(pDepth, pImage);
        //if(ch1>=0 && ch2>=0 && ch1!=ch2)
        _this->CallBackFunction(pDepth, pImage);
        //ch1=ch2=-1;
    }
    delete[] pDepth;
    delete[] pImage;
    pthread_exit(NULL);
}

void KinectGrabber::start()
{
    if(!is_running)
    {
        is_running = true;
        if(pthread_create(&grabb_thread, NULL, thread_function, (void*)this))
        {
            std::cout<<"Error: unable to create grabb thread.\n"<<std::endl;
            exit(1);
        }
    }
}

//void KinectGrabber::calculateHistogram(float* pHistogram, int histogramSize,
//                                       const openni::VideoFrameRef& frame)
//{
//    const openni::DepthPixel* pDepth = (const openni::DepthPixel*)frame.getData();

//    // Calculate the accumulative histogram (the yellow display...)
//    memset(pHistogram, 0, histogramSize*sizeof(float));

//    int restOfRow = frame.getStrideInBytes() / sizeof(openni::DepthPixel) - frame.getWidth();

//    int height = frame.getHeight();
//    int width = frame.getWidth();

//    unsigned int nNumberOfPoints = 0;
//    for (int y = 0; y < height; ++y)
//    {
//        for (int x = 0; x < width; ++x, ++pDepth)
//        {
//            if (*pDepth != 0)
//            {
//                pHistogram[*pDepth]++;
//                nNumberOfPoints++;
//            }
//        }
//        pDepth += restOfRow;
//    }

//    for (int nIndex=1; nIndex<histogramSize; nIndex++)
//    {
//        pHistogram[nIndex] += pHistogram[nIndex-1];
//    }
//    if (nNumberOfPoints)
//    {
//        for (int nIndex=1; nIndex<histogramSize; nIndex++)
//        {
//            pHistogram[nIndex] = (256 * (1.0f - (pHistogram[nIndex] / nNumberOfPoints)));
//        }
//    }
//}

void KinectGrabber::stop()
{
    if(is_running)
    {
        is_running = false;

        pthread_join(grabb_thread, NULL);

        std::cout<<"Stoping depth stream."<<std::endl;

        //m_depthStream.stop();
        std::cout<<"Destroying depth stream."<<std::endl;

        //m_depthStream.destroy();
        std::cout<<"Stoping color stream."<<std::endl;

        //m_colorStream.stop();
        std::cout<<"Destroying color stream."<<std::endl;

        //m_colorStream.destroy();
        std::cout<<"Closing device."<<std::endl;

        //m_device.close();
        std::cout<<"Shuting down Openni."<<std::endl;
    }
}

void KinectGrabber::grabb(float* pDepth, unsigned char* pImage)
{

    //get data from kinect
    _nRetVal = _context.WaitAndUpdateAll();
    if (_nRetVal != XN_STATUS_OK) {
        std::cerr<<"Failed updating kinect "<<xnGetStatusString(_nRetVal)<<std::endl;
        return;
    }

    _depth.GetMetaData(_depthMetaData);
    _image.GetMetaData(_imageMetaData);

    memset(m_pDepth1, 0, m_width*m_height*sizeof(float));

    //unsigned char * ptr_rgb = (unsigned char*)img_rgb.data;
    //float * ptr_depth = (float *)img_depth.data;

    //buffer rgb image
    memcpy(pImage,(unsigned char*)_imageMetaData.Data(), m_width*m_height*3);
    //memcpy(ptr_rgb, (unsigned char*)pImage, m_width*m_height*3);
    //divide the depth values by a 1000

    //generate 3D image
    for(int y = 0; y < m_height; y++)
    {
        //cv::Vec3f* Mi = g_im3D.ptr<cv::Vec3f>(y);
        for(int x = 0; x < m_width; x++){
            float d = (float)_depthMetaData(x,y);
            if ( d != 0 ){
                m_pDepth1[y*m_width + x] = d/1000.0;
            }
        }
    }
    memcpy(pDepth, (float *)m_pDepth1, sizeof(float)*m_width*m_height);
    //memcpy(ptr_depth, (float *)pDepth, sizeof(float)*m_width*m_height);

    //    cv::imshow("IMG-RGB", img_rgb);
    //    cv::imshow("IMG-DEPTH", img_depth);
    //    cv::waitKey(10);
}
