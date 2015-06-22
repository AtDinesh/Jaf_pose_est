#ifndef KINECTGRABBER_H
#define KINECTGRABBER_H

#include <ni/XnCppWrapper.h>
#include <pthread.h>
//#include <cv.h>
//#include <highgui.h>

#define MAX_DEPTH 10000


class KinectGrabber
{
public:
    KinectGrabber(void (*call_back_function)(const float *, const unsigned char *));
    ~KinectGrabber();
    virtual unsigned int init();
    void start();
    void stop();

    void (*CallBackFunction)(const float * depth, const unsigned char * image);

protected:
    /* OpenNI2 related stuff
    openni::VideoFrameRef		m_depthFrame;
    openni::VideoFrameRef		m_colorFrame;

    openni::Device			m_device;
    openni::VideoStream			m_depthStream;
    openni::VideoStream			m_colorStream;
    openni::VideoStream**		m_streams;
*/
    //relplacing OpenNI1 stuff
    xn::Context _context;
    xn::NodeInfoList _list;
    xn::DepthGenerator _depth;
    xn::ImageGenerator _image;
    XnStatus _nRetVal;
    XnUInt64 _focalLength;
    XnDouble _pixelSize;
    xn::DepthMetaData _depthMetaData;
    xn::ImageMetaData _imageMetaData;
    XnUInt64 _timestamp;


    int _depthWidth;
    int _depthHeight;
    int _RGBWidth;
    int _RGBHeight;

    //    cv::Mat img_rgb, img_depth;
private:
    bool is_running;
    static void *thread_function(void *owner);
    //void calculateHistogram(float* pHistogram, int histogramSize, const openni::VideoFrameRef& frame);
    void grabb(float* pDepth, unsigned char* pImage);
    //void grabb_depth(float *pDepth);//
    //void grabb_rgb(unsigned char *pImage);

    unsigned int		m_width;
    unsigned int		m_height;
    float*	m_pDepth1;
    //openni::RGB888Pixel*	m_pImage; FIXME
    unsigned char* m_pImage; //place holder pointer to float

    pthread_t grabb_thread;
};

#endif // KINECTGRABBER_H
