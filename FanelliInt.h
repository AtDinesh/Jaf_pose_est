#ifndef FANELLIINT_H
#define FANELLIINT_H

#include <string.h>
#include <XnCppWrapper.h>
#include "head-pose-fanelli/CRForestEstimator.h"
#include "head-pose-fanelli/gl_camera.hpp"

//#include <cv.hpp>

namespace riddle
{
class FanelliInt
{
public:
    FanelliInt(std::string configFile);
    ~FanelliInt();
    void init(XnUInt64 &focal_length, XnDouble &pixel_size);
    cv::Mat process(xn::DepthMetaData &metaData, std::vector< cv::Vec<float,POSE_SIZE> >& means);
    gl_camera& getCamera();

protected:
    void loadConfig(const char* filename);
    void extract3DMap(xn::DepthMetaData &metaData);

protected:
    //global variables from original fanelli soft
    int g_ntrees;
    std::string g_treepath;
    float g_maxv;
    float g_larger_radius_ratio;
    float g_smaller_radius_ratio;
    int g_stride;
    int g_max_z;
    int g_th;
    cv::Mat g_im3D;
    int g_im_w;
    int g_im_h;
    XnUInt64 g_focal_length;
    XnDouble g_pixel_size;
    bool g_first_rigid;

    //results
    std::vector< cv::Vec<float,POSE_SIZE> > g_means; //outputs
    std::vector< std::vector< Vote > > g_clusters; //full clusters of votes
    std::vector< Vote > g_votes; //all votes returned by the forest
    //threshold for the probability of a patch to belong to a head
    float g_prob_th;

    //maybe to remove
    gl_camera g_camera;

    //new variables
    CRForestEstimator* _detector;

};

}

#endif
