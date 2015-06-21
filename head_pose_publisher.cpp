#include <iostream>
#include <string>
#include <XnCppWrapper.h>
#include "head-pose-fanelli/CRForestEstimator.h"
#include "head-pose-fanelli/gl_camera.hpp"

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "image_transport/image_transport.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


#include "riddle/HeadPoseList.h"


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

namespace riddle {

class FanelliInt
{
public:
    FanelliInt();//std::string configFile);
    ~FanelliInt();
    void start();
    void process();//xn::DepthMetaData &metaData, std::vector< cv::Vec<float,POSE_SIZE> >& means);
    //    gl_camera& getCamera();

    void depthMapReader(const sensor_msgs::PointCloud2ConstPtr& msg);

protected:
    void loadConfig(const char* filename);
    //void extract3DMap(xn::DepthMetaData &metaData);

protected:
    //global variables from original fanelli soft
    int g_ntrees;			std::string g_treepath;
    float g_maxv;		float g_larger_radius_ratio;
    float g_smaller_radius_ratio;
    int g_stride;	int g_max_z;	int g_th;
    int g_im_w;	int g_im_h;

    cv::Mat g_im3D;

    //camera related vars
    //XnUInt64 g_focal_length;
    //XnDouble g_pixel_size;
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

    //bool g_head_depth_ready; (seems to be for face detection control)
    bool g_cloud_ready;
    //bool g_transform_ready; (could be removed for now)


private: //ros related variables
    ros::NodeHandle _nh;
    ros::Subscriber _sub;
    ros::Publisher _pub;

    std::string _depth_msg_name;
    std::string _config_files_name;

    bool is_initialized;
    riddle::HeadPoseList _head_poses;
    riddle::HeadPose _h_pose;
};

}

using namespace riddle;

FanelliInt::FanelliInt():
    _depth_msg_name("depth_msg")//, _config_files_name("config_file")
{
    using namespace std;

    g_im_w = 640;           g_im_h = 480;
    g_first_rigid = true;   g_prob_th = 1.0f;

    ros::NodeHandle nh_t("~");
    nh_t.param("config_file", _config_files_name, string("config.txt"));

    ROS_INFO("Loading config file [%s] ", _config_files_name.c_str());
    this->loadConfig(_config_files_name.c_str());
    this->_detector = new CRForestEstimator();

    if( !this->_detector->loadForest(g_treepath.c_str(), g_ntrees) )
    {
        ROS_ERROR("Could not read forest!");
        exit(-1);
    }

    //buffer for each head pose
    //_h_pose.pose.resize(POSE_SIZE);

    //g_im3D.create(g_im_h,g_im_w,CV_32FC3);
    //g_head_depth_ready = false;
    g_cloud_ready = false;
    //g_transform_ready = false;
    //ROS_INFO("Initialization successful!")
}

FanelliInt::~FanelliInt()
{
    delete this->_detector;
}

void FanelliInt::start(){

    _sub = _nh.subscribe(_depth_msg_name, 1,
                         &FanelliInt::depthMapReader, this);

    _pub = _nh.advertise<riddle::HeadPoseList>("head_pose_list", 10, true);

}

//void FanelliInt::init(  &focal_length, XnDouble &pixel_size)
//{
//    g_focal_length = focal_length;
//    g_pixel_size = pixel_size;
//}

//perhaps I might try to sendin the depth data in cv::Mat format...FIXME!
void FanelliInt::process() //xn::DepthMetaData &metaData, std::vector< cv::Vec<float,POSE_SIZE> >& means)
{
    using namespace std;

    //need to make it opencv or ros friendly
    //this->extract3DMap(metaData);//compute g_im3D
    std::vector< cv::Vec<float,POSE_SIZE> > means;

    g_means.clear();    g_votes.clear();    g_clusters.clear();

    //do the actual estimation
    this->_detector->estimate( 	this->g_im3D,
                                g_means,
                                g_clusters,
                                g_votes,
                                g_stride,
                                g_maxv,
                                g_prob_th,
                                g_larger_radius_ratio,
                                g_smaller_radius_ratio,
                                false,
                                g_th
                                );

    means.assign(g_means.begin(), g_means.end());

    //Fill in the data to the publisher...
    this->_head_poses.headPoseLists.resize(means.size());

    for (size_t i = 0; i < means.size(); i++)
    {
        cv::Vec<float,POSE_SIZE> v = means[i];

        for (size_t j = 0; j < POSE_SIZE; j++)
            //this->_h_pose.pose[j] = v[j];
            this->_head_poses.headPoseLists[i].pose[j] = v[j];//this->_h_pose.pose;
    }

    //ROS_INFO("Detected [%d] heads", (int)means.size());
    this->_head_poses.header.stamp = ros::Time::now();
    _pub.publish(this->_head_poses);
}

void FanelliInt::loadConfig(const char* filename)
{

    using namespace std;

    ifstream in(filename);
    string dummy;

    // and add it to the treepath
    string f_name = filename;

    size_t len = f_name.find_last_of('/');
    string path_ = f_name.substr(0, len);

    if(in.is_open()) {

        // Path to trees
        in >> dummy;    in >> g_treepath;
        g_treepath = path_ + '/' + g_treepath;

        //correct g_treepath....to encorporate full path

        // Number of trees
        in >> dummy;    in >> g_ntrees;

        in >> dummy;    in >> g_maxv;

        in >> dummy;    in >> g_larger_radius_ratio;

        in >> dummy;    in >> g_smaller_radius_ratio;

        in >> dummy;    in >> g_stride;

        in >> dummy;    in >> g_max_z;

        in >> dummy;    in >> g_th;
    } else {
        ROS_ERROR("File not found [%s] ", filename);
        exit(-1);
    }
    in.close();

    ROS_INFO("------------------------------------");
    ROS_INFO("Estimation:       ");
    ROS_INFO("Trees:            %d  %s", g_ntrees, g_treepath.c_str());
    ROS_INFO("Stride:           %d ", g_stride);
    ROS_INFO("Max Variance:     %f ", g_maxv);
    ROS_INFO("Max Distance:     %d ", g_max_z);
    ROS_INFO("Head Threshold:   %d ", g_th);
    ROS_INFO(""); //newline
    ROS_INFO("------------------------------------");

    //g_treepath = "/home/christophe/.local/share/nao/" + g_treepath;
}

void FanelliInt::depthMapReader(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    PointCloud cloud;
    pcl::fromROSMsg(*msg, cloud);

    //g_cloud_frame = cloud.header.frame_id;
    g_cloud_ready = true;

    //if(!g_head_depth_ready) return; [sort of like if face detection occured]

    //Mat img3D;
    this->g_im3D = cv::Mat::zeros(cloud.height, cloud.width, CV_32FC3);
    //img3D.create(cloud.height, cloud.width, CV_32FC3);

    int yMin, xMin, yMax, xMax;
    yMin = 0; xMin = 0;
    yMax =  this->g_im3D.rows; xMax =  this->g_im3D.cols;

    //get 3D from depth
    for(int y = yMin ; y <  this->g_im3D.rows; y++) {
        cv::Vec3f* img3Di =  this->g_im3D.ptr<cv::Vec3f>(y);

        for(int x = xMin; x <  this->g_im3D.cols; x++) {
            pcl::PointXYZ p = cloud.at(x,y);

            //WARNING! Please try to harmonize these steps
            //if((p.z>g_head_depth-0.2) && (p.z<g_head_depth+0.2))
            if (p.z < g_max_z && p.z > 0)
            {
                img3Di[x][0] = p.x*1000;
                img3Di[x][1] = p.y*1000;
                img3Di[x][2] = hypot(img3Di[x][0], p.z*1000); //they seem to have trained with incorrectly projected 3D points
                //img3Di[x][2] = p.z*1000;
            } else {
                img3Di[x] = 0;
            }
        }
    }

    this->process();
    
    printf("processed point cloud...\n");
    fflush(stdout);
}


int main(int argc, char * argv[])
{

    ros::init(argc, argv, "head_pose_publisher");

    ROS_INFO(" [INFO] Initializing head pose publisher (based on adaptation of Fanelli's demo...");
    fflush(stdout);

    riddle::FanelliInt head_pose_server;

    ROS_INFO(" [INFO]...init done!"); fflush(stdout);

    head_pose_server.start();

    ros::spin();

    return 0;
}
