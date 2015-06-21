#include <iostream>
#include <string>
#include <XnCppWrapper.h>
#include "head-pose-fanelli/CRForestEstimator.h"
#include "head-pose-fanelli/gl_camera.hpp"

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>

#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "image_transport/image_transport.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


#include "riddle/HeadShoulderPoseList.h"


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

namespace riddle {

class HeadShoulderPosePub
{
public:
    HeadShoulderPosePub();//std::string configFile);
    ~HeadShoulderPosePub();
    void start();
    void process();//xn::DepthMetaData &metaData, std::vector< cv::Vec<float,POSE_SIZE> >& means);
    //    gl_camera& getCamera();

    void depthMapReader(const sensor_msgs::PointCloud2ConstPtr& msg);
    bool is_ok() {  return this->_nh.ok();   }

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
    tf::TransformListener *_listener;

    std::string _depth_msg_name;
    std::string _config_files_name;

    bool is_initialized;
    riddle::HeadShoulderPoseList _head_shoulder_poses;
    riddle::HeadPose _h_pose;
};

}

using namespace riddle;

HeadShoulderPosePub::HeadShoulderPosePub():
    _depth_msg_name("depth_msg")//, _config_files_name("config_file")
{
    using namespace std;

    g_im_w = 640;           g_im_h = 480;
    g_first_rigid = true;   g_prob_th = 1.0f;

    ros::NodeHandle nh_t("~");
    nh_t.param("config_file", _config_files_name, string("../data/fanelli/config.txt"));

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
    this->_listener = new tf::TransformListener();
    this->_listener->waitForTransform("/camera_depth_frame", "/camera_depth_optical_frame", ros::Time(), ros::Duration(1.0));

    //g_im3D.create(g_im_h,g_im_w,CV_32FC3);
    //g_head_depth_ready = false;
    g_cloud_ready = false;
    //g_transform_ready = false;
    //ROS_INFO("Initialization successful!")
}

HeadShoulderPosePub::~HeadShoulderPosePub()
{
    //delete this->_listener;
    //delete this->_detector;
}

void HeadShoulderPosePub::start(){
    _sub = _nh.subscribe(_depth_msg_name, 1,
                         &HeadShoulderPosePub::depthMapReader, this);

    _pub = _nh.advertise<riddle::HeadShoulderPoseList>("head_shoulder_pose_list", 10, true);
    

}

//void FanelliInt::init(  &focal_length, XnDouble &pixel_size)
//{
//    g_focal_length = focal_length;
//    g_pixel_size = pixel_size;
//}

//perhaps I might try to sendin the depth data in cv::Mat format...FIXME!
void HeadShoulderPosePub::process() //xn::DepthMetaData &metaData, std::vector< cv::Vec<float,POSE_SIZE> >& means)
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
    this->_head_shoulder_poses.headPoseLists.resize(means.size());

    for (size_t i = 0; i < means.size(); i++)
    {
        cv::Vec<float,POSE_SIZE> v = means[i];

        for (size_t j = 0; j < POSE_SIZE; j++)
            //this->_h_pose.pose[j] = v[j];
            this->_head_shoulder_poses.headPoseLists[i].pose[j] = v[j];//this->_h_pose.pose;
    }
    
    this->_head_shoulder_poses.shoulderPanLists.clear();
    this->_head_shoulder_poses.headSkeletonLists.clear();
    riddle::HeadPose headSkeleton;

    tf::StampedTransform transform;

    for ( int i=0 ; i<4 ; i++) //limit to four
    {
        //printf(":");
        std::ostringstream oss, oss_h;
        oss << "neck_" << i;                    oss_h<<"head_" << i;
        std::string childFrame = oss.str();     std::string headFrame = oss_h.str();

        //oss1 << "/left_shoulder_" << i;       //oss2 << "/right_shoulder_" << i;
        //tf::Vector3 leftShoulder, rightShoulder;
        tfScalar yaw, pitch, roll;

        //printf(" %s [-%d-]  ", childFrame.c_str(), static_cast<int>(_listener->frameExists(childFrame)));

        if (_listener->frameExists(childFrame))
        {
            try{
                _listener->lookupTransform("/camera_depth_frame", childFrame, ros::Time(), transform);
                transform.getBasis().getEulerYPR(yaw, pitch, roll);

                yaw = (yaw*180/3.14) - 90;
                if (yaw < -180) yaw = 360 - yaw;

                this->_head_shoulder_poses.shoulderPanLists.push_back(yaw);

                //also if head position exists
            }
            catch(tf::TransformException& ex)
            {
                std::cout << "Failure at "<< ros::Time::now() << std::endl;
                std::cout << "Exception thrown:" << ex.what()<< std::endl;
            }
            //printf(":");
        }

        if (_listener->frameExists(headFrame))
        {
            try{
                _listener->lookupTransform("/camera_depth_frame", headFrame, ros::Time(), transform);
                transform.getBasis().getEulerYPR(yaw, pitch, roll);

                tf::Vector3 v = transform.getOrigin();
                headSkeleton.pose[0] = v.getX();    headSkeleton.pose[3] = yaw;
                headSkeleton.pose[1] = v.getY();    headSkeleton.pose[4] = pitch;
                headSkeleton.pose[2] = v.getZ();    headSkeleton.pose[5] = roll;

                this->_head_shoulder_poses.headSkeletonLists.push_back(headSkeleton);

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
    //printf("\n");
    //fflush(stdout);

    this->_head_shoulder_poses.header.stamp = ros::Time::now();
    _pub.publish(this->_head_shoulder_poses);
}

void HeadShoulderPosePub::loadConfig(const char* filename)
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

void HeadShoulderPosePub::depthMapReader(const sensor_msgs::PointCloud2ConstPtr& msg)
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

            float factor = 1000.;
            //WARNING! Please try to harmonize these steps
            //if((p.z>g_head_depth-0.2) && (p.z<g_head_depth+0.2))
            if (p.z < g_max_z && p.z > 0)
            {
                img3Di[x][0] = p.x*factor;
                img3Di[x][1] = p.y*factor;
                img3Di[x][2] = hypot(img3Di[x][0], p.z*factor); //they seem to have trained with incorrectly projected 3D points
                //img3Di[x][2] = p.z*1000;
            } else {
                img3Di[x] = 0;
            }
        }
    }
    
    /*for(int y = 0; y < g_im3D.rows; y++)
	{
		cv::Vec3f* Mi = g_im3D.ptr<cv::Vec3f>(y);
		for(int x = 0; x < g_im3D.cols; x++){

			float d = (float)metaData(x,y);

			if ( d < g_max_z && d > 0 ){

				valid_pixels++;

				Mi[x][0] = ( float(d * (x - 320)) / f );
				Mi[x][1] = ( float(d * (y - 240)) / f );
				Mi[x][2] = d;

			}
			else
				Mi[x] = 0;

		}
	}*/

    this->process();
}

/*
class EchoListener
{
public:
    tf::TransformListener tf;

    EchoListener()      {       };
    ~EchoListener()     {       };
};
*/

int main(int argc, char * argv[])
{

    ros::init(argc, argv, "head_pose_publisher");

    ROS_INFO(" [INFO] Initializing head pose publisher (based on adaptation of Fanelli's demo...");
    fflush(stdout);

    riddle::HeadShoulderPosePub head_pose_server;

    // EchoListener echoListener;

    ROS_INFO(" [INFO]...init done!"); fflush(stdout);

    head_pose_server.start();

    /*
    while(head_pose_server.is_ok())
    {
        try
        {
            tf::StampedTransform echo_transform;
            echoListener.tf.lookupTransform("/camera_depth_frame",
                                            "/camera_depth_optical_frame", ros::Time(),
                                            echo_transform);
            std::cout.precision(3);
            std::cout.setf(std::ios::fixed,std::ios::floatfield);
            std::cout << "At time " << echo_transform.stamp_.toSec() << std::endl;
            double yaw, pitch, roll;
            echo_transform.getBasis().getRPY(roll, pitch, yaw);
            tf::Quaternion q = echo_transform.getRotation();
            tf::Vector3 v = echo_transform.getOrigin();
            std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
            std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", "
                      << q.getZ() << ", " << q.getW() << "]" << std::endl
                      << "            in RPY [" <<  roll << ", " << pitch << ", " << yaw << "]" << std::endl;

            //print transform
        }
        catch(tf::TransformException& ex)
        {
            std::cout << "Failure at "<< ros::Time::now() << std::endl;
            std::cout << "Exception thrown:" << ex.what()<< std::endl;
            std::cout << "The current list of frames is:" <<std::endl;
            std::cout << echoListener.tf.allFramesAsString()<<std::endl;
        }
        sleep(1);

        ros::spinOnce();
    }
    */
    ros::spin();

    return 0;
}
