#include <ros/ros.h>


#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/PointCloud2.h"
#include "image_transport/image_transport.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


//Point clouds
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "kinectgrabber.h"

#include <sys/time.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include "Globals.h"
#include "Vector.h"
#include "AncillaryMethods.h"
#include "config_file/ConfigFile.h"

#include <stdio.h>

#include "detector.h"
#include "depthdetector.h"
#include "depthdetector_lm.h"
#include "detector_seg.h"
#include "depthdetector_seg.h"
#include "depthdetector_lm_seg.h"

#include "pointcloud.h"
#include "groundplaneestimator.h"

#include "fovis/fovis.hpp"
//#include "Tracker.h"
#include "Eigen/Core"

#include <queue>

//After Tracker.h removal
#include <CImg/CImg.h>
#include "camera/Camera.h"
#include "Visualization.h"
#include "Detections.h"
#include <cstdio>               //for getch function

// include OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//to use pca (libpca)
//#include <vector>
//#include "/home/datchuth/src-build/include/pca.h"

#include <cv.h>
#include <highgui.h>
#include <list>

//message topic headers
#include "riddle/DetectedPersonsList.h"


#include <riddle/FindPersonAction.h>
#include <actionlib/server/simple_action_server.h>

/* Define this variable to generate detection files
 * for ROC curve generation (after post-processing)
 */
//#define IROS_EVAL_SECTION

typedef actionlib::SimpleActionServer<riddle::FindPersonAction> FindPersonServer;
//tf stuff

//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
//#include "Streaming/StreamingApp.h"

string g_data_root_path;
///////////////////////////////////////////////////////////////////////
// for timing the code
///////////////////////////////////////////////////////////////////////
//time_t  user_time_start, user_time_end;
//double  cpu_time_start, cpu_time_end;
// Get RGB Image

int global_frame_counter = 0;
Matrix<int> labeledMap;                     //Added !!!
Vector<double> pos3D_plane;

inline double CPUTime()
{
    struct rusage ruse;
    //    getrusage(RUSAGE_SELF,&ruse);
    getrusage(RUSAGE_THREAD,&ruse);

    return ( ruse.ru_utime.tv_sec + ruse.ru_stime.tv_sec +
             1e-6 * (ruse.ru_utime.tv_usec + ruse.ru_stime.tv_usec) );
}

///////////////////////////////////////////////////////////////////////
// ReadConfigFile:
//      Reads the config file for setting up the global variables.
///////////////////////////////////////////////////////////////////////
void ReadConfigFile(std::string data_root_path)
{

    ConfigFile config(data_root_path + "config_Asus.inp");


    Globals::preload = config.read("preload",false);

    //=====================================
    // Input paths
    //=====================================
    config.readInto(Globals::camPath_left, "camPath_left");
    Globals::camPath_left = data_root_path + Globals::camPath_left;

    config.readInto(Globals::sImagePath_left, "sImagePath_left");

    config.readInto(Globals::tempDepthL, "tempDepthL");
    config.readInto(Globals::path_to_planes, "path_to_planes");

    //=====================================
    // Distance Range Accepted Detections
    //=====================================
    Globals::distance_range_accepted_detections = config.read<double>("distance_range_accepted_detections", 7);

    //======================================
    // ROI
    //======================================
    Globals::inc_width_ratio = config.read<double>("inc_width_ratio");
    Globals::inc_height_ratio = config.read<double>("inc_height_ratio");
    Globals::region_size_threshold = config.read<double>("region_size_threshold", 10);

    Globals::max_height = config.read<double>("max_height", 2.0);
    Globals::min_height = config.read<double>("min_height", 1.4);

    //======================================
    // Freespace Parameters
    //======================================
    Globals::freespace_scaleZ = config.read<double>("freespace_scaleZ", 20);
    Globals::freespace_scaleX = config.read<double>("freespace_scaleX", 20);
    Globals::freespace_minX = config.read<double>("freespace_minX", -20);
    Globals::freespace_minZ = config.read<double>("freespace_minZ", 0);
    Globals::freespace_maxX = config.read<double>("freespace_maxX", 20);
    Globals::freespace_maxZ = config.read<double>("freespace_maxZ", 30);
    Globals::freespace_threshold = config.read<double>("freespace_threshold", 120);
    Globals::freespace_max_depth_to_cons = config.read<int>("freespace_max_depth_to_cons", 20);
    Globals::log_weight = config.read("log_weight", false);
    Globals::freespace_max_height = config.read<double>("freespace_max_height", 2.5);

    //======================================
    // Evaluation Parameters
    //======================================
    Globals::evaluation_NMS_threshold = config.read<double>("evaluation_NMS_threshold",0.4);
    Globals::evaluation_NMS_threshold_LM = config.read<double>("evaluation_NMS_threshold_LM",0.4);
    Globals::evaluation_NMS_threshold_Border = config.read<double>("evaluation_NMS_threshold_Border",0.4);
    Globals::evaluation_inc_height_ratio = config.read<double>("evaluation_inc_height_ratio",0.2);
    Globals::evaluation_stride = config.read<int>("evaluation_stride",3);
    Globals::evaluation_scale_stride = config.read<double>("evaluation_scale_stride",1.03);
    Globals::evaluation_nr_scales = config.read<int>("evaluation_nr_scales",1);
    Globals::evaluation_inc_cropped_height = config.read<int>("evaluation_inc_cropped_height",20);
    Globals::evaluation_greedy_NMS_overlap_threshold = config.read<double>("evaluation_greedy_NMS_overlap_threshold", 0.1);
    Globals::evaluation_greedy_NMS_threshold = config.read<double>("evaluation_greedy_NMS_threshold", 0.25);
    //======================================
    // World scale
    //======================================
    config.readInto(Globals::WORLD_SCALE, "WORLD_SCALE");

    //======================================
    // height and width of images
    //======================================
    Globals::dImHeight = config.read<int>("dImHeight");
    Globals::dImWidth = config.read<int>("dImWidth");

    //======================================
    // Camera
    //======================================
    Globals::baseline = config.read<double>("baseline");

    //====================================
    // Number of Frames / offset
    //====================================
    Globals::numberFrames = config.read<int>("numberFrames");
    Globals::nOffset = config.read<int>("nOffset");

    //======================================
    // Console output
    //======================================
    Globals::verbose = config.read("verbose", false);

    //=====================================
    // Determines if save bounding boxes or not
    //=====================================
    Globals::export_bounding_box = config.read("export_bounding_box", false);
    // Path of exported bounding boxes
    config.readInto(Globals::bounding_box_path, "bounding_box_path");

    //=====================================
    // Determines if save result images or not
    //=====================================
    Globals::export_result_images = config.read("export_result_images", false);
    config.readInto(Globals::result_images_path, "result_images_path");

    //====================================
    // Size of Template
    //====================================
    Globals::template_size = config.read<int>("template_size");


    /////////////////////////////////TRACKING PART/////////////////////////
    //======================================
    // Detections
    //======================================
    Globals::cutDetectionsUsingDepth = config.read("cutDetectionsUsingDepth", false);

    Globals::frameRate = config.read<int>("frameRate");

    //======================================
    // Camera
    //======================================
    Globals::farPlane = config.read<double>("farPlane");

    //======================================
    // World scale
    //======================================
    config.readInto(Globals::binSize, "binSize");

    //======================================
    // Pedestrians width and height
    //======================================
    Globals::pedSizeWVis = config.read<double>("pedSizeWVis");
    Globals::pedSizeWCom = config.read<double>("pedSizeWCom");
    Globals::pedSizeHCom = config.read<double>("pedSizeHCom");

    //======================================
    // History
    //======================================
    Globals::history = config.read<int>("history");

    //======================================
    // Pedestrians parameter
    //======================================
    Globals::dObjHeight = config.read<double>("dObjHeight");
    Globals::dObjHVar = config.read<double>("dObjHVar");

    //======================================
    // Adjustment for HOG detections
    //======================================
    Globals::cutHeightBBOXforColor = config.read<double>("cutHeightBBOXforColor");
    Globals::cutWidthBBOXColor = config.read<double>("cutWidthBBOXColor");
    Globals::posponeCenterBBOXColor = config.read<double>("posponeCenterBBOXColor");

    //======================================
    // Thresholds for combining the detection from left and right camera
    //======================================
    Globals::probHeight = config.read<double>("probHeight");

    //======================================
    // Visualisation
    //======================================
    Globals::render_bbox3D = config.read("render_bbox3D", true);
    Globals::render_bbox2D = config.read("render_bbox2D", false);
    //Globals::render_tracking_numbers = config.read("render_tracking_numbers", false);																//Modified

    /*//======================================																//Modified
    // MDL parameters for trajectories
    //======================================
    Globals::k1 = config.read<double>("k1");																//Modified
    Globals::k2 = config.read<double>("k2");																//Modified
    Globals::k3 = config.read<double>("k3");																//Modified
    Globals::k4 = config.read<double>("k4");																//Modified
	*/

    //======================================
    // Threshold for distinction between static/moving object
    //======================================
    Globals::minvel = config.read<double>("minvel");
    Globals::dMaxPedVel = config.read<double>("dMaxPedVel");

    //======================================
    // Threshold for identity management
    //======================================
    Globals::dSameIdThresh = config.read<double>("dSameIdThresh");

    /*//======================================																//Modified
    // Trajectory
    //======================================
    Globals::threshLengthTraj = config.read<int>("threshLengthTraj");*/

    //======================================
    // Thresholds for accepted and displayed hypotheses
    //======================================
    Globals::dTheta2 = config.read<double>("dTheta2");

    //======================================
    // Time ant for temporal decay
    //======================================
    Globals::dTau = config.read<double>("dTau");

    //======================================
    // Time horizon for event cone search
    //======================================
    Globals::coneTimeHorizon = config.read<int>("coneTimeHorizon");
    Globals::maxHoleLen = config.read<int>("maxHoleLen");
    Globals::dHolePenalty = config.read<double>("dHolePenalty");

    // Q - the system covariance
    Globals::sysUncX = config.read<double>("sysUncX");
    Globals::sysUncY = config.read<double>("sysUncY");
    Globals::sysUncRot = config.read<double>("sysUncRot");
    Globals::sysUncVel = config.read<double>("sysUncVel");
    Globals::sysUncAcc = config.read<double>("sysUncAcc");

    Globals::kalmanObsMotionModelthresh = config.read<double>("kalmanObsMotionModelthresh");																//Modified
    Globals::kalmanObsColorModelthresh = config.read<double>("kalmanObsColorModelthresh");																//Modified

    /////////////////////////////////GP Estimator/////////////////////////
    Globals::nrInter_ransac = config.read<int>("nrInter_ransac");
    Globals::numberOfPoints_reconAsObstacle = config.read<int>("numberOfPoints_reconAsObstacle");

    //======================================
    // ROI Segmentation
    //======================================
    // Blurring parameters
    Globals::sigmaX = config.read<double>("sigmaX", 2.0);
    Globals::precisionX = config.read<double>("precisionX", 2.0);
    Globals::sigmaZ = config.read<double>("sigmaZ", 3.0);
    Globals::precisionZ = config.read<double>("precisionZ", 2.0);

    ///////////////////////////Recording /////////////////////
    Globals::from_camera = config.read("from_camera", true);
    config.readInto(Globals::from_file_path, "from_file_path");
    Globals::from_file_path = data_root_path + Globals::from_file_path;
    //////////////////////////Streaming///////////////////////
    //config.readInto(Globals::stream_dest_IP, "stream_dest_IP");

    /*////////////////////////HOG Detector////////////////////////															//Modified
    Globals::hog_max_scale = config.read<float>("hog_max_scale",1.9);
    Globals::hog_score_thresh = config.read<float>("hog_score_thresh",0.4);

    ///////////////////////Components///////////////////////////
    Globals::use_hog = config.read("use_hog", false);*/
    Globals::use_segmentation_roi = config.read("use_segmentation_roi", false);
    Globals::use_local_max = config.read("use_local_max", true);
}

void PrintUsage( const std::string &sProgName)
{
    std::cout << "\nUsage:  " << sProgName << " <options>";
    std::cout << "\n---------------------------------------";
    std::cout << "\n   -c                <name>          (config_file_name)";
    std::cout << "\n" << std::endl;
}

///////////////////////////////////////////////////////////////////////
// ProcessCommandArgs:
//      Processes command line arguments.
///////////////////////////////////////////////////////////////////////
//void ProcessCommandArgs(int argc, char **argv)
//{
//    for( int i=0; i<argc; ++i )
//    {
//        if( argv[i][0]=='-' )
//        {
//            if( (strcmp(argv[i],"-help")==0) || (strcmp(argv[i],"--help")==0)
//                    || (strcmp(argv[i],"-h")==0) || (strcmp(argv[i],"-?")==0) )
//            {
//                PrintUsage(argv[0]);
//                exit(0);
//            }
//            else if( strcmp(argv[i],"-c")==0 )
//            {
//                if( argc>(i+1) )
//                    path_config_file = argv[++i];
//            }
//        }
//    }
//}

///////////////////////////////////////////////////////////////////////
// ReadUpperBodyTemplate:
//      Reads template of upper body from file and resizes it to
//      Global::template_size which is determined in ConfigFile.
//
// parameters:
//      output:
//          upper_body_template -   Template of upper body.
///////////////////////////////////////////////////////////////////////
void ReadUpperBodyTemplate(Matrix<double>& upper_body_template)
{
    // read template from file
    string upper_fname = g_data_root_path + "upper_temp_n.txt";
    upper_body_template.ReadFromTXT(upper_fname, 150, 150);

    // resize it to the fixed size that is defined in Config File
    if(upper_body_template.x_size() > Globals::template_size)
    {
        upper_body_template.DownSample(Globals::template_size, Globals::template_size);
    }
    else if(upper_body_template.x_size() < Globals::template_size)
    {
        upper_body_template.UpSample(Globals::template_size, Globals::template_size);
    }
}

void RenderBBox2D(const Vector<double>& bbox, CImg<unsigned char>& image, int r, int g, int b)
{
    int x =(int) bbox(0);
    int y =(int) bbox(1);
    int w =(int) bbox(2);
    int h =(int) bbox(3);

    const unsigned char color[] = {r,g,b};
    image.draw_rectangle_1(x,y,x+w,y+h,color,3);
}

enum DETECTOR_MODE
{
    DEPTH_DETECTOR,
    DEPTH_LM_DETECTOR
} detector_mode;

class DetectionTrackingSystem
{
public:

    int computeOrientation(Vector<int>& x_distribution, Vector<int>& dist_distribution, double& dispersionXY)
    {
        if (x_distribution.getSize() != dist_distribution.getSize() ||x_distribution.getSize()==0)
        {
            std::cout << "Error in vector sizes - computeOrientation function" << std::endl;
			return -1;
        }

        // find barycenter in x and y
        double barycenter_x = x_distribution.sum();;
        barycenter_x = barycenter_x/x_distribution.getSize();
        // arithmetical mean

		double barycenter_y = dist_distribution.sum();
        barycenter_y = barycenter_y/dist_distribution.getSize();

        // calculate mu_11, mu_20 and mu_02 (phi = arctan(2mu_11/(mu_20 - mu_02))/2)
        // mu_pq = SUM[y=0..L-1]SUM[x=0..C-1](x-x_bar)^p*(y-y_bar)^q

        double mu_11 = 0;
        double mu_20 = 0;
        double mu_02 = 0;

        for (int i = 0; i< int(x_distribution.getSize()); ++i)
        {
            mu_11 += pow(x_distribution(i)-barycenter_x,1.0)*pow(dist_distribution(i)-barycenter_y,1.0);
            mu_20 += pow(x_distribution(i)-barycenter_x,2.0);
            mu_02 += pow(dist_distribution(i)-barycenter_y,2.0);
        }

        dispersionXY = mu_20/mu_02;
        double resultat = atan(2*mu_11/(mu_20-mu_02));

        resultat = 0.5*resultat * 180/3.14159265;
//        resultat = resultat * 180/3.14159265;
//        resultat = int(resultat) % 360;

        return resultat;

    }

    void correctAngle(double& angle, double& ratio_disp)
    {
        int poly_coeffs[3] = {3.3744, -0.0004, -0.0009}; //polynomial coeeficients (from reference least square given by matlab)

        double reference_value = poly_coeffs[2]*pow(angle,2) + poly_coeffs[1]*angle + poly_coeffs[0]; //compute the reference value corresponding to the detected angle
        if(ratio_disp< reference_value)
        {
            double disparity = 1 - (ratio_disp/reference_value);
            double correction_factor = ((exp(disparity)-1)*1.16)*(0.5*log(2*disparity)+0.7);
            std::cout << "correction_factor : " << correction_factor << std::endl;
            int8_t signe;
            if(angle <0)
                signe = -1;
            else
                signe = +1;

            angle += signe*45*correction_factor;
        }
    }

    void save_features(const Vector<Vector< double > >& detected_Bbox, const Vector<Vector<double> >& x_point_cloud_distribution,const Vector<Vector<double> >& y_point_cloud_distribution,
                        const Vector<Vector<double> >& x_point_cloud_distribution_full,const Vector<Vector<double> >& y_point_cloud_distribution_full,const Matrix<double>& depth_map, const PointCloud& point_cloud)
    {
    //initialise needed elements
        // File to save bounding box coorinates
        std::ofstream myfile;
        myfile.open("bounding_boxes", std::ios::app);

        //For occupancy grid
        const unsigned char color[] = {0,255,0};
        double scale_z_ = Globals::freespace_scaleZ;
        double scale_x_ = Globals::freespace_scaleX;
        double min_x_ = Globals::freespace_minX;
        double min_z_ = Globals::freespace_minZ;
        double max_x_ = Globals::freespace_maxX;
        double max_z_ = Globals::freespace_maxZ;
        int x_bins = (int)round((max_x_ - min_x_)*scale_x_)+1;
        int z_bins = (int)round((max_z_ - min_z_)*scale_z_)+1;
        double step_x = (max_x_ - min_x_)/double(x_bins-1);
        double step_z = (max_z_ - min_z_)/double(z_bins-1);

        int width = depth_map.x_size();

        //for each detection :
        std::cout << "process each detection..." << std::endl;
        for(int i=0; i< detected_Bbox.getSize(); ++i)
        {
            char str[10], str_pcd[10];
            std::cout << "global_frame_counter : " << global_frame_counter <<  std::endl;
            sprintf(str, "image_%d.txt", global_frame_counter);
            sprintf(str_pcd, "image_%d.pcd", global_frame_counter);
            //needed containers
            Vector<int> vect_pos_x;
            Vector<int> vect_pos_z;
            //Creating pcl derived point cloud
            pcl::PointCloud<pcl::PointXYZ> cloud;
            cloud.width = detected_Bbox(i)(2);
            cloud.height = detected_Bbox(i)(3);
            cloud.is_dense=false;
            cloud.points.resize(cloud.width*cloud.height);

            //fill initial pcl point cloud

        //Save coordinates Bbox
            if(myfile.is_open())
            {
            //myfile << "#x y" << std::endl;
                myfile << "frame: " << global_frame_counter << ";" << detected_Bbox(i)(0) << ' ' << detected_Bbox(i)(1) << ' ' << detected_Bbox(i)(2) << ' ' << detected_Bbox(i)(3) << std::endl;
                //myfile.close();
            }
            else std::cout << "Unable to open file" << std::endl;

        //Project to ground plane (occupancy grid)
            //Matrix<double> occupancy;
            //occupancy.set_size(x_bins, z_bins);
            //occupancy.fill(0);

            //Create a PCL point cloud that contins all the detected person
            for(int vector_size = 0; vector_size < (x_point_cloud_distribution_full(i)).getSize(); ++vector_size)
            {
                int element_place =  (y_point_cloud_distribution_full(i)(vector_size)*width) + x_point_cloud_distribution_full(i)(vector_size);

                //fill in pcl point cloud
                int x_inBbox = x_point_cloud_distribution_full(i)(vector_size) - detected_Bbox(i)(0);
                int y_inBbox = y_point_cloud_distribution_full(i)(vector_size) - detected_Bbox(i)(1);
                int element_place_pcl = (y_inBbox*cloud.width) + x_inBbox;
                cloud.points[element_place_pcl].x = point_cloud.X(element_place);
                cloud.points[element_place_pcl].y = point_cloud.Y(element_place);
                cloud.points[element_place_pcl].z = point_cloud.Z(element_place);

                //if(pos_x>0 && pos_x < x_bins && pos_z > 0 && pos_z < z_bins)
                    //occupancy(pos_x, pos_z) = 255;
            }

            //Use cropped version of point cloud to project on the ground
            for(int vector_size = 0; vector_size < (x_point_cloud_distribution(i)).getSize(); ++vector_size)
            {
                int element_place =  (y_point_cloud_distribution(i)(vector_size)*width) + x_point_cloud_distribution(i)(vector_size);
                double zj = point_cloud.Z(element_place);
                double xj = point_cloud.X(element_place);
                double yj = point_cloud.Y(element_place);

                double x = xj - min_x_;
                double z = zj - min_z_;

                int pos_x  =(int)round(x/step_x);
                int pos_z  =(int)round(z/step_z);

                vect_pos_x.pushBack(pos_x);
                vect_pos_z.pushBack(pos_z);
            }

            //Write occupancy matrix to file
            //Create pairs and pushback in list
            std::list<pair<int,int> > list1;
            std::pair<int,int> foo;

            for(int i =0; i<vect_pos_x.getSize(); ++i)
            {
                 foo = std::make_pair(vect_pos_x(i), vect_pos_z(i));
                 list1.push_back(foo);
            }
            list1.sort();
            list1.unique();

            //search for min_x, max_x, min_y and max_y
            int min_x=1000;
            int min_y=1000;
            int max_x=0;
            int max_y=0;
            for(list<pair<int,int> >::iterator it=list1.begin();it!=list1.end();++it)
            {
                min_x = ((*it).first < min_x)?(*it).first:min_x;
                min_y = ((*it).second < min_y)?(*it).second:min_y;
                max_x = ((*it).first > max_x)?(*it).first:max_x;
                max_y = ((*it).second > max_y)?(*it).second:max_y;
                  //std::cout << "(" << (*it).first << "," << (*it).second << ")" << std::endl;
            }

            //offset on all elements of list
            for(list<pair<int,int> >::iterator it=list1.begin();it!=list1.end();++it)
            {
                (*it).first -= min_x;
                (*it).second -= min_y;
                  //std::cout << "(" << (*it).first << "," << (*it).second << ")" << std::endl;
            }

            Matrix<int> m1(max_x-min_x+1,max_y-min_y+1);
            m1.fill(0);

            for(list<pair<int,int> >::iterator it=list1.begin();it!=list1.end();++it)
            {
                  m1((*it).first, (*it).second) = 1;
            }

            m1.WriteToTXTApp(str, 1);
        //save pcd
            pcl::io::savePCDFileASCII(str_pcd,cloud);
        }
    }


    void get_image(unsigned char* b_image, uint w, uint h, CImg<unsigned char>& cim)
    {
        unsigned char* ptr = b_image;
        for (unsigned int row = 0; row < h; ++row)
        {
            for (unsigned int col = 0; col < w; ++col)
            {
                // access the viewerImage as column, row
                cim(col,row,0,0) = *(ptr++); // red component
                cim(col,row,0,1) = *(ptr++); // green
                cim(col,row,0,2) = *(ptr++); // blue
            }
        }
        display.display(cim);                                                                              //Modified
    }

    void get_depth(const Matrix<double>& depth_map, uint w, uint h, CImg<unsigned char>& cim, PointCloud pc, Vector<double> gp)
    {
        double min=depth_map.data()[0];
        double max=min;
        for(uint i=1; i<w*h; ++i)
        {
            double value = depth_map.data()[i];
            if(value > max)
                max=value;
            if(value < min)
                min=value;
        }

        int i=0;
        for (unsigned int row = 0; row < h; ++row)
        {
            for (unsigned int col = 0; col < w; ++col)
            {
                unsigned char value = (unsigned char)((depth_map(col,row)-min)*255/(max-min));
                double d = fabs(pc.X(i)*gp(0)+pc.Y(i)*gp(1)+pc.Z(i)*gp(2)+gp(3));
                if(d>0.02)
                {
                    // access the viewerImage as column, row
                    cim(col,row,0,0) = value; // red component
                    cim(col,row,0,1) = value; // green
                    cim(col,row,0,2) = value; // blue
                }
                else
                {
                    // access the viewerImage as column, row
                    cim(col,row,0,0) = 0; // red component
                    cim(col,row,0,1) = 0; // green
                    cim(col,row,0,2) = value; // blue
                }
                ++i;
            }
        }
    }

    void get_depth(const Matrix<double>& depth_map, uint w, uint h, CImg<unsigned char>& cim, PointCloud pc, Vector<double> gp, Vector<double> pos_on_plane) //Added
    {
        double min=depth_map.data()[0];
        double max=min;
        for(uint i=1; i<w*h; ++i)
        {
            double value = depth_map.data()[i];
            if(value > max)
                max=value;
            if(value < min)
                min=value;
        }

        int i=0;
        for (unsigned int row = 0; row < h; ++row)
        {
            for (unsigned int col = 0; col < w; ++col)
            {
                unsigned char value = (unsigned char)((depth_map(col,row)-min)*255/(max-min));
                double d = fabs(pc.X(i)*gp(0)+pc.Y(i)*gp(1)+pc.Z(i)*gp(2)+gp(3));
                double dp;
                if(pos_on_plane.getSize() != 0)
                {
                    //std::cout << "size gp :" << gp.getSize() << ", size pos_screen : " << pos_on_plane.getSize() << std::endl;
                    //dp = fabs(pc.X(i)*pos_on_plane(0)+pc.Y(i)*pos_on_plane(1)+pc.Z(i)*pos_on_plane(2)+pos_on_plane(3));
                    dp = fabs(pc.X(i)*pos_on_plane(0)+pc.Y(i)*pos_on_plane(1));
                    //std::cout << "dp : " << dp << std::endl;
                }
                else
                    dp = 100;

                if(d>0.1)
                {
                    // access the viewerImage as column, row
                    cim(col,row,0,0) = value; // red component
                    cim(col,row,0,1) = value; // green
                    cim(col,row,0,2) = value; // blue
                }
                else if (dp < 0.2)
                {
                    // access the viewerImage as column, row
                    cim(col,row,0,0) = 0; // red component
                    cim(col,row,0,1) = value; // green
                    cim(col,row,0,2) = 0; // blue
                }
                else
                {
                    // access the viewerImage as column, row
                    cim(col,row,0,0) = 0; // red component
                    cim(col,row,0,1) = 0; // green
                    cim(col,row,0,2) = value; // blue
                }
                ++i;
            }
        }
    }

    void draw_roi(const Matrix<int>& roi_mat, uint w, uint h, CImg<unsigned char>& cim)
    {
        if(roi_mat.x_size()<w || roi_mat.y_size()<h) return;

        unsigned char color_array[][3] = {   {0  ,     0,     0},
                                             {204,     0,   255},
                                             {255,     0,     0},
                                             {0,   178,   255},
                                             {255,     0,   191},
                                             {255,   229,     0},
                                             {0,   255,   102},
                                             {89,   255,     0},
                                             {128,     0,   255},
                                             {242,     0,   255},
                                             {242,   255,     0},
                                             {255,     0,    77},
                                             {51,     0,   255},
                                             {0,   255,   140},
                                             {0,   255,    25},
                                             {204,   255,     0},
                                             {255,   191,     0},
                                             {89,     0,   255},
                                             {0,   217,   255},
                                             {0,    64,   255},
                                             {255,   115,     0},
                                             {255,     0,   115},
                                             {166,     0,   255},
                                             {13,     0,   255},
                                             {0,    25,   255},
                                             {0,   255,   217},
                                             {0,   255,    64},
                                             {255,    38,     0},
                                             {255,     0,   153},
                                             {0,   140,   255},
                                             {255,    77,     0},
                                             {255,   153,     0},
                                             {0,   255,   179},
                                             {0,   102,   255},
                                             {255,     0,    38},
                                             {13,   255,     0},
                                             {166,   255,     0},
                                             {0,   255,   255},
                                             {128,   255,     0},
                                             {255,     0,   230},
                                             {51,   255,     0}
                                         };
        int ind;
        for (unsigned int row = 0; row < h; ++row)
        {
            for (unsigned int col = 0; col < w; ++col)
            {
                ind = roi_mat(col,row);
                cim(col,row,0,0) = color_array[ind][0]; // red component
                cim(col,row,0,1) = color_array[ind][1]; // green
                cim(col,row,0,2) = color_array[ind][2]; // blue
            }
        }
    }

    void draw_hist(const Matrix<int>& hist_mat, uint w, uint h, CImg<unsigned char>& cim)
    {
        if(hist_mat.x_size()<w || hist_mat.y_size()<h) return;

        unsigned char color_array[][3] = {   {0  ,     0,     0},
                                             {204,     0,   255},
                                             {255,     0,     0},
                                             {0,   178,   255},
                                             {255,     0,   191},
                                             {255,   229,     0},
                                             {0,   255,   102},
                                             {89,   255,     0},
                                             {128,     0,   255},
                                             {242,     0,   255},
                                             {242,   255,     0},
                                             {255,     0,    77},
                                             {51,     0,   255},
                                             {0,   255,   140},
                                             {0,   255,    25},
                                             {204,   255,     0},
                                             {255,   191,     0},
                                             {89,     0,   255},
                                             {0,   217,   255},
                                             {0,    64,   255},
                                             {255,   115,     0},
                                             {255,     0,   115},
                                             {166,     0,   255},
                                             {13,     0,   255},
                                             {0,    25,   255},
                                             {0,   255,   217},
                                             {0,   255,    64},
                                             {255,    38,     0},
                                             {255,     0,   153},
                                             {0,   140,   255},
                                             {255,    77,     0},
                                             {255,   153,     0},
                                             {0,   255,   179},
                                             {0,   102,   255},
                                             {255,     0,    38},
                                             {13,   255,     0},
                                             {166,   255,     0},
                                             {0,   255,   255},
                                             {128,   255,     0},
                                             {255,     0,   230},
                                             {51,   255,     0}
                                         };
        int ind, x_size = hist_mat.x_size()-100;
        for (unsigned int row = 0; row < h; ++row)
        {
            for (unsigned int col = 0; col < w; ++col)
            {
                ind = hist_mat(x_size-col,row);
                cim(col,row,0,0) = color_array[ind][0]; // red component
                cim(col,row,0,1) = color_array[ind][1]; // green
                cim(col,row,0,2) = color_array[ind][2]; // blue
            }
        }
    }

    void draw_hist(const Matrix<double>& hist_mat, uint w, uint h, CImg<unsigned char>& cim)
    {
        if(hist_mat.x_size()<w || hist_mat.y_size()<h) return;

        int ind;//, x_size = hist_mat.x_size()-100, y_size = hist_mat.y_size()-110;
        double max = hist_mat(0,0), min = max;
        for(int i=1;i<hist_mat.total_size();++i)
        {
            if(max<hist_mat.data()[i]) max = hist_mat.data()[i];
            if(min>hist_mat.data()[i]) min = hist_mat.data()[i];
        }
        max-=min;
        int w1=200, h1=200;
        CImg<unsigned char> cim1(w1,h1,1,3);
        for (unsigned int row = 0; row < h1; ++row)
        {
            for (unsigned int col = 0; col < w1; ++col)
            {
                ind = (int)((hist_mat(col+300,row)-min)*255/max);
                cim1(col,row,0,0) = ind;//color_array[ind][0]; // red component
                cim1(col,row,0,1) = ind;//color_array[ind][1]; // green
                cim1(col,row,0,2) = ind;//color_array[ind][2]; // blue
            }
        }
        cim1.resize(cim);
        cim.draw_image(cim1);
    }

    void vis_gp(CImg<unsigned char>& cim, const Camera& camera)
    {
        double lx = -1.0, ux = 1.0;
        double lz = 0.5, uz = 8.0;
        double z2 = uz/4, z3 = uz/2, z4 = uz/4*3;
        double X[]={lx, lx, 0., 0., ux, ux, lx, ux, lx, ux, lx, ux, lx, ux};
        double Z[]={lz, uz, lz, uz, lz, uz, uz, uz, z4, z4, z3, z3,  z2,  z2};
        double c20 = camera.K()(2,0);
        double c00 = camera.K()(0,0);
        double c21 = camera.K()(2,1);
        double c11 = camera.K()(1,1);

        unsigned char color[3] = {255,255,0};
        char dis[100];
        for(int i=0; i<14; ++i)
        {
            double Y = (-last_gp(3)-last_gp(0)*X[i]-last_gp(2)*Z[i])/last_gp(1);
            int x1 = X[i]*c00/Z[i]+c20;
            int y1 = Y*c11/Z[i]+c21;
            ++i;
            Y = (-last_gp(3)-last_gp(0)*X[i]-last_gp(2)*Z[i])/last_gp(1);
            int x2 = X[i]*c00/Z[i]+c20;
            int y2 = Y*c11/Z[i]+c21;
            if(i>6)
            {
                sprintf(dis,"%0.1f",Z[i]);
                cim.draw_text(335,y2-21,dis,color,0,1,20);
            }
            cim.draw_line(x1,y1,x2,y2,color);
        }
    }

    double getOrientation(vector<cv::Point> &pts, cv::Mat &img)
    {
        //Construct a buffer used by the pca analysis
        cv::Mat data_pts = cv::Mat(pts.size(), 2, CV_64FC1);
        for (int i = 0; i < data_pts.rows; ++i)
        {
            data_pts.at<double>(i, 0) = pts[i].x;
            data_pts.at<double>(i, 1) = pts[i].y;
        }

        //Perform PCA analysis
        cv::PCA pca_analysis(data_pts, cv::Mat(), CV_PCA_DATA_AS_ROW);

        //Store the position of the object
        cv::Point pos = cv::Point(pca_analysis.mean.at<double>(0, 0), pca_analysis.mean.at<double>(0, 1));

        //Store the eigenvalues and eigenvectors
        vector<cv::Point2d> eigen_vecs(2);
        vector<double> eigen_val(2);
        for (int i = 0; i < 2; ++i)
        {
            eigen_vecs[i] = cv::Point2d(pca_analysis.eigenvectors.at<double>(i, 0), pca_analysis.eigenvectors.at<double>(i, 1));

            eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
        }

        //Muliply vectors by a scalar (better visualization)
        double scalar = 10;

        // Draw the principal components
        cv::circle(img, pos, 3, CV_RGB(255, 0, 255), 2);
        cv::line(img, pos, pos + scalar*0.02 * cv::Point(eigen_vecs[0].x * eigen_val[0], eigen_vecs[0].y * eigen_val[0]) , CV_RGB(255, 255, 0));
        cv::line(img, pos, pos + scalar*0.02 * cv::Point(eigen_vecs[1].x * eigen_val[1], eigen_vecs[1].y * eigen_val[1]) , CV_RGB(0, 255, 255));

//        return atan2(eigen_vecs[0].y, eigen_vecs[0].x)*180/3.14159265;
        double distance_line1 = sqrt(eigen_val[0]);
        double distance_line2 = sqrt(eigen_val[1]) ;
        //return rate distance_line1/distance_line2
        return distance_line1/distance_line2;

    }

    std::string isometryToString(const Eigen::Isometry3d& m)
    {
        char result[80];
        memset(result, 0, sizeof(result));
        Eigen::Vector3d xyz = m.translation();
        Eigen::Vector3d rpy = m.rotation().eulerAngles(0, 1, 2);
        snprintf(result, 79, "%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f",
                 xyz(0), xyz(1), xyz(2),
                 rpy(0) * 180/M_PI, rpy(1) * 180/M_PI, rpy(2) * 180/M_PI);
        return std::string(result);
    }

    /*struct HogParams															                                //Modified
    {
        HogParams(DetectionTrackingSystem* obj, int f, unsigned char* img, Camera cam, Vector<Vector<double> >* detected_bb, Vector<Vector < double > >* OutputHOGdet)
        {
            _this = obj;
            frame = f;
            image = img;
            camera = cam;
            detected_bounding_boxes = detected_bb;
            OutputHOGdetL = OutputHOGdet;
        }

        DetectionTrackingSystem* _this;
        int frame;
        unsigned char* image;
        Camera camera;
        Vector<Vector<double> >* detected_bounding_boxes;
        Vector<Vector < double > >* OutputHOGdetL;
    };*/

    struct UpperParams
    {
        UpperParams(DetectionTrackingSystem* obj,Camera cam, Matrix<double>* dp, PointCloud* pc,Vector<Vector<double> >* detected_bb)
        {
            _this = obj;
            camera = cam;
            depth_map = dp;
            point_cloud = pc;
            detected_bounding_boxes = detected_bb;
        }

        DetectionTrackingSystem* _this;
        Camera camera;
        Matrix<double>* depth_map;
        PointCloud* point_cloud;
        Vector<Vector<double> >* detected_bounding_boxes;
    };

#ifdef USE_HOG
    static void* hog_thread_function(void* params)
    {
        HogParams* hog_params = (struct HogParams*) params;
        *(hog_params->OutputHOGdetL) = hog_params->_this->hog_detector.runHog(hog_params->frame,hog_params->image,hog_params->camera, *(hog_params->detected_bounding_boxes));
        pthread_exit(NULL);
    }
#endif

    static void* upper_thread_function(void* params)
    {
        std::cout<< "entering upper_thread_function" << std::endl;
        UpperParams* upper_params = (struct UpperParams*) params;
        if(upper_params->_this->is_seg)
        {
            upper_params->_this->detector_seg->ProcessFrame(upper_params->camera, *(upper_params->depth_map), *(upper_params->point_cloud), upper_params->_this->upper_body_template, *(upper_params->detected_bounding_boxes));
        }
        else
        {
            upper_params->_this->detector->ProcessFrame(upper_params->camera, *(upper_params->depth_map), *(upper_params->point_cloud), upper_params->_this->upper_body_template, *(upper_params->detected_bounding_boxes));
        }
        pthread_exit(NULL);
    }

    int gp_count_down;
    int motion_not_valid_count;

    void main_process(unsigned char* b_image, float* b_depth, uint w, uint h)
    {
        cpu_time_start = CPUTime();
        double ct, fovis_time, PC_time, GP_time, detection_time;             //, tracking_time;																//Modified

        // Construct depth_map Matrix
        Matrix<double> depth_map(w,h);
        uint total_pixels = w*h;
        for(uint i=0;i<total_pixels;++i)
        {
            double v =(double)b_depth[i];
            if(isnan(v))
                depth_map.data()[i]=0.0;
            else
                depth_map.data()[i]=v;
        }

        ////////////////// FOVIS /////////////////////////////////////////////////////////////////
        ct=CPUTime();

        if(is_first)
        {
            fovis_rect = new fovis::Rectification(cam_params);
            fovis::VisualOdometryOptions options = fovis::VisualOdometry::getDefaultOptions();
            odom = new fovis::VisualOdometry(fovis_rect, options);
            is_first=false;

            lodx=320;
            lodz=240;
            unsigned char cl[]={255,255,255};
            odimg.draw_circle((int)lodx,(int)lodz,2,cl);
        }
        if(!is_odom_valid)
        {
            fovis::VisualOdometryOptions options = fovis::VisualOdometry::getDefaultOptions();
            delete odom;
            odom = new fovis::VisualOdometry(fovis_rect, options);

            lodx=320;
            lodz=240;
            odimg = CImg<unsigned char>(640,480,1,3,0);
            unsigned char cl[]={255,255,255};
            odimg.draw_circle((int)lodx,(int)lodz,2,cl);
        }

        fovis::DepthImage* fv_dp = new fovis::DepthImage(cam_params,w,h);
        fv_dp->setDepthImage(b_depth);

        //Computing Gray Image
        unsigned char* b_gray_image = new unsigned char[total_pixels];
        unsigned char* ptr_rgb = b_image;
        for(int i=0; i<total_pixels;++i)
        {
            b_gray_image[i] = (unsigned char)round(0.2125 * *(ptr_rgb++) +
                                                   0.7154 * *(ptr_rgb++) +
                                                   0.0721 * *(ptr_rgb++));
            //            b_gray_image[i] = (unsigned char)((*(ptr_rgb++)+*(ptr_rgb++)+*(ptr_rgb++))/3);
        }

        odom->processFrame(b_gray_image, fv_dp);

        if(!odom->getMotionEstimator()->isMotionEstimateValid())
        {
            gp_count_down = 10;
            dp_queue.push(b_depth);
            img_queue.push(b_image);
            delete fv_dp;
            is_last_gp_valid=false;
            is_odom_valid = true;
            if(motion_not_valid_count>10)
            {
                motion_not_valid_count = 0;
                is_odom_valid = false;
            }
            ++motion_not_valid_count;
            ROS_INFO("-----------------motion is not valid. [ %d ]", motion_not_valid_count);
            return;
        }

        Eigen::Isometry3d cam_to_local = odom->getPose();
        Eigen::Isometry3d motion =  odom->getMotionEstimate();

        Eigen::Matrix4d m1 = cam_to_local.matrix().inverse();
        Eigen::Matrix4d m2 = motion.matrix().inverse();
        mm = Matrix<double>(4,4,m1.data());
        Matrix<double> motion_matrix(4,4,m2.data());

        is_odom_valid = true;
        for(int i = 0; i<mm.total_size(); ++i)
        {
            if(isnan(mm.data()[i]))
            {
                is_odom_valid = false;
                break;
            }
        }

        //if(!is_odom_valid)
        if(!odom->getMotionEstimator()->isMotionEstimateValid() || !is_odom_valid)
        {
            gp_count_down = 10;
            //            delete[] b_depth;
            dp_queue.push(b_depth);
            //            delete[] b_image;
            img_queue.push(b_image);
            delete fv_dp;
            is_last_gp_valid=false;
            ROS_INFO("-----------------odom is not valid.--------------------------------------");
            return;
        }

        ///////Just for visualising odometry///////////////////////////
        unsigned char cl[]={255,0,255};
        double nodx = lodx+motion_matrix(0,3)*100,
                nodz = lodz+motion_matrix(2,3)*100;
        odimg.draw_line_1((int)lodx,(int)lodz,(int)nodx,(int)nodz,cl,1);
        lodx=nodx; lodz=nodz;
        //////////////////////////////////////////////////////////////////

        Matrix<double> R(mm, 0,2,0,2);
        Vector<double> t(mm(3,0), mm(3,1), mm(3,2));

        fovis_time = CPUTime() - ct;
        ///////////////////////////////////////////////////////////////////////////////////////////////////


        /////////////////////// Point Cloud ///////////////////////////////////
        ct = CPUTime();
        PointCloud point_cloud(base_camera, depth_map);
        PC_time = CPUTime()-ct;
        //////////////////////////////////////////////////////////////////////


        //////////////////////// GP Estimation //////////////////////////////
        ct = CPUTime();

        if(display.is_keyG() || !is_last_gp_valid || gp_count_down>0)// ||rx>1 || ry>1 || rz>1 ||/*fabs(motion_xyz(0))>0.01 || */fabs(motion_xyz(1))>0.01 /*|| fabs(motion_xyz(2))>0.01*/)
        {
            //cout<<"gp estimation "<<gp_count_down<<endl;
            --gp_count_down;
            last_gp = GPEstimator.ComputeGroundPlane(point_cloud);
            is_last_gp_valid = true;
        }
        else
        {
            Matrix<double> rotate(motion_matrix, 0,2,0,2);
            Vector<double> t(motion_matrix(0,3), motion_matrix(1,3), motion_matrix(2,3));
            Vector<double> pv(last_gp(0), last_gp(1), last_gp(2));
            rotate.Transpose();
            pv = rotate * pv;

            double d = last_gp(3) - DotProduct(pv, t);

            last_gp(0) = pv(0)/pv.norm();
            last_gp(1) = pv(1)/pv.norm();
            last_gp(2) = pv(2)/pv.norm();
            last_gp(3) = d;
        }

        Camera camera(base_camera.K(), R, t, last_gp);
        Vector<double> gp1 = AncillaryMethods::PlaneToWorld(camera, last_gp);
        camera = Camera(camera.K(), R, t, gp1);
        GP_time = CPUTime()-ct;
        ////////////////////////////////////////////////////////////////////
        ////////////////////////// Visualization Part I/////////////////////
        CImg<unsigned char> cim_tmp(w,h,1,3);
        int i = 0;
//        pthread_t run_thread;
//        pthread_create(&run_thread,NULL,run,NULL);                                                                                          //Modified (void *)&run_thread
        //run();

/////////////////////////////////CHOOSE DISPLAY MODE ///////////////////////////////////////////////:
         //Change the display mode here (need to implement threads later...)
        display_mode = ROI_MODE;
        detector_seg->visualize_roi=true;
        detector->visualize_roi=true;

//        display_mode = DEPTH_MODE;
//        detector_seg->visualize_roi=false;
//        detector->visualize_roi=false;
/////////////////////////////////////////////////////////////////////////////////////////////////////

        CImg<unsigned char> cim_final(w,h,1,3), cim_labeledROI(w,h,1,3);
        get_image(b_image,w,h,cim_final);                                                       // Added
        //display_labeledROIs.display(cim_final);                                                                  //ADDED
        display_labeledROIs.set_title("display_labeledROI");                                                        //ADDED !

        if(display_mode == IMAGE_MODE)
        {
            cim_final.draw_image(cim_final);
        }
        else if(display_mode == DEPTH_MODE)
        {
            get_depth(depth_map,w,h,cim_final,point_cloud,last_gp);
            //get_depth(depth_map,w,h,cim_final,point_cloud,last_gp,pos_screen);
        }
        else if (display_mode == ODOM_MODE)
        {
            cim_final.draw_image(odimg);
            unsigned char cl[]={0,255,0};
            cim_final.draw_circle((int)lodx,(int)lodz,2,cl);
        }

        /////////////////////////Detection////////////////////////////////////////////
        Vector<Vector< double > > detected_bounding_boxes1;
        /*Vector<Vector < double > > OutputHOGdetL;*/															//Modified
        //        cout<<"--------------------------------------"<<endl;
        ct=CPUTime();
        pthread_t tupp; /*pthread_t thog,tupp;*/															                    //Modified

#ifdef USE_HOG
        if(use_HOG)
        {
            OutputHOGdetL = hog_detector.runHog(cnt,cim_tmp.get_permute_axes("cxyz").data(),camera, detected_bounding_boxes1);
            //            HogParams hog_params(this,cnt,cim.get_permute_axes("cxyz").data(),camera, &detected_bounding_boxes1, &OutputHOGdetL);
            //            pthread_create(&thog,NULL,SimpleOpenNIViewer::hog_thread_function,(void*)&hog_params);
        }
#endif

        Vector<Vector< double > > detected_bounding_boxes;

        UpperParams upper_params(this ,camera, &depth_map, &point_cloud, &detected_bounding_boxes);
        pthread_create(&tupp,NULL,DetectionTrackingSystem::upper_thread_function,(void*)&upper_params);

        //        pthread_join(thog, NULL);
        pthread_join(tupp, NULL);

        Vector<double> oneDet(9);
        for(int j = 0; j < detected_bounding_boxes.getSize(); ++j)
        {
            oneDet(0) = cnt;
            oneDet(1) = j;
            oneDet(2) = 1;
            oneDet(3) = 1 - detected_bounding_boxes(j)(4)+1; // make sure that the score is always positive
            oneDet(4) = detected_bounding_boxes(j)(0);
            oneDet(5) = detected_bounding_boxes(j)(1);
            oneDet(6) = detected_bounding_boxes(j)(2);
            oneDet(7) = detected_bounding_boxes(j)(3) * 3;
            oneDet(8) = detected_bounding_boxes(j)(5);  //mean distance
            //OutputHOGdetL.pushBack(oneDet);															//Modified
        }
        detected_bounding_boxes.append(detected_bounding_boxes1);

        //detected bounding box

        cv::namedWindow( "Display window", CV_WINDOW_NORMAL );// Create a window for display. OpenCV                 // Added
        //cv::namedWindow( "Display window 2", CV_WINDOW_NORMAL );// Create a window for display. OpenCV                 // Added

        save_features(detected_bounding_boxes, detector_seg->x_distribution, detector_seg->y_distribution, detector_seg->x_distribution_full, detector_seg->y_distribution_full, depth_map, point_cloud);

        ///Attempt 4 - project detected pc from depth_map
        Matrix<double> objects_projected;
        bool get_orientation = true;
        if(get_orientation)
        {
            cim_labeledROI.fill(0);
            if (detected_bounding_boxes.getSize() != 0)
            {
                const unsigned char color[] = {0,255,0};
                double scale_z_ = Globals::freespace_scaleZ;
                double scale_x_ = Globals::freespace_scaleX;
                double min_x_ = Globals::freespace_minX;
                double min_z_ = Globals::freespace_minZ;
                double max_x_ = Globals::freespace_maxX;
                double max_z_ = Globals::freespace_maxZ;

                int x_bins = (int)round((max_x_ - min_x_)*scale_x_)+1;
                int z_bins = (int)round((max_z_ - min_z_)*scale_z_)+1;

                objects_projected.set_size(x_bins, z_bins);
                objects_projected.fill(0);

                double step_x = (max_x_ - min_x_)/double(x_bins-1);
                double step_z = (max_z_ - min_z_)/double(z_bins-1);

                int width = depth_map.x_size();
                Vector<Vector<double> > vect_pos_angle;
                Vector<double> vect_ratioXY;
                double ratioXY=0;

                vect_pos_angle.clearContent();
                vect_ratioXY.clearContent();

                for(int num_elements=0; num_elements< detector_seg->x_distribution.getSize(); ++num_elements)
                {
                    Vector<int> vect_pos_x;
                    Vector<int> vect_pos_z;
                    vect_pos_x.clearContent();
                    vect_pos_z.clearContent();
                    Vector<double> pos_angle;
                    pos_angle.clearContent();
                    int min_posx = 1000;
                    int min_posz = 1000;
                    int max_posx = 0;
                    int max_posz = 0;

                    for(int vector_size = 0; vector_size < (detector_seg->x_distribution(num_elements)).getSize(); ++vector_size)
                    {
                        int element_place =  (detector_seg->y_distribution(num_elements)(vector_size)*width) + detector_seg->x_distribution(num_elements)(vector_size);
                        double zj = point_cloud.Z(element_place);
                        double xj = point_cloud.X(element_place);
                        double yj = point_cloud.Y(element_place);

                        double x = xj - min_x_;
                        double z = zj - min_z_;

                        int pos_x  =(int)round(x/step_x);
                        int pos_z  =(int)round(z/step_z);

                        vect_pos_x.pushBack(pos_x);
                        vect_pos_z.pushBack(pos_z);

                        if(pos_x>0 && pos_x < x_bins && pos_z > 0 && pos_z < z_bins)
                            objects_projected(pos_x, pos_z) = 255;

                        if(vector_size == 0) //Sometimes, the vector may contain nothing --> would result in a seg_fault
                        {
                            pos_angle.pushBack(vect_pos_x(0));
                            pos_angle.pushBack(vect_pos_z(0));
                        }
                    }

                    //compute Orientation
                    int operation = -1;
                    operation = computeOrientation(vect_pos_x, vect_pos_z, ratioXY);
                    std::cout << "detection number " << num_elements << ": orientation is " << operation << " degrees" << std::endl;
                    pos_angle.pushBack(operation);
                    vect_pos_angle.pushBack(pos_angle);
                    vect_ratioXY.pushBack(ratioXY);
                }

                draw_hist(objects_projected, w,h,cim_labeledROI);
                /// Adding PCA to the image
                //Prepare Image
                cv::Mat img_extraite = cv::Mat::zeros(h, w, CV_8UC3); // create Matrix full of 0; warning : size !!
                cv::Mat pca_image;

                for (int column=0; column < w; column++)
                {
                    for (int row=0; row < h; ++row)
                    {
//                        img_extraite.at<unsigned char>(column,row) = (cim_final((w-w_img_extrait)/2 + column,row,0,0),cim_final((w-w_img_extrait)/2 + column,row,0,1),cim_final((w-w_img_extrait)/2 + column,row,0,2));
                        img_extraite.at<cv::Vec3b>(row,column)[0] = (cim_labeledROI(column,row,0,0));
//                        img_extraite.at<cv::Vec3b>(column,row)[1] = (cim_final(column,row,0,0));
//                        img_extraite.at<cv::Vec3b>(column,row)[2] = (cim_final(column,row,0,0));
                    }
                }

                // PCA Part
                cv::cvtColor(img_extraite, pca_image, cv::COLOR_BGR2GRAY);  //convert to grayscale image
                // Process grayscale image and find objects of interest
                //cv::threshold(pca_image, pca_image, 10, 255, CV_THRESH_BINARY); //apply thresholding
                vector<vector<cv::Point> > contours;
                vector<cv::Vec4i> hierarchy;
                cv::findContours(pca_image,contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
                double rate = 0;

                //Find the orientation of contours
                for (size_t i = 0; i < contours.size(); ++i)
                {
                    // Draw each contour only for visualisation purposes
                    cv::drawContours(img_extraite, contours, i, CV_RGB(255, 0, 0), 2, 8, hierarchy, 0);
                    // Find the orientation of each shape
                    rate = getOrientation(contours[i], img_extraite);
                }
                std::cout << "rate : " << rate << std::endl;
                //cv::waitKey(30);

                for(int num_elements=0; num_elements< detector_seg->x_distribution.getSize(); ++num_elements)
                {
                    if(vect_pos_angle(num_elements).getSize() ==3)
                    {
                        //correctAngle(vect_pos_angle(num_elements)(2), vect_ratioXY(num_elements));
                        ///Put data in a buffer so that we can print thel on image or save in files easily
                        char buffer[2], buffer2[5], bufferXY[5];
                        sprintf(buffer,"%f",vect_pos_angle(num_elements)(2)); //angle
                        sprintf(buffer2,"%f",rate); //ratio eigen values
                        sprintf(bufferXY,"%f",vect_ratioXY(num_elements)); //Disp
                        std::cout << "angle : " << buffer << ", disp : " << bufferXY << std::endl;
                        cv::putText(img_extraite, buffer, cv::Point((vect_pos_angle(num_elements)(0)),(vect_pos_angle(num_elements)(1))),CV_FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0,255,0), 3,8);
                        cv::putText(img_extraite, buffer2, cv::Point((vect_pos_angle(num_elements)(0))+30,(vect_pos_angle(num_elements)(1))+30),CV_FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255,255,0), 3,8);
                        cv::putText(img_extraite, bufferXY, cv::Point((vect_pos_angle(num_elements)(0))+60,(vect_pos_angle(num_elements)(1))+60),CV_FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0,255,255), 3,8);

                        ///save data in text file
//                        std::ofstream myfile;
//                        myfile.open("TM_acq_data1.dat", ios::app);
//                        if(myfile.is_open())
//                        {
//                            //myfile << "#x y" << std::endl;
//                            myfile << buffer << ' ' << buffer2 << ' ' << bufferXY <<  std::endl;
//                            myfile.close();
//                        }
//                        else std::cout << "Unable to open file" << std::endl;
                    }
                }
                cv::imshow( "Display window", img_extraite);
                cv::waitKey(30);
                img_extraite.release();
                /// End PCA
//                for(int num_elements=0; num_elements< detector_seg->x_distribution.getSize(); ++num_elements)
//                {
//                    char buffer[2];
//                    sprintf(buffer,"%d",int(vect_pos_angle(num_elements)(2)));
//                    cim_final.draw_text((vect_pos_angle(num_elements)(0)),(vect_pos_angle(num_elements)(1)),buffer,color);
//                }
            }
        }
        /// End of attempt 4

        detection_time = CPUTime()-ct;
        //////////////////////////////////////////////////////////////////////////////

        /*///////////////////////////////////////////TRACKING///////////////////////////																//Modified
        ct=CPUTime();
        tracker.process_tracking_oneFrame(HyposAll, det_comb, cnt, OutputHOGdetL, cim_tmp, camera);
        tracking_time = CPUTime()-ct;
        //////////////////////////////////////////////////////////////////////////////*/

        /*if(HyposAll.getSize() > 0)                                                                            //Modified
        {
            tracking_update_ctr++;
            //std::cout<<" Total tracked hypothesis : "<<HyposAll.getSize()<<
            //           "update_ctr "<<tracking_update_ctr<<std::endl;
            //tracking_started = false;
        }
        else
        {
            tracking_update_ctr = 0;
            tracking_started = false;
        }

        if(tracking_update_ctr >= tracking_update_thresh)
        {
            tracking_started = true;
            tracking_update_ctr = 0;
        }*/

        ////////////////////////// Visualization Part II////////////////////
        if(display_mode == ROI_MODE)
        {
            if(is_seg)
            {
                draw_roi(detector_seg->roi_image, w,h,cim_final);
                //draw_hist(detector_seg->labeledROIs, w,h,cim_final);

                /// ****************** Test crop cim_final ******************* //
//                int w_img_extrait = 100;
//                int h_img_extrait = 100;
//                cv::Mat img_extraite = cv::Mat::zeros(w_img_extrait, h_img_extrait, CV_8UC3); // create Matrix full of 0; warning : size !!
//                cv::Mat img_extraite_rescale = cv::Mat::zeros(h, w, CV_8UC3); //rescaled image for display --> usual size w:640, h:480
//                cv::Mat pca_image;
//
//
//                for (int column=0; column < w_img_extrait; column++)
//                {
//                    for (int row=0; row < h_img_extrait; ++row)
//                    {
////                        img_extraite.at<unsigned char>(column,row) = (cim_final((w-w_img_extrait)/2 + column,row,0,0),cim_final((w-w_img_extrait)/2 + column,row,0,1),cim_final((w-w_img_extrait)/2 + column,row,0,2));
//                        img_extraite.at<cv::Vec3b>(column,row)[0] = (cim_final((w-w_img_extrait)/2 + column,row,0,0));
//                        img_extraite.at<cv::Vec3b>(column,row)[1] = (cim_final((w-w_img_extrait)/2 + column,row,0,1));
//                        img_extraite.at<cv::Vec3b>(column,row)[2] = (cim_final((w-w_img_extrait)/2 + column,row,0,2));
//                    }
//                }
//                cv::resize(img_extraite, img_extraite_rescale,img_extraite_rescale.size(), 0, 0, cv::INTER_LINEAR );
//
//                    /// PCA part
//                cv::cvtColor(img_extraite_rescale, pca_image, cv::COLOR_BGR2GRAY);  //convert to grayscale image
//                // Process grayscale image and find objects of interest
//                cv::threshold(pca_image, pca_image, 50, 255, CV_THRESH_BINARY); //apply thresholding
//                vector<vector<cv::Point> > contours;
//                vector<cv::Vec4i> hierarchy;
//                cv::findContours(pca_image,contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
//
//                //Find the orientation of contours
//                for (size_t i = 0; i < contours.size(); ++i)
//                {
//                    // Draw each contour only for visualisation purposes
//                    cv::drawContours(img_extraite_rescale, contours, i, CV_RGB(255, 0, 0), 2, 8, hierarchy, 0);
//                    // Find the orientation of each shape
//                    getOrientation(contours[i], img_extraite_rescale);
//                }
//                    ///End of PCA part - WORKING
//                cv::imshow( "Display window", img_extraite_rescale );
//                cv::waitKey(30);
                /// END OF TEST --> COMPLETED
            }
            else
            {
                draw_roi(detector->roi_image, w,h,cim_final);
                draw_hist(detector->labeledROIs, w,h,cim_final);
            }
        }


        ROS_INFO("about to display ");

        std::ofstream myfile;
        char filename[100];

        if(display_mode != ODOM_MODE)
        {
            riddle::PersonDetected p;

            detList.detectedPersonsList.resize(detected_bounding_boxes.getSize());

            //vis_gp(cim_final, camera);

            int indx = -1;
            float d = 10.;
            useful_person_detected = false;

            ROS_INFO("Numer of bouding boxes [%d] ", (int)detected_bounding_boxes.getSize());


#ifdef IROS_EVAL_SECTION
            sprintf(filename, "/home/aamekonn/catkin_ws/tmp/pdt/img%04d.txt", global_frame_counter);

            myfile.open(filename, ios::out);
            //global_frame_counter
            myfile<<detected_bounding_boxes.getSize()<<std::endl;
#endif
            for(int jj = 0; jj < detected_bounding_boxes.getSize(); ++jj)
            {

                //push the bounding box onto the message
                RenderBBox2D(detected_bounding_boxes(jj), cim_final, 0, 255, 0);
                p.position[0] = (int)detected_bounding_boxes(jj)(0);
                p.position[1] = (int)detected_bounding_boxes(jj)(1);
                p.position[2] = (int)detected_bounding_boxes(jj)(2);
                p.position[3] = (int)detected_bounding_boxes(jj)(3);

//                std::cout << "show data detected_bounding_boxes : " << std::endl;
//                std::cout << " p.position[0] : " <<  p.position[0] << ",  p.position[1] : " <<  p.position[1] << std::endl;
//                std::cout << " p.position[2] : " <<  p.position[2] << ",  p.position[3] : " <<  p.position[3] << std::endl;

#ifdef IROS_EVAL_SECTION
                myfile<<p.position[0]<<" "<<p.position[1]<<" "<<p.position[2]<<" "<<p.position[3]<<" "<<(1.0/detected_bounding_boxes(jj)(4))<<std::endl;
#endif
                //p.dist TODO
                //access this from b_depth

                //dist...take the minimum distance within the bounding box
                int x_indx_min = (p.position[0] - 0.5*p.position[2])<0?0:(p.position[0] - 0.5*p.position[2]);
                int x_indx_max = (p.position[0] - 0.5*p.position[2])>w?w:(p.position[0] + 0.5*p.position[2]);
                int y_indx_min = (p.position[1] - 0.5*p.position[3])<0?0:(p.position[1] - 0.5*p.position[3]);
                int y_indx_max = (p.position[1] - 0.5*p.position[3])>h?h:(p.position[1] + 0.5*p.position[3]);

                float min_dist = 10.;
                for (int l = x_indx_min; l < x_indx_max; l++)
                {
                    for (int k = y_indx_min; k < y_indx_max; k++)
                    {
                        float ds = b_depth[k*w + l];
                        if ((ds > 0.4) && (ds < min_dist))
                            min_dist = ds;
                    }
                }
                p.dist = min_dist;//b_depth[p.position[1]*w + p.position[0]];

                if(p.dist < d)
                {
                    //printf("p.dist [%f] and indx [%d] max indx [%d] ",
                    //       p.dist, jj, detected_bounding_boxes.getSize()); fflush(stdout);
                    indx = jj;
                    d = p.dist;
                }

                detList.detectedPersonsList[jj] = p;
            }

#ifdef IROS_EVAL_SECTION
            myfile.close();
#endif

            if(indx != -1)
            {

                position[0] = detected_bounding_boxes(indx)(0);;
                position[1] = detected_bounding_boxes(indx)(1);;
                position[2] = d;

                useful_person_detected = true;

                ROS_INFO("useful_person_detected with [%f %f %f] ",
                         position[0], position[1], position[2]);
            }

            detList.header.stamp = ros::Time::now();
        }

        const unsigned char color[] = {0,255,0};
        const unsigned char bcolor[] = {50,50,50};
        //cim_tmp.draw_text(10,10, "Tracking",color,0,1,20);                                                          //COMMENT**********************
        cim_final.draw_text(10,10, "Detection and Tracking",color,0,1,20);
        //cim_final.draw_image(w,cim_tmp);                                                                            // COMMENT !!!!!!!!!!!!!!!!!!!!!!

        if(show_help)
            cim_final.draw_text(650,200, help_string, color,bcolor,0.7,20);
        cim_final.draw_text(700,450,"Press F1 for more help!",color,bcolor,0.7,20);

        // Time
        char str[50];
        float fps=0;
        cpu_time_end = CPUTime();
        double diff = cpu_time_end - cpu_time_start;
        fps = 1.0/(diff);
        cpu_time_start = cpu_time_end;

        int l = 20;
        if(cnt%l==0)
        {
            fps_l = t_fps/l;
            t_fps=0;
        }
        t_fps +=fps;
        if(show_stat)
        {
            sprintf(str,"Total time per frame: %d", int(diff*1000));
            cim_final.draw_text(10,380,str,color,bcolor,0.7,20);
            sprintf(str,"%fFps", fps);
            cim_final.draw_text(10,410,str,color,bcolor,0.7,20);
            sprintf(str,"%fFps (average of %d frames)", fps_l, l);
            cim_final.draw_text(10,440,str,color,bcolor,0.7,20);

            if(display_mode != ODOM_MODE)
            {
                sprintf(str,"%d", int(fovis_time*1000));
                cim_final.draw_text(10,200,"Odometry",color,bcolor,0.7,20);
                cim_final.draw_text(170,200,str,color,bcolor,0.7,20);
                sprintf(str,"%d  (# of points %0.2f)", int(PC_time*1000), point_cloud.number_of_points/307200.0);
                cim_final.draw_text(10,225,"PointCloud",color,bcolor,0.7,20);
                cim_final.draw_text(170,225,str,color,bcolor,0.7,20);
                sprintf(str,"%d", int(GP_time*1000));
                cim_final.draw_text(10,250,"GP Estimation",color,bcolor,0.7,20);
                cim_final.draw_text(170,250,str,color,bcolor,0.7,20);
                sprintf(str,"%d", int(detection_time*1000));
                cim_final.draw_text(10,275,"Detector",color,bcolor,0.7,20);
                cim_final.draw_text(170,275,str,color,bcolor,0.7,20);
                /*sprintf(str,"%d", int(tracking_time*1000));																//Modified
                cim_final.draw_text(10,300,"Tracker",color,bcolor,0.7,20);
                cim_final.draw_text(170,300,str,color,bcolor,0.7,20);*/																//Modified
                // Fovis
                //                cim1.draw_text(10,100,isometryToString(cam_to_local).c_str(),color,bcolor,0.7,20);
            }
        }

        const unsigned char rcolor[3]= {255,0,0};
        if(record_sequence)
            cim_final.draw_text(10,150,"RECORDING ...", rcolor, bcolor, 0.7, 30);

        //display.display(cim_final);
        //1024 x 768
        /*
        cim_final.resize(1024);
        CImg<unsigned char> rimg(1024,768,1,3,0);
        rimg.draw_image(0,(rimg.height() - cim_final.height())/2,cim_final);

        //unsigned char* res = new unsigned char[rimg.size()];
        //memcpy(res,rimg.get_permute_axes("cxyz").data(), rimg.size());
        //result_image_queue.push(res);

        if(capture || display.is_keyC())
        {
            static unsigned int oi=0;
            sprintf(capture_path,path,++oi);
            cim_final.save(capture_path);
            capture = false;
        }
        */
        //convert cim_final to opencv
        unsigned char * src_ptr = (unsigned char*)cim_final.data();

#ifdef IROS_EVAL_SECTION
        unsigned char *tmp = new unsigned char[w*h*3];
#endif

        int channel_sz = h*w;
        for(int i = 0; i < h; i++)
        {
            for(int j = 0; j < w; j++)
            {
                img_vis.data[(i*w + j)*3 + 0] = src_ptr[ i*w + j + 0*channel_sz]; //r channel
                img_vis.data[(i*w + j)*3 + 1] = src_ptr[ i*w + j + 1*channel_sz]; //g channel
                img_vis.data[(i*w + j)*3 + 2] = src_ptr[ i*w + j + 2*channel_sz]; //b channel

#ifdef IROS_EVAL_SECTION
                tmp[(i*w + j)*3 + 2] = src_ptr[ i*w + j + 0*channel_sz]; //r channel
                tmp[(i*w + j)*3 + 1] = src_ptr[ i*w + j + 1*channel_sz]; //g channel
                tmp[(i*w + j)*3 + 0] = src_ptr[ i*w + j + 2*channel_sz]; //b channel
#endif
            }
        }
        img_vis.header.stamp = ros::Time::now();

#ifdef IROS_EVAL_SECTION
        sprintf(filename, "/home/aamekonn/catkin_ws/tmp/img/img%04d.png", global_frame_counter);
        cv::Mat img(h, w, CV_8UC3, tmp);
        cv::imwrite(filename, img);
        delete [] tmp;

        global_frame_counter++;
#endif
        global_frame_counter++;
        /////////////////////////////////////////////////////////////////////////
        ++cnt;

        dp_queue.push(b_depth);
        img_queue.push(b_image);

        delete[] b_gray_image;
        delete fv_dp;
    }

    static DetectionTrackingSystem* _this;

    //    static void grabber_callback(const float *depth, const unsigned char *image)
    //    {
    //        //std::cout<<"grabber callback"<<std::endl;
    //        uint w = Globals::dImWidth, h = Globals::dImHeight;

    //        // Get RGB Image
    //        //unsigned char* b_image = new unsigned char[w*h*3];
    //        memcpy(b_image_g, image, w*h*3);

    //        // Get DepthMap
    //        //float* b_depth = new float[h*w];
    //        memcpy(b_depth_g, depth, w*h*sizeof(float));

    //        _this->main_process(b_image_g, b_depth_g, w, h);
    //    }


    void get_from_file(long int frm)
    {
        char pth[200];
        sprintf(pth,"%s/img_%08d",Globals::from_file_path.c_str(),(int)frm);
        ifstream imgf(pth, ios::binary);
        sprintf(pth,"%s/dp_%08d",Globals::from_file_path.c_str(),(int)frm);
        ifstream dpf(pth,ios::binary);
        if(!imgf.is_open() || !dpf.is_open())
            return;

        uint w = Globals::dImWidth, h = Globals::dImHeight;

        unsigned char* b_image = new unsigned char[w*h*3];
        imgf.read((char*)b_image, Globals::dImWidth*Globals::dImHeight*3);
        imgf.close();

        float* b_depth = new float[h*w];
        dpf.read((char*)b_depth, Globals::dImWidth*Globals::dImHeight*sizeof(float));
        dpf.close();

        main_process(b_image, b_depth, Globals::dImWidth, Globals::dImHeight);
    }

    static void* get_files(void* obj)
    {
        DetectionTrackingSystem* o = (DetectionTrackingSystem*)obj;
        long int frm1 = 0;
        while(1)
            o->get_from_file(frm1++);
    }

    // send to stream, save images and depth and clean memory
    void sink_frame(long int& frm, bool is_runing = true)
    {
        char pth[200];

        // Try to send the result_image to the stream(s)
        if(result_image_queue.size()>0)
        {
            unsigned char* ptr2 = result_image_queue.front();
            if(ptr2)
            {
                result_image_queue.pop();
                //try{
                //if(is_runing)
                //streaming.sendImage(ptr2, 3*1024*768);
                //delete[] ptr2;
                //}catch(...){}
            }
        }

        // Record image and depth map (in recording mode only,
        // then delete image and depth from memory.
        if(img_queue.size()>0 && dp_queue.size()>0)
        {
            unsigned char* ptr = img_queue.front();
            float* ptr1 = dp_queue.front();
            img_queue.pop();
            dp_queue.pop();

            if(record_sequence)
            {
                sprintf(pth,"rec/img_%08d",(int)frm);
                ofstream imgf(pth, ios::binary);
                sprintf(pth,"rec/dp_%08d",(int)frm);
                ofstream dpf(pth, ios::binary);
                imgf.write((char*)ptr,640*480*3);
                dpf.write((char*)ptr1,640*480*sizeof(float));
                imgf.close();
                dpf.close();
            }

            ++frm;
        }
    }

//    void run ()
//    {
//        std::cout<<"...starting grab"<<std::endl;
//        long int frm = 0;
//
//        while(!display.is_closed())
//        {
//            std::cout << "Display is open !!" << std::endl;                                             // Modified
//            check_keys();
//            sink_frame(frm);
//        }
//
//        while(dp_queue.size()>0)
//        {
//            sink_frame(frm,false);
//        }
//    }

    void run ()
    {
        std::cout<<"...starting grab"<<std::endl;
        long int frm = 0;

        if(!display.is_closed())
        {
            std::cout << "Display is open !!" << std::endl;                                             // Modified
            check_keys();
            sink_frame(frm);
        }

        while(dp_queue.size()>0)
        {
            sink_frame(frm,false);
        }
    }


    void check_keys()
    {
        std::cout <<"Checking Keys" << std::endl;                                                               //Modified

        if(display.is_keyC())
        {
            capture = true;
        }
        else if(display.is_keyI())
        {
            display_mode = IMAGE_MODE;
            detector_seg->visualize_roi=false;
            detector->visualize_roi=false;
        }
        else if(display.is_keyD())
        {
            display_mode = DEPTH_MODE;
            detector_seg->visualize_roi=false;
            detector->visualize_roi=false;
        }
        else if(display.is_keyO())
        {
            display_mode = ODOM_MODE;
            detector_seg->visualize_roi=false;
            detector->visualize_roi=false;
        }
        else if(display.is_keyR())
        {
            display_mode = ROI_MODE;
            detector_seg->visualize_roi=true;
            detector->visualize_roi=true;
        }
        else if(display.is_keyW())
        {
            if(is_seg)
            {
                is_seg = false;
                SetTitle();
            }
        }
        else if(display.is_keyQ())
        {
            if(!is_seg)
            {
                is_seg = true;
                SetTitle();
            }
        }
        /*else if(display.is_keyA())															//Modified
        {
            use_HOG = true;
            SetTitle();
        }
        else if(display.is_keyS())
        {
            use_HOG = false;
            SetTitle();
        }*/
        else if(display.is_key1())
        {
            detector_mode = DEPTH_DETECTOR;
            depth_detector.visualize_roi = detector->visualize_roi;
            detector = &depth_detector;
            depth_detector_seg.visualize_roi = detector_seg->visualize_roi;
            detector_seg = &depth_detector_seg;
            SetTitle();
        }
        else if(display.is_key2())
        {
            detector_mode = DEPTH_LM_DETECTOR;
            depth_detector_lm.visualize_roi = detector->visualize_roi;
            detector = &depth_detector_lm;
            depth_detector_lm_seg.visualize_roi = detector_seg->visualize_roi;
            detector_seg = &depth_detector_lm_seg;
            SetTitle();
        }
        else if(display.is_keyF1())
        {
            show_help = true;
        }
        else if(display.is_keyF2())
        {
            show_help = false;
        }
        else if(display.is_keyF5())
        {
            record_sequence = true;
        }
        else if(display.is_keyF6())
        {
            record_sequence = false;
        }
        else if(display.is_keyZ())
            show_stat = true;
        else if(display.is_keyX())
            show_stat = false;
    }

    void SetTitle()
    {
        char title[300];
        char pattern[] = "Upper Body Detector - %s <without ROI-Segmentation> - %s";
        char patern_seg[] = "Upper Body Detector - %s <with ROI-Segmentation> - %s";
        char* p = is_seg ? patern_seg : pattern;
        /*char hog_title[100];															//Modified
        if(use_HOG)
            sprintf(hog_title,"%s","With groundHOG");
        else
            sprintf(hog_title,"%s","Without groundHOG");
        */
        switch(detector_mode)
        {
        case DEPTH_DETECTOR:
            sprintf(title,p,"Depth Detector","Without groundHOG");                            //sprintf(title,p,"Depth Detector",hog_title);
            break;
        case DEPTH_LM_DETECTOR:
            sprintf(title,p,"Local Max Depth Detector","Without groundHOG");                  //sprintf(title,p,"Local Max Depth Detector",hog_title);
            break;
        }

        display.set_title("without groundHOG");                                     //display.set_title(title);
    }

    void init()
    {
        //use_HOG = Globals::use_hog;															//Modified
        gp_count_down=0;
        motion_not_valid_count = 0;
        cnt = 0;
        t_fps=0;
        fps_l=0;

        std::cout<<"in init.."<<std::endl;

        strcpy(path,"%d.png");
        strcpy(help_string,
               "1, 2       Select detector: 1)Depth  2)Local Maximum Depth\n"
               "q/w       Activate/Deactivate Segmentation ROI\n"
               "a/s       Activate/Deactivate groundHOG\n"
               "d         Show Depth-Map\n"
               "i         Show RGB Image\n"
               "r         Show ROI\n"
               "o         Show Odometry\n"
               "c         Capture frame\n"
               "F5/F6     Start/Stop recording\n"
               "z/x       Show/Hide Statistics\n"
               "F1/F2     Show/Hide Help\n");

        std::cout<<help_string<<std::endl;

        ReadUpperBodyTemplate(upper_body_template);
        capture=false;
        display_mode = IMAGE_MODE;
        if(Globals::use_local_max)
            detector_mode = DEPTH_LM_DETECTOR;
        else
            detector_mode = DEPTH_DETECTOR;
        show_help = false;
        show_stat = false;
        is_seg=Globals::use_segmentation_roi;

        is_first=true;
        is_odom_valid = true;

        cpu_time_start = CPUTime();
        detector = &depth_detector;
        detector_seg = &depth_detector_seg;

        last_gp = base_camera.get_GP();
        is_last_gp_valid = false;
        is_gp_estim_ignored = false;

        record_sequence = false;

        memset(&cam_params, 0, sizeof(fovis::CameraIntrinsicsParameters));
        cam_params.width = Globals::dImWidth;
        cam_params.height = Globals::dImHeight;
        cam_params.fx = base_camera.K()(0,0);
        cam_params.fy = base_camera.K()(1,1);
        cam_params.cx = base_camera.K()(2,0);
        cam_params.cy = base_camera.K()(2,1);

        this->img_vis.height = Globals::dImHeight;
        this->img_vis.width = Globals::dImWidth;
        this->img_vis.encoding = sensor_msgs::image_encodings::RGB8;
        this->img_vis.is_bigendian = 0;
        this->img_vis.step = this->img_vis.width*3;
        this->img_vis.data.resize(this->img_vis.height*this->img_vis.step);

        SetTitle();
    }

    const sensor_msgs::Image & get_img() const {    return this->img_vis; }
    const riddle::DetectedPersonsList & get_people_list() const  {   return this->detList;   }
    const std::vector<double> & getPersonPosition() const   {   return this->position;      }
    std::vector<double> & getServicePosition()   {   return this->servicePosition;      }
    bool is_useful_person_det() {   return this->useful_person_detected;    }
    /*bool is_tracking_started()  {   return this->tracking_started;          }                                             //Modified
    void set_tracking_started() {   this->tracking_started = true;
                                    this->tracking_update_ctr = 0;          }

    int get_tracking_update_ctr()   {   return this->tracking_update_ctr;   }
    void set_tracking_update_thresh(int th) {   tracking_update_thresh = th;    }*/
    void reset_counters()   {                                                                                           //Modified
        //tracking_update_ctr = 0;
        useful_person_detected = false;
        //tracking_started = false;
        //HyposAll.clearContent();
    }

    DetectionTrackingSystem() : det_comb(23,0),
        base_camera(Globals::camPath_left.c_str(),true),
        odimg(640,480,1,3,0)
    {
        init();
        useful_person_detected = false;
        //tracking_started = false;
        //tracking_update_ctr = 0;
        position.resize(3);
        std::cout<<"initialization done!"<<std::endl;
    }

    ~DetectionTrackingSystem()
    {
        delete odom;
        delete fovis_rect;
    }

    CImg<unsigned char> odimg;
    double lodx, lody, lodz;

    queue<unsigned char*> img_queue;
    queue<float*> dp_queue;
    queue<unsigned char*> result_image_queue;
    bool record_sequence;

    Camera base_camera;
    Vector<double> last_gp;
    GroundPlaneEstimator GPEstimator;
    bool is_last_gp_valid;
    bool is_gp_estim_ignored;
    Eigen::Vector3d motion_xyz;
    Eigen::Vector3d motion_rpy;

    //////////////////Detectors//////////////////
    Detector* detector;
    DepthDetector depth_detector;
    DepthDetector_LM depth_detector_lm;

    Detector_Seg* detector_seg;
    DepthDetector_Seg depth_detector_seg;
    DepthDetector_LM_Seg depth_detector_lm_seg;


    /*bool use_HOG;*/															//Modified
#ifdef USE_HOG
    HOGDetector hog_detector;
#endif
    //////////// Time ////////////////////////
    double cpu_time_start, cpu_time_end;
    double t_fps, fps_l;

    ///Streaming//////////////////////////////
    //StreamingApp streaming;

    ///FOVIS//////////////////////////////////
    fovis::CameraIntrinsicsParameters cam_params;
    fovis::VisualOdometry* odom;
    fovis::Rectification* fovis_rect;
    bool is_first, is_odom_valid;
    Matrix<double> mm;

    Detections det_comb;																//Modified
    /*///////TRACKING///////////////////////////																//Modified
    Detections det_comb;
    Tracker tracker;
    Vector< Hypo > HyposAll;*/
    long int cnt;

    //////////////////////////////////////////
    Matrix<double> upper_body_template;
    CImgDisplay display, display_labeledROIs;            //ADDED display_labeledROIs
    bool capture;
    char capture_path[128];
    char path[128];
    char help_string[1000];
    bool show_help;
    bool show_stat;
    bool is_seg;

    enum DISPLAY_MODE
    {
        IMAGE_MODE,
        DEPTH_MODE,
        ROI_MODE,
        ODOM_MODE
    } display_mode;

    sensor_msgs::Image img_vis;
    riddle::DetectedPersonsList detList;

    std::vector<double> position;
    std::vector<double> servicePosition;
    bool useful_person_detected;
    /*bool tracking_started;																//Modified
    int tracking_update_ctr;
    int tracking_update_thresh;*/
};

DetectionTrackingSystem* DetectionTrackingSystem::_this = NULL;

///////////////////////////////////////////////////////////////////////
// PreLoadData:
//      Reads required data from files.
// parameters:
//      output:
//          cameras     -   Camera settings that is related to each frame.
//          images      -   Input images for each frame.
//          depth_maps  -   Depth maps of input images.
///////////////////////////////////////////////////////////////////////
void PreLoadData(Vector<Camera>& cameras, Vector<CImg<unsigned char> >& images, Vector<Matrix<double> >& depth_maps)
{
    char path[128];
    char pToGp[128];

    cameras.setSize(Globals::numberFrames);
    depth_maps.setSize(Globals::numberFrames);
    //    if(Globals::export_result_images)
    images.setSize(Globals::numberFrames);

    for (int i = Globals::nOffset, j = 0; i < Globals::nOffset+Globals::numberFrames; i++, j++)
    {

        if(i%10==0)
        {
            cout << "Loading " << i << " out of " << Globals::nOffset+Globals::numberFrames << endl;
        }

        // ------------------- LOAD CAMERAS
        sprintf(path, Globals::camPath_left.c_str(), i);
        Camera left_camera(path, 1);

        if(Globals::is_project_plane_to_world)
        {
            Matrix<double> left_camera_R = left_camera.get_R();
            Vector<double> left_camera_T = left_camera.get_t();
            Matrix<double> left_camera_K = left_camera.get_K();

            sprintf(pToGp, Globals::path_to_planes.c_str(), i);
            Vector<double> gp;
            gp.readTXT(pToGp, 4);

            gp(3) *= 1.0/Globals::WORLD_SCALE;
            gp = AncillaryMethods::PlaneToWorld(left_camera, gp);
            left_camera = Camera(left_camera_K, left_camera_R, left_camera_T, gp);
        }

        cameras(j) = left_camera;

        // ------------------- LOAD RGB IMAGES
        //        if(Globals::export_result_images)
        //        {
        sprintf(path, Globals::sImagePath_left.c_str(), i);
        //            QImage imageLeft(path);
        //            images(j) = imageLeft;
        images(j).load(path);
        //        }

        // ------------------- LOAD DEPTH
        depth_maps(j) = AncillaryMethods::GetDepthLibelas(i, left_camera, Globals::baseline);
    }
}

Vector<Vector<double> > BboxNMS(Vector<Vector<double> >& detected_bounding_boxes, double thresh)
{
    Vector<int> remove(detected_bounding_boxes.getSize(), -1);
    Vector<double> interRect;
    for(int i = 0; i < detected_bounding_boxes.getSize(); i++)
    {

        if(remove(i) > 0)
            continue;

        for(int j = 0; j < detected_bounding_boxes.getSize(); ++j)
        {
            if(j == i || remove(j) > 0)
                continue;

            AncillaryMethods::IntersetRect(detected_bounding_boxes(i), detected_bounding_boxes(j), interRect);
            double intersection = interRect(2)*interRect(3);
            double unionRect = detected_bounding_boxes(i)(2)*detected_bounding_boxes(i)(3)+
                    detected_bounding_boxes(j)(2)*detected_bounding_boxes(j)(3) - intersection;



            if(intersection/unionRect> thresh)
            {
                // Compare distances (reverse of score)
                if(detected_bounding_boxes(i)(4) < detected_bounding_boxes(j)(4))
                {
                    remove(j) = 1;
                }
                else
                {
                    remove(i) = 1;
                    break;
                }
            }
        }
    }

    Vector<Vector<double> > detected_bounding_boxes_after_nms;
    for(int i = 0; i < detected_bounding_boxes.getSize(); ++i)
    {
        if(remove(i) < 0)
        {
            detected_bounding_boxes_after_nms.pushBack(detected_bounding_boxes(i));
        }
    }
    return detected_bounding_boxes_after_nms;
}

void WriteToFile(Vector<Vector< double > > detected_bounding_boxes, int index, ofstream& det_file, bool thriple=true, bool reverse_score=true)
{
    char filename[50];
    sprintf(filename,"image_%08d_0.png",index);

    if(!det_file.is_open())
    {
        cout<<"bad file path."<<endl;
        return;
    }

    double x,y,sc,x2,y2;
    int w,h;

    det_file<<"\""<<filename<<"\"";

    for(int i =0; i<detected_bounding_boxes.getSize();++i)
    {
        x = detected_bounding_boxes(i)(0);
        y = detected_bounding_boxes(i)(1);
        w = detected_bounding_boxes(i)(2);
        h = detected_bounding_boxes(i)(3);
        if(thriple)
            h*=3;
        sc = detected_bounding_boxes(i)(4);
        if(reverse_score)
            sc=1-sc;
        x2=(x+w)>640?640:x+w;
        y2=(y+h)>480?480:y+h;

        if(i==0)
            det_file<<":("<<(int)x<<","<<(int)y<<","<<(int)x2<<","<<(int)y2<<"):"<<sc;
        else
            det_file<<",("<<(int)x<<","<<(int)y<<","<<(int)x2<<","<<(int)y2<<"):"<<sc;
    }
    det_file<<";"<<endl;
}

Vector<double> getBBoxFrom3DPos(Camera cam, Vector<double> pos, double height, bool occFlag)
{

    Vector<double> bbox;

    //******************************
    // Get Ground Plane
    //******************************
    Vector<double> gpn=cam.get_GPN();
    Vector<double> copyGPN;

    //*******************************
    // Get further Camera parameter
    //*******************************

    Vector<double> VPN=cam.get_VPN();

    //*******************************
    // Init bbox and score
    //*******************************
    Vector<double> x(pos);
    Vector<double> xTop;

    //*************************************
    // Project x on the Ground Plane
    //*************************************

    cam.ProjectToGP(x, Globals::WORLD_SCALE, x);

    //**************************************
    // Compute x Top
    //**************************************
    xTop = x;
    copyGPN = gpn;
    copyGPN *=height;
    xTop -= copyGPN;

    //***************************************
    // Check if x is in front of the Camera
    //***************************************

    double invWorldScale = 1.0/Globals::WORLD_SCALE;
    Vector<double> auxX = x * invWorldScale;
    Vector<double> auxXTop = xTop * invWorldScale;

    if(!(cam.isPointInFrontOfCam(auxX) && cam.isPointInFrontOfCam(auxXTop)))
    {
        bbox.clearContent();
        return bbox;
    }

    //***************************************
    // Projection on Screen
    //***************************************

    Vector<double> p1;
    Vector<double> p2;


    cam.WorldToImage(x, Globals::WORLD_SCALE, p1);

    //    if(p1(0) < 0 || p1(0) >= Globals::dImWidth || p1(1) < 0 || p1(1) >= Globals::dImHeight)
    //    {
    //        bbox.clearContent();
    //        return bbox;
    //    }
    cam.WorldToImage(xTop, Globals::WORLD_SCALE, p2);

    Vector<double> diffP2P1(2);
    diffP2P1(0)= fabs(p2(0)) - fabs(p1(0));
    diffP2P1(1)= fabs(p2(1)) - fabs(p1(1));

    double ht = diffP2P1.norm();

    double wt_half;
    if(occFlag)
    {
        wt_half = ht / 4;
    }
    else
    {
        wt_half = ht / 5;
    }

    bbox.clearContent();;
    bbox.pushBack(floor(p1(0) - wt_half));
    bbox.pushBack(floor(p1(1) - ht));
    bbox.pushBack(floor(p1(0) + wt_half));
    bbox.pushBack(floor(p1(1)));

    bbox(0) = max(0.0,bbox(0));
    bbox(1) = max(0.0,bbox(1));

    bbox(2) = min((double)Globals::dImWidth-1,bbox(2));
    bbox(2) = fabs(bbox(2)) - fabs(bbox(0));

    bbox(3) = min((double)Globals::dImHeight-1,bbox(3));
    bbox(3) = fabs(bbox(3)) - fabs(bbox(1));

    //******************************************
    // Check if the bbox out of frame
    //******************************************

    if(bbox(0) < 0) bbox(0) = 0;
    if(bbox(0) + bbox(2)>=Globals::dImWidth) bbox(2) = Globals::dImWidth-bbox(0)-1;

    if(bbox(1) < 0) bbox(1) = 0;
    if(bbox(1)+bbox(3) >= Globals::dImHeight) bbox(3) = Globals::dImHeight - bbox(1) - 1;


    return bbox;
}

void exportBBOX(Vector<Hypo> Hypos, Camera cam, int frame, Vector<Vector<double> >& bboxes)
{

    // How create a ofstream
    // ofstream aStream('filename');

    Matrix<double> mat( 13, Hypos.getSize(), 0.0);
    Vector<Vector<double> >TrajPts;
    Vector<double> score;
    Vector<double> bb;

    Hypo hypo;
    Matrix<double> allP;

    for(int i = 0; i < Hypos.getSize(); i++)
    {
        hypo = Hypos(i);

        hypo.getXProj(allP);
        Vector<double> vX = allP.getRow(allP.y_size()-1);
        vX(2) = vX(1);
        vX(1) = vX(3);
        vX.resize(3);

        double height = hypo.getHeight();

        bb = getBBoxFrom3DPos(cam, vX, min(1.98,height), false);
        if(bb.getSize()==4)
        {
            score.pushBack(hypo.getScoreMDL());
            bb.pushBack(hypo.getScoreMDL());
            bboxes.pushBack(bb);
        }

    }
}

///////////////////////////////////////////////////////////////////////
// for timing the preload part
///////////////////////////////////////////////////////////////////////
time_t  user_time_start, user_time_end;
double  cpu_time_start, cpu_time_end;

int main_preload()
{
    std::cout << "entering main_preload" << std::endl;
    Vector<Camera> cameras;
    Vector<CImg<unsigned char> > images;
    Vector<Matrix<double> > depth_maps;
    PreLoadData(cameras, images, depth_maps);

    Matrix<double> upper_body_template;

    ReadUpperBodyTemplate(upper_body_template);

    // Timing Code - Start
    time(&user_time_start);
    cpu_time_start = CPUTime();

    //    Detector detector;
    Detector* detector=0;
    Detector_Seg* detector_seg=0;
    if(Globals::use_segmentation_roi)
    {
        if(Globals::use_local_max)
            detector_seg = new DepthDetector_LM_Seg();
        else
            detector_seg = new DepthDetector_Seg();
    }
    else
    {
        if(Globals::use_local_max)
            detector = new DepthDetector_LM();
        else
            detector = new DepthDetector();
    }
#ifdef USE_HOG
    HOGDetector hog_detector;
    hog_detector.rescore=true;
#endif

    GroundPlaneEstimator GPEstimator;
    /*Tracker tracker;																												//Modified
    Vector< Hypo > HyposAll;*/
    Detections det_comb(23,0);


    ofstream* det_file;
    if(Globals::export_bounding_box)
    {
        cout<<Globals::bounding_box_path.c_str()<<endl;
        det_file = new ofstream(Globals::bounding_box_path.c_str());
    }

    CImg<unsigned char> cim_out(Globals::dImWidth*2,Globals::dImHeight,1,3); //cim_labeledROI(Globals::dImWidth*2,Globals::dImHeight,1,3);

    for(int current_frame = 0; current_frame < Globals::numberFrames; current_frame++)
    {
        int cnt = current_frame+Globals::nOffset;
        if(Globals::verbose){
            cout << "\33[33;40;1m" << "---------------------------------------------------------------------------------------------------------------" << "\33[0m" << endl;
            cout << "\33[33;40;1m" << "---------------------------------------------------------------------------------------------------------------" << "\33[0m" << endl;
            cout << "\33[33;40;1m" <<"                                  Processing image " << current_frame + Globals::nOffset << "\33[0m"<< endl;
            cout << "\33[33;40;1m" << "---------------------------------------------------------------------------------------------------------------" << "\33[0m" << endl;
        }

        ///////////////////////////////////////////////////////////////////////////
        // main Process
        std::cout<< "entering main_preload > main_process" << std::endl;
        PointCloud point_cloud(cameras(current_frame), depth_maps(current_frame));
        //        Vector<double> gp = GPEstimator.ComputeGroundPlane(point_cloud);
        Vector<Vector< double > > detected_bounding_boxes;
        if(Globals::use_segmentation_roi)
        {
            detector_seg->ProcessFrame(cameras(current_frame), depth_maps(current_frame), point_cloud, upper_body_template, detected_bounding_boxes);
        }
        else
            detector->ProcessFrame(cameras(current_frame), depth_maps(current_frame), point_cloud, upper_body_template, detected_bounding_boxes);
        ///////////////////////////////////////////////////////////////////////////

        CImg<unsigned char> cnt_image=images(current_frame);

        /*Vector<Vector < double > > OutputHOGdetL;*/															//Modified

#ifdef USE_HOG
        if(Globals::use_hog)
            OutputHOGdetL = hog_detector.runHogPr2(cnt,cnt_image.get_permute_axes("cxyz").data(),cameras(current_frame), detected_bounding_boxes);
#endif

        bool multiply_by_3 = false;
        for(int i = 0; i < detected_bounding_boxes.getSize(); i++)
        {
            detected_bounding_boxes(i)(3) = detected_bounding_boxes(i)(3) *3.0;
        }

        /*if(Globals::use_hog)                                                                                                  //Modified
            detected_bounding_boxes = BboxNMS(detected_bounding_boxes,0.5);*/

        /*// Tracking ////////////////////////////////////////////////////																//Modified
        Vector<double> oneDet(9);
        for(int j = 0; j < detected_bounding_boxes.getSize(); ++j)
        {
            oneDet(0) = cnt;
            oneDet(1) = j;
            oneDet(2) = 1;
            oneDet(3) = 1 - detected_bounding_boxes(j)(4)+1; // make sure that the score is always positive
            oneDet(4) = detected_bounding_boxes(j)(0);
            oneDet(5) = detected_bounding_boxes(j)(1);
            oneDet(6) = detected_bounding_boxes(j)(2);
            oneDet(7) = detected_bounding_boxes(j)(3);
            oneDet(8) = detected_bounding_boxes(j)(5);
            //OutputHOGdetL.pushBack(oneDet);																//Modified
        }

        tracker.process_tracking_oneFrame(HyposAll, det_comb, cnt, OutputHOGdetL, cnt_image, cameras(current_frame));*/																//Modified

        ////////////////////////////////////////////////////////////////


        if(Globals::export_bounding_box)
        {
            // without tracking
            WriteToFile(detected_bounding_boxes,current_frame+Globals::nOffset,*det_file,multiply_by_3);																//Modified

            /*// with tracking																																			//Modified
            Vector<Vector<double> > bboxes;
            exportBBOX(tracker.HyposMDL, cameras(current_frame), current_frame, bboxes);
            WriteToFile(bboxes,current_frame+Globals::nOffset,*det_file,multiply_by_3,false);*/
        }

        if(Globals::export_result_images)
        {
            //without tracking
            for(int jj = 0; jj < detected_bounding_boxes.getSize(); jj++)
            {
                detected_bounding_boxes(jj)(3)/=3;
                RenderBBox2D(detected_bounding_boxes(jj), images[current_frame], 255, 0, 0);
            }

            /*// with tracker																																//Modified
            Vector<Vector<double> > bboxes;
            exportBBOX(tracker.HyposMDL, cameras(current_frame), current_frame, bboxes);
            for(int jj = 0; jj < bboxes.getSize(); jj++)
            {
                bboxes(jj)(3)/=3.0;
                RenderBBox2D(bboxes(jj), images[current_frame], 255, 0, 0);
            }*/

            char path[128];
            sprintf(path, Globals::result_images_path.c_str(), current_frame + Globals::nOffset);
            cim_out.draw_image(images(current_frame));
            cim_out.draw_image(Globals::dImWidth,cnt_image);
            cim_out.save(path);
        }
    }
    if(Globals::export_bounding_box)
    {
        det_file->close();
    }

    // Timing Code - End
    time(&user_time_end);
    cpu_time_end = CPUTime();
    cout << "TIMING :" << cpu_time_end-cpu_time_start << "s (system), "
         << user_time_end-user_time_start << "s (user)" << endl;

    delete detector;
    delete detector_seg;

    return 0;
}

namespace riddle {

class KinectPersonDetector {

public:
    KinectPersonDetector()
        :it_(_nh),_rgb_image_topic("color_image"), _depth_image_topic("depth_image")
        //as_(_nh, "find_person", boost::bind(&KinectPersonDetector::executeFindPerson, this, _1), false),
        //action_name_("find_person")
    {

        //subsribe to the topics
        _rgb_image_sub = it_.subscribe(_rgb_image_topic, 1,
                                       &KinectPersonDetector::rgbImageReader, this);


        _depth_image_sub = it_.subscribe(_depth_image_topic, 1,
                                         &KinectPersonDetector::depthMapReader, this);

        _img_res_pub = it_.advertise("/kinect_person_detector/image", 1);
        _people_list_pub = _nh.advertise<riddle::DetectedPersonsList>("/kinect_person_detector/detected_people", 1);


        DetectionTrackingSystem::_this = &this->v;

        b_image_g = new unsigned char[Globals::dImWidth*Globals::dImHeight*3];
        b_depth_g = new float[Globals::dImWidth*Globals::dImHeight];

        //match_threshold = 10;																					//Modified
        //tracker_ctr = 0;																					//Modified

        local_data_ready = false;

        this->stop();
        //as_.start(); //start actions server


        total_processed_frames_ctr = 0;
        this->start();
    }

    int total_processed_frames_ctr;

    ~KinectPersonDetector();
    void depthMapReader(const sensor_msgs::ImageConstPtr& msg);
    void rgbImageReader(const sensor_msgs::ImageConstPtr& msg);
    void checkAndProceed2Detection()
    {
        printf("in checkAndProceed2Detection [%d %d %d] : ",
               (int)is_rgb_filled, (int)is_depth_filled, (int)local_data_ready);  fflush(stdout);
        if(is_rgb_filled && is_depth_filled && !local_data_ready)
        {
            printf("frame [%d] ", total_processed_frames_ctr);
            total_processed_frames_ctr++;
            is_rgb_filled = false;
            is_depth_filled = false;
            v.main_process(b_image_g, b_depth_g, Globals::dImWidth, Globals::dImHeight);

            //publish results
            _img_res_pub.publish(v.get_img());
            _people_list_pub.publish(v.get_people_list());

            //publish person position as tf

            //            _transform.setOrigin( tf::Vector3(1, 0, 3) );
            //            _transform.setRotation( tf::Quaternion(0, 0, 0) );
            //            _br.sendTransform(tf::StampedTransform(_transform, ros::Time::now(),
            //                                                   "camera_depth_optical_frame", "person1"));
            /*
            if (v.is_useful_person_det())
            {
                const std::vector<double> & p = v.getPersonPosition();
                double X, Y, Z;


                float f = 525.;
                float cx = 320; float cy = 240;
                Z = p[2];
                X = (p[0] - cx)*Z/f;
                Y = (p[1] - cy)*Z/f;

                float dist = sqrt((p[0]- prev_x)*(p[0]- prev_x) + (p[1]- prev_y)*(p[1]- prev_y));

                prev_x = p[0];
                prev_y = p[1];

                if (dist < match_threshold)
                    tracker_ctr++;
                else
                    tracker_ctr = 0;

                //_transform.setOrigin( tf::Vector3(X, Y, Z) );
                //_transform.setRotation( tf::Quaternion(0, 0, 0) );
                //_br.sendTransform(tf::StampedTransform(_transform, ros::Time::now(),
                //                                                       "/camera_depth_optical_frame", "person1"));
                // std::cout<<" v.getPersonPosition "<<p[0]<<" "<<p[1]<<" "<<p[2]<<std::endl;
                //                _transform.setOrigin( tf::Vector3(X, Y, Z) );
                //                _transform.setRotation( tf::Quaternion(0, 0, 0) );
                //                _br.sendTransform(tf::StampedTransform(_transform, ros::Time::now(),
                //                                                                       "/head_mount_kinect_rgb_link", "person1"));

                //if(v.is_tracking_started())
                if(tracker_ctr > 3)
                {
                    v.set_tracking_started();

                    std::cout<<" TRACKING OFFICIALLY STARTED: STOP HEAD MOTION!"<<std::endl;
                    std::cout<<" STOPPING SERVICE!"<<std::endl;

                    std::cout<<" X Y Z "<<X<<" "<<Y<<" "<<Z<<std::endl;
                    //this->stop();
                    std::vector<double> & pos = v.getServicePosition();
                    pos.resize(3);
                    pos[0] = X;
                    pos[1] = Y;
                    pos[2] = Z;

                    _transform.setOrigin( tf::Vector3(X, Y, Z) );
                    _transform.setRotation( tf::Quaternion(0, 0, 0) );
                    _br.sendTransform(tf::StampedTransform(_transform, ros::Time::now(),
                                                           "/head_mount_kinect_rgb_optical_frame", "person1"));

                    local_data_ready = true;
                }
            }
            */
            //const cv::Mat & img = v.get_opencv_img();
            //use image transport and publish
            //            transform.setOrigin( tf::Vector3(0.0, 2.0, 0.0) );
            //               transform.setRotation( tf::Quaternion(0, 0, 0) );
            //               br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "turtle1", "carrot1")
        }

        printf("\n"); fflush(stdout);
    }

    void executeFindPerson(const riddle::FindPersonGoalConstPtr &goal)
    {
        /*
        std::cout<<" Function executeFindPerson..."<<std::endl;
        // helper variables
        ros::Rate r(10);
        bool success = true;

        feedback_.currentCnt = v.get_tracking_update_ctr();

        v.set_tracking_update_thresh(goal->requiredCnt);
        this->tracker_ctr = 0;
        // publish info to the console for the user
        //ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);
        this->start();

        // start executing the action
        while(!v.is_tracking_started())
        {
            // check that preempt has not been requested by the client
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                // set the action state to preempted
                as_.setPreempted();
                success = false;
                break;
            }
            feedback_.currentCnt = v.get_tracking_update_ctr();

            // publish the feedback
            as_.publishFeedback(feedback_);

            r.sleep();
        }

        if(success)
        {
            while(!local_data_ready)
                r.sleep();

            this->stop();
            std::vector<double> & pos = v.getServicePosition();
            result_.position[0] = pos[0];
            result_.position[1] = pos[1];
            result_.position[2] = pos[2];

            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            as_.setSucceeded(result_);

            local_data_ready = false;
        }
        */
    }

    bool start()     {   is_rgb_filled = false;  is_depth_filled = false;   det_status =true; v.reset_counters();}
    bool stop()     {    is_rgb_filled = true;  is_depth_filled = true;     det_status =false; }
    bool is_on()    {   return det_status;  }

protected:
    ros::NodeHandle _nh;

    //published topics
    ros::Publisher _pub_pos;
    ros::Publisher _pub_track;

    image_transport::ImageTransport it_;
    image_transport::Subscriber _rgb_image_sub;
    image_transport::Subscriber _depth_image_sub;
    image_transport::Publisher _img_res_pub; //output image publisher
    ros::Publisher _people_list_pub; //output image publisher

    tf::TransformBroadcaster _br;
    tf::Transform _transform;

    //action server
    //FindPersonServer as_;

    //topic names (read)
    std::string _rgb_image_topic;
    std::string _depth_image_topic;

    //image data buffering variables
    unsigned char* b_image_g;// = new unsigned char[w*h*3];
    float* b_depth_g;// = new float[h*w];
    //memcpy(b_depth, depth, w*h*sizeof(float));
    DetectionTrackingSystem v;
    bool is_rgb_filled;
    bool is_depth_filled;

    bool det_status;
    bool local_data_ready;

    float prev_x;
    float prev_y;

    //float match_threshold;																					//Modified
    //int tracker_ctr;																					//Modified

    riddle::FindPersonFeedback feedback_;
    riddle::FindPersonResult result_;
    std::string action_name_;
};
}

using namespace riddle;

//KinectPersonDetector::KinectPersonDetector()


KinectPersonDetector::~KinectPersonDetector()
{
    delete [] b_image_g;
    delete [] b_depth_g;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kinect_person_detector");

    std::string data_path;
    {
        ros::NodeHandle nh_t("~");
        nh_t.param("data_path", data_path,
                   std::string("/home/aamekonn/catkin_ws/src/riddle/data/pdt/"));
        g_data_root_path = data_path;
    }

    ReadConfigFile(data_path);

    KinectPersonDetector pdt;

    //pdt.start();

    ros::spin();
    //    while(ros::ok())
    //    {
    //        //    while (ros::ok())
    //        //    {
    //        //      int c = getch();   // call your non-blocking input function

    //        //      if (c == 's')         pdt.start();
    //        //      else if (c == 'p')    pdt.stop();

    //        //      ros::spinOnce();
    //        //    }
    //        ros::spinOnce();

    return 0;
}


void KinectPersonDetector::depthMapReader(const sensor_msgs::ImageConstPtr& msg)
{
    if(is_depth_filled == true)
        return;

    cv_bridge::CvImagePtr cv_ptr;
    try
    {   cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Update GUI Window
    cv::Mat &img = cv_ptr->image;
    float *ptr_dt = (float *) img.data;
    for(long int i = 0; i < (img.rows*img.cols); i++)
        b_depth_g[i] = ptr_dt[i]/1000.;

    is_depth_filled = true;
    checkAndProceed2Detection();
}


void KinectPersonDetector::rgbImageReader(const sensor_msgs::ImageConstPtr& msg)
{
    if(is_rgb_filled == true)
        return;

    cv_bridge::CvImagePtr cv_ptr;
    try
    {   cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8); }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    memcpy((unsigned char*)b_image_g,
           (unsigned char*)cv_ptr->image.data,
           (cv_ptr->image.rows*cv_ptr->image.cols*3));

    is_rgb_filled = true;
    checkAndProceed2Detection();
}
