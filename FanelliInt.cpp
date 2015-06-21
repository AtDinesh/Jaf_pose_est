#include "FanelliInt.h"

using namespace riddle;

FanelliInt::FanelliInt(std::string configFile)
{
    using namespace std;

    g_im_w = 640;
    g_im_h = 480;
    g_first_rigid = true;
    g_prob_th = 1.0f;

    this->loadConfig(configFile.c_str());
    this->_detector = new CRForestEstimator();
    if( !this->_detector->loadForest(g_treepath.c_str(), g_ntrees) ){

        cerr << "could not read forest!" << endl;
        exit(-1);
    }

    g_im3D.create(g_im_h,g_im_w,CV_32FC3);

}

FanelliInt::~FanelliInt()
{
    delete this->_detector;
}

void FanelliInt::init(XnUInt64 &focal_length, XnDouble &pixel_size)
{
    g_focal_length = focal_length;
    g_pixel_size = pixel_size;
}

cv::Mat FanelliInt::process(xn::DepthMetaData &metaData, std::vector< cv::Vec<float,POSE_SIZE> >& means)
{
    using namespace std;

    this->extract3DMap(metaData);//compute g_im3D

    g_means.clear();
    g_votes.clear();
    g_clusters.clear();

    //do the actual estimation
    this->_detector->estimate( 	g_im3D,
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

    //cout << g_means[0][0] << " " << g_means[0][1] << " " << g_means[0][2] << endl;

    return g_im3D;
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
        in >> dummy;
        in >> g_treepath;
        g_treepath = path_ + '/' + g_treepath;

        // Number of trees
        in >> dummy;
        in >> g_ntrees;

        in >> dummy;
        in >> g_maxv;

        in >> dummy;
        in >> g_larger_radius_ratio;

        in >> dummy;
        in >> g_smaller_radius_ratio;

        in >> dummy;
        in >> g_stride;

        in >> dummy;
        in >> g_max_z;

        in >> dummy;
        in >> g_th;


    } else {
        cerr << "File not found " << filename << endl;
        exit(-1);
    }
    in.close();

    cout << endl << "------------------------------------" << endl << endl;
    cout << "Estimation:       " << endl;
    cout << "Trees:            " << g_ntrees << " " << g_treepath << endl;
    cout << "Stride:           " << g_stride << endl;
    cout << "Max Variance:     " << g_maxv << endl;
    cout << "Max Distance:     " << g_max_z << endl;
    cout << "Head Threshold:   " << g_th << endl;

    cout << endl << "------------------------------------" << endl << endl;

    //    g_treepath = "/home/christophe/.local/share/nao/" + g_treepath;

}

void FanelliInt::extract3DMap(xn::DepthMetaData &metaData)
{
    float f = g_focal_length/g_pixel_size;
    int valid_pixels = 0;

    //generate 3D image
    for(int y = 0; y < g_im3D.rows; y++)
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
    }

    //this part is to set the camera position, depending on what's in the scene
    if (g_first_rigid ) {

        if( valid_pixels > 50000){ //wait for something to be in the image

            // calculate gravity center
            cv::Vec3f gravity(0,0,0);
            int count = 0;
            for(int y=0;y<g_im3D.rows;++y){
                const cv::Vec3f* Mi = g_im3D.ptr<cv::Vec3f>(y);
                for(int x=0;x<g_im3D.cols;++x){

                    if( Mi[x][2] > 0 ) {

                        gravity = gravity + Mi[x];
                        count++;
                    }
                }
            }

            float maxDist = 0;
            if(count > 0) {

                gravity = (1.f/(float)count)*gravity;

                for(int y=0;y<g_im3D.rows;++y){
                    const cv::Vec3f* Mi = g_im3D.ptr<cv::Vec3f>(y);
                    for(int x=0;x<g_im3D.cols;++x){

                        if( Mi[x][2] > 0 ) {

                            maxDist = MAX(maxDist,(float)norm( Mi[x]-gravity ));
                        }
                    }
                }
            }

            g_camera.resetview( math_vector_3f(gravity[0],gravity[1],gravity[2]), maxDist );
            g_camera.rotate_180();
            g_first_rigid = false;
        }
    }
}

gl_camera& FanelliInt::getCamera()
{
    return g_camera;
}
