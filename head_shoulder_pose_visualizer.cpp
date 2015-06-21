#include <iostream>
#include <string>

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

//#include "riddle/HeadPoseList.h"
#include "riddle/HeadShoulderPoseList.h"


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

//opengl window size
int w,h;

bool g_first_rigid = true;
bool g_draw_triangles = false;
int g_max_z = 5000;
//for interactive visualization
gl_camera g_camera;

math_vector_3f g_face_curr_dir, g_face_dir(0,0,-1);


namespace riddle {

// ##############################################################################
void HPKeyEvent(unsigned char _k, int, int) {

    switch(_k) {

    case 't': {

        g_draw_triangles = !g_draw_triangles;
        ROS_INFO("toggled triangles %d ", g_draw_triangles);
        break;

    }
    case 'h':{

        ROS_INFO("\nAvailable command(s):");
        //            printf("\t 's' : toggle votes display \n");
        ROS_INFO("\t 't' : toggle triangles display");
        //            printf("\t '+' : increase stride \n");
        //            printf("\t '-' : decrease stride \n");
        //            printf("\t '*' : increase head threshold \n");
        //            printf("\t '/' : decrease head threshold \n");

        break;
    }
    default:
        break;
    }
    glutSwapBuffers();

}

// ##############################################################################
void HPResize(int _w, int _h) {
    w = _w;
    h = _h;
    ROS_INFO("Window resize");
}

// ##############################################################################
void HPmm(int x, int y)
{
    y = h-y;
    g_camera.mouse_move(x,y);

}

// ##############################################################################
void HPmb(int button, int state, int x, int y)
{
    y = h-y;

    Mouse::button b = Mouse::NONE;

    if(button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {

        b = Mouse::ROTATE;
        g_camera.mouse(x,y,b);

    }
    else if(button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN) {

        b = Mouse::MOVEXY;
        g_camera.mouse(x,y,b);
    }
    else if((button & 3) == 3) {

        g_camera.mouse_wheel(20);
    }
    else if ((button & 4) == 4) {

        g_camera.mouse_wheel(-20);
    }
}

void HPDrawCylinder( const math_vector_3f& p1,
                     const math_vector_3f& p2 ,
                     float radius, GLUquadric *quadric)
{
    math_vector_3f d = p2 - p1;
    if (d[2] == 0)
        d[2] = .0001f;

    float n = length(d);
    float ax = ( d[2] < 0.0 ) ? -57.295779f*acos( d[2]/n ) : 57.295779f*acos( d[2]/n );

    glPushMatrix();

    glTranslatef( p1[0],p1[1],p1[2] );
    glRotatef( ax, -d[1]*d[2], d[0]*d[2], 0.0);
    gluQuadricOrientation(quadric,GLU_OUTSIDE);
    gluCylinder(quadric, radius, radius, n, 10, 1);

    gluQuadricOrientation(quadric,GLU_INSIDE);
    gluDisk( quadric, 0.0, radius, 10, 1);
    glTranslatef( 0,0,n );

    gluQuadricOrientation(quadric,GLU_OUTSIDE);
    gluDisk( quadric, 0.0, radius, 10, 1);
    glPopMatrix();
}

class HeadShoulderPoseVisualiser
{
public:
    HeadShoulderPoseVisualiser();
    ~HeadShoulderPoseVisualiser();


    void depthMapReader(const sensor_msgs::PointCloud2ConstPtr& msg);
    void headShoulderPoseReader(const riddle::HeadShoulderPoseList & msg);
    void filteredHeadShoulderPoseReader(const riddle::HeadShoulderPoseList & msg);
    void drawHeadPose();

    //    void drawCylinder( const math_vector_3f& p1,
    //                       const math_vector_3f& p2 ,
    //                       float radius, GLUquadric *quadric);
protected:
    cv::Mat g_im3D;
    //riddle::HeadPoseList headPoseList;
    riddle::HeadShoulderPoseList headShoulderPoseList;
    riddle::HeadShoulderPoseList filteredHsPoseList;

private:
    ros::NodeHandle _nh;
    ros::Subscriber _sub_depth;
    ros::Subscriber _sub_hpose;
    ros::Subscriber _sub_filtered_hpose;

    std::string _depth_image_topic;
    std::string _head_pose_topic;
    std::string _filtered_head_pose_topic;
};

}

void riddle::HeadShoulderPoseVisualiser::depthMapReader(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    PointCloud cloud;
    pcl::fromROSMsg(*msg, cloud);

    //g_cloud_frame = cloud.header.frame_id;
    //g_cloud_ready = true;

    //if(!g_head_depth_ready) return; [sort of like if face detection occured]

    //Mat img3D;
    g_im3D = cv::Mat::zeros(cloud.height, cloud.width, CV_32FC3);
    //img3D.create(cloud.height, cloud.width, CV_32FC3);

    int yMin, xMin, yMax, xMax;
    yMin = 0; xMin = 0;
    yMax =  g_im3D.rows; xMax =  g_im3D.cols;

    int valid_pixels = 0;

    //get 3D from depth
    for(int y = yMin ; y <  g_im3D.rows; y++) {
        cv::Vec3f* img3Di =  g_im3D.ptr<cv::Vec3f>(y);

        for(int x = xMin; x <  g_im3D.cols; x++) {
            pcl::PointXYZ p = cloud.at(x,y);

            //            for(int x = 0; x < g_im3D.cols; x++){

            //                float d = (float)g_depthMD(x,y);

            //                if ( d < g_max_z && d > 0 ){

            //                    valid_pixels++;

            //                    Mi[x][0] = ( float(d * (x - 320)) / f );
            //                    Mi[x][1] = ( float(d * (y - 240)) / f );
            //                    Mi[x][2] = d;

            //                }
            //                else
            //                    Mi[x] = 0;

            //            }

            //WARNING! Please try to harmonize these steps
            //if((p.z>g_head_depth-0.2) && (p.z<g_head_depth+0.2))
            if (p.z < g_max_z && p.z > 0)
            {
                valid_pixels++;
                img3Di[x][0] = p.x*1000;
                img3Di[x][1] = p.y*1000;
                img3Di[x][2] = p.z*1000;//hypot(img3Di[x][0], p.z*1000); //they seem to have trained with incorrectly projected 3D points
                //img3Di[x][2] = p.z*1000;
            } else {
                img3Di[x] = 0;
            }
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

            //ROS_INFO("camera gravity : %f %f %f ", gravity[0],gravity[1],gravity[2]);
            g_camera.resetview( math_vector_3f(gravity[0],gravity[1],gravity[2]), maxDist );
            g_camera.rotate_180();
            g_first_rigid = false;
        }
    }
}

void riddle::HeadShoulderPoseVisualiser::headShoulderPoseReader(const riddle::HeadShoulderPoseList & msg)
{
    /* msg  - std_msgs/Header header
     *      - riddle/HeadPose[] headPoseLists - pose[6] x y z (in mm) teta phi psi (in radian)
     *      - riddle/HeadPose[] headSkeletonLists - pose[6] x y z (in m) teta phi psi (in radian)
     *      - float64[] shoulderPanLists
     */
    //read poses and draw ontop of this
    this->headShoulderPoseList = msg;

    //    ROS_INFO("Received headposelist - [%d] ", (int)msg.headPoseLists.size());

    //    for (int i = 0; i < (int) msg.headPoseLists.size(); i++)
    //    {
    //        riddle::HeadPose p = msg.headPoseLists[i];
    //        ROS_INFO(" [%d] position [%.3f %.3f %.3f] orient [%.3f %.3f %.3f]",
    //                 i, p.pose[0], p.pose[1], p.pose[2], p.pose[3], p.pose[4], p.pose[5]);
    //    }
    this->drawHeadPose();
}

void riddle::HeadShoulderPoseVisualiser::filteredHeadShoulderPoseReader(const riddle::HeadShoulderPoseList & msg)
{
    /* msg  - std_msgs/Header header
     *      - riddle/HeadPose[] headPoseLists - pose[6] x y z (in mm) teta phi psi (in radian)
     *      - riddle/HeadPose[] headSkeletonLists - pose[6] x y z (in m) teta phi psi (in radian)
     *      - float64[] shoulderPanLists
     */
    //read poses and draw ontop of this
    this->filteredHsPoseList = msg;

    //    ROS_INFO("Received headposelist - [%d] ", (int)msg.headPoseLists.size());

    //    for (int i = 0; i < (int) msg.headPoseLists.size(); i++)
    //    {
    //        riddle::HeadPose p = msg.headPoseLists[i];
    //        ROS_INFO(" [%d] position [%.3f %.3f %.3f] orient [%.3f %.3f %.3f]",
    //                 i, p.pose[0], p.pose[1], p.pose[2], p.pose[3], p.pose[4], p.pose[5]);
    //    }
    //this->drawHeadPose();
}

void riddle::HeadShoulderPoseVisualiser::drawHeadPose()
{
    //    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


    /* this->headShoulderPoseList
     *      - std_msgs/Header header
     *      - riddle/HeadPose[] headPoseLists - pose[6] x y z (in mm) teta phi psi (in radian)
     *      - riddle/HeadPose[] headSkeletonLists - pose[6] x y z (in m) teta phi psi (in radian)
     *      - float64[] shoulderPanLists            pan angle in degrees
     */
    //read poses and draw ontop of this


    glEnable(GL_NORMALIZE);
    glEnable(GL_DEPTH_TEST);

    //    g_camera.set_viewport(0,0,w,h);
    //    g_camera.setup();
    //    g_camera.use_light(true);

    glClearColor(1,1,1,1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_CULL_FACE);

    glPushMatrix();
    glColor3f(0.6f,0.6f,0.6f);

    using namespace cv;
    g_draw_triangles = false;
    if(g_draw_triangles){

        math_vector_3f d1,d2;
        glBegin(GL_TRIANGLES);

        for(int y = 0; y < g_im3D.rows-1; y++)
        {

            const Vec3f* Mi = g_im3D.ptr<Vec3f>(y);
            const Vec3f* Mi1 = g_im3D.ptr<Vec3f>(y+1);

            for(int x = 0; x < g_im3D.cols-1; x++){

                if( Mi[x][2] <= 0 || Mi1[x][2] <= 0 || Mi[x+1][2] <= 0 || Mi1[x+1][2] <= 0 )
                    continue;

                d1[0] = Mi[x][0] - Mi1[x][0];// v1 - v2;
                d1[1] = Mi[x][1] - Mi1[x][1];
                d1[2] = Mi[x][2] - Mi1[x][2];

                d2[0] = Mi[x+1][0] - Mi1[x][0];// v1 - v2;
                d2[1] = Mi[x+1][1] - Mi1[x][1];
                d2[2] = Mi[x+1][2] - Mi1[x][2];

                if ( fabs(d2[2])>20 || fabs(d1[2])>20 )
                    continue;

                math_vector_3f norm = cross_product(d2,d1);

                glNormal3f(norm[0],norm[1],norm[2]);
                glVertex3f(Mi[x][0],Mi[x][1],Mi[x][2]);
                glVertex3f(Mi1[x][0],Mi1[x][1],Mi1[x][2]);
                glVertex3f(Mi[x+1][0],Mi[x+1][1],Mi[x+1][2]);
                glVertex3f(Mi1[x][0],Mi1[x][1],Mi1[x][2]);
                glVertex3f(Mi1[x+1][0],Mi1[x+1][1],Mi1[x+1][2]);
                glVertex3f(Mi[x+1][0],Mi[x+1][1],Mi[x+1][2]);

            }
        }
        glEnd();

    }
    else {

        math_vector_3f d1,d2;
        glBegin(GL_POINTS);

        for(int y = 0; y < g_im3D.rows-1; y++)
        {

            const Vec3f* Mi = g_im3D.ptr<Vec3f>(y);
            const Vec3f* Mi1 = g_im3D.ptr<Vec3f>(y+1);

            for(int x = 0; x < g_im3D.cols-1; x++){

                if( Mi[x][2] <= 0 || Mi[x][2] <= 0 )
                    continue;

                d1[0] = Mi[x][0] - Mi1[x][0];// v1 - v2;
                d1[1] = Mi[x][1] - Mi1[x][1];
                d1[2] = Mi[x][2] - Mi1[x][2];

                d2[0] = Mi[x+1][0] - Mi1[x][0];// v1 - v2;
                d2[1] = Mi[x+1][1] - Mi1[x][1];
                d2[2] = Mi[x+1][2] - Mi1[x][2];

                math_vector_3f norm = cross_product(d2,d1);
                glNormal3f(norm[0],norm[1],norm[2]);
                glVertex3f(Mi[x][0],Mi[x][1],Mi[x][2]);

            }
        }
        glEnd();

    }


    glPopMatrix();


    GLUquadric* point = gluNewQuadric();
    GLUquadric *quadric = gluNewQuadric();
    gluQuadricNormals(quadric, GLU_SMOOTH);

    //draw head poses
    //if(g_means.size()>0){
    if(this->headShoulderPoseList.headPoseLists.size() > 0)
    {
        glColor3f( 0, 1, 0);
        //degree to rad conversion factor
        float mult = 0.0174532925f;

        for(unsigned int i=0;i<this->headShoulderPoseList.headPoseLists.size();++i){

            const riddle::HeadPose & p = this->headShoulderPoseList.headPoseLists[i];

            rigid_motion<float> rm;
            rm.m_rotation = euler_to_rotation_matrix( mult*p.pose[3], mult*p.pose[4], mult*p.pose[5] );
            math_vector_3f head_center( p.pose[0], p.pose[1], p.pose[2] );

            glPushMatrix();
            glTranslatef( head_center[0], head_center[1], head_center[2] );
            gluSphere( point, 10.f, 10, 10 );
            glPopMatrix();

            g_face_curr_dir = rm.m_rotation * (g_face_dir);
            math_vector_3f head_front(head_center + 150.f*g_face_curr_dir);

            riddle::HPDrawCylinder(head_center, head_front, 8, quadric);
        }

    }

    if(this->filteredHsPoseList.headPoseLists.size() > 0)
    {
        glColor3f( 0, 0, 1);
        //degree to rad conversion factor
        float mult = 0.0174532925f;

        for(unsigned int i=0;i<this->filteredHsPoseList.headPoseLists.size();++i){

            const riddle::HeadPose & p = this->filteredHsPoseList.headPoseLists[i];

            rigid_motion<float> rm;
            rm.m_rotation = euler_to_rotation_matrix( mult*p.pose[3], mult*p.pose[4], mult*p.pose[5] );
            math_vector_3f head_center( p.pose[0], p.pose[1], p.pose[2] );

            glPushMatrix();
            glTranslatef( head_center[0], head_center[1], head_center[2] );
            gluSphere( point, 10.f, 10, 10 );
            glPopMatrix();

            g_face_curr_dir = rm.m_rotation * (g_face_dir);
            math_vector_3f head_front(head_center + 200.f*g_face_curr_dir);

            riddle::HPDrawCylinder(head_center, head_front, 8, quadric);
        }

    }

    if(this->headShoulderPoseList.headSkeletonLists.size() > 0)
    {

        glColor3f( 1, 0, 0);
        //degree to rad conversion factor
        float mult = 0.0174532925f;
        unsigned int i = 0;

        const riddle::HeadPose & p = this->headShoulderPoseList.headSkeletonLists[i];
        rigid_motion<float> rm;
        rm.m_rotation = euler_to_rotation_matrix( p.pose[3], p.pose[4], p.pose[5] );

        //axis camera_frame vs camer_optical_frame [x' -> -y; y' -> -z; z' -> x]
        float pos[3];
        pos[0] = -p.pose[1]*1000.;
        pos[1] = -p.pose[2]*1000.;
        pos[2] = p.pose[0]*1000.;
        math_vector_3f head_center( pos[0], pos[1], pos[2]);

        glPushMatrix();
        glTranslatef( head_center[0], head_center[1], head_center[2] );
        gluSphere( point, 20.f, 10, 10 );
        glPopMatrix();

        g_face_curr_dir = rm.m_rotation * (g_face_dir);
        //math_vector_3f head_front(head_center + 150.f*g_face_curr_dir);
        //riddle::HPDrawCylinder(head_center, head_front, 8, quadric);
    }


    gluDeleteQuadric(point);
    gluDeleteQuadric(quadric);


    //glutSwapBuffers();
    //glutPostRedisplay();


    //    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    //    /* Setup cube vertex data. */
    //    v[0][0] = v[1][0] = v[2][0] = v[3][0] = -1;
    //    v[4][0] = v[5][0] = v[6][0] = v[7][0] = 1;
    //    v[0][1] = v[1][1] = v[4][1] = v[5][1] = -1;
    //    v[2][1] = v[3][1] = v[6][1] = v[7][1] = 1;
    //    v[0][2] = v[3][2] = v[4][2] = v[7][2] = 1;
    //    v[1][2] = v[2][2] = v[5][2] = v[6][2] = -1;

    //    /* Enable a single OpenGL light. */
    //    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    //    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    //    glEnable(GL_LIGHT0);
    //    glEnable(GL_LIGHTING);

    //    /* Use depth buffering for hidden surface elimination. */
    //    glEnable(GL_DEPTH_TEST);

    //    /* Setup the view of the cube. */
    //    glMatrixMode(GL_PROJECTION);
    //    gluPerspective( /* field of view in degree */ 40.0,
    //                    /* aspect ratio */ 1.0,
    //                    /* Z near */ 1.0, /* Z far */ 10.0);
    //    glMatrixMode(GL_MODELVIEW);
    //    gluLookAt(0.0, 0.0, 5.0,  /* eye is at (0,0,5) */
    //              0.0, 0.0, 0.0,      /* center is at (0,0,0) */
    //              0.0, 1.0, 0.);      /* up is in positive Y direction */

    //    /* Adjust cube position to be asthetic angle. */
    //    glTranslatef(0.0, 0.0, -1.0);
    //    glRotatef(60, 1.0, 0.0, 0.0);
    //    glRotatef(-20, 0.0, 0.0, 1.0);

    //    int i;

    //    for (i = 0; i < 6; i++) {
    //        glBegin(GL_QUADS);
    //        glNormal3fv(&n[i][0]);
    //        glVertex3fv(&v[faces[i][0]][0]);
    //        glVertex3fv(&v[faces[i][1]][0]);
    //        glVertex3fv(&v[faces[i][2]][0]);
    //        glVertex3fv(&v[faces[i][3]][0]);
    //        glEnd();
    //    }

    //    glutSwapBuffers();

    //    int i;

    //    for (i = 0; i < 6; i++) {
    //        glBegin(GL_QUADS);
    //        glNormal3fv(&n[i][0]);
    //        glVertex3fv(&v[faces[i][0]][0]);
    //        glVertex3fv(&v[faces[i][1]][0]);
    //        glVertex3fv(&v[faces[i][2]][0]);
    //        glVertex3fv(&v[faces[i][3]][0]);
    //        glEnd();
    //    }

    glutSwapBuffers();
    glutPostRedisplay();

    //    cv::imshow("IMG", g_im3D);
    //    cv::waitKey(10);
}



riddle::HeadShoulderPoseVisualiser::HeadShoulderPoseVisualiser()
    :_depth_image_topic("depth_msg"), _head_pose_topic("hs_msg"), _filtered_head_pose_topic("/filtered_hs_pose")
{
    _sub_depth = _nh.subscribe(_depth_image_topic, 1,
                               &HeadShoulderPoseVisualiser::depthMapReader, this);

    _sub_hpose = _nh.subscribe(_head_pose_topic, 1,
                               &HeadShoulderPoseVisualiser::headShoulderPoseReader, this);

    _sub_filtered_hpose =  _nh.subscribe(_filtered_head_pose_topic, 1,
                                       &HeadShoulderPoseVisualiser::filteredHeadShoulderPoseReader, this);
}

riddle::HeadShoulderPoseVisualiser::~HeadShoulderPoseVisualiser()
{

}

int main(int argc, char* argv[])
{
    glutInit(&argc, argv);
    glutInitWindowSize(800, 800);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutCreateWindow("head_pose_listener");

    //start a separate thread for glutMainLoop();

    //glutDisplayFunc(display);
    glMatrixMode(GL_PROJECTION);
    gluPerspective( /* field of view in degree */ 40.0,
                    /* aspect ratio */ 1.0,
                    /* Z near */ 100.0, /* Z far */ 10000.0);
    glMatrixMode(GL_MODELVIEW);
    gluLookAt(0.0, 0.0, 5000.0,  /* eye is at (0,0,5) */
              0.0, 0.0, 0.0,      /* center is at (0,0,0) */
              0.0, -1.0, 0.0);      /* up is in positive Y direction */

    ros::init(argc, argv, "head_pose_listener");

    // initialize GLUT
    //glutInit(&argc, argv);
    //    glutInit(&argc, argv);

    //    //glutInitWindowSize(800, 800);
    //    //glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    //    //glutInitWindowSize(800, 800);
    //    //glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    //    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    //    glutCreateWindow("red 3D lighted cube");
    //glutCreateWindow("head_pose_listener (press h for list of available commands)");

    //glClearColor(0.0, 0.0, 0.0, 0.0);
    //glEnable(GL_BLEND);
    //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    //glutDisplayFunc(riddle::HPDraw);
    //glutDisplayFunc(riddle::HPDraw);
    //glutMouseFunc(riddle::HPmb);
    //glutMotionFunc(riddle::HPmm);
    //glutKeyboardFunc(riddle::HPKeyEvent);
    //glutReshapeFunc(riddle::HPResize);


    ROS_INFO("Initializing head pose listener (based on fanelli demo...\n");

    riddle::HeadShoulderPoseVisualiser head_shoulder_pose_vis;

    ROS_INFO("...init done!");


    ros::spin();

    //glutMainLoop();
    //ros::spin();

    return 0;
}

/***
 *void Fusion::drawShoulders()
{
    for (int i=0 ; i<_leftShoulder.size() ; i++)
    {
        math_vector_3f dist = _leftShoulder[i] - _rightShoulder[i];
        float n = length(dist);
        float ax = ( dist[2] < 0.0 ) ? -57.295779f*acos( dist[2]/n ) : 57.295779f*acos( dist[2]/n );

        GLUquadric *quadric = gluNewQuadric();
        gluQuadricNormals(quadric, GLU_SMOOTH);

        glPushMatrix();

        //shoulder barre
        glColor3f( 1.f, 0, 0);
        glTranslatef(_rightShoulder[i][0], -_rightShoulder[i][1], _rightShoulder[i][2]);
        gluSphere( quadric, 20.f, 10, 10 );

        glRotatef( ax, -dist[1]*dist[2], dist[0]*dist[2], 0.0);
        gluCylinder(quadric, 8, 8, n, 10, 1);

        glTranslatef(0, 0, n);
        gluSphere( quadric, 20.f, 10, 10 );

        //shoulder direction
        glTranslatef(0, 0, -n/2.f);
        glRotatef(-90.f, 0, 1, 0);
        glColor3f( 0, 0, 1.0f);
        gluCylinder(quadric, 8, 8, 200, 10, 1);
        glColor3f( 1.f, 0, 0);
        glTranslatef(0, 0, 10);
        gluDisk(quadric, 0, 8, 10, 1);


        glPopMatrix();

        gluDeleteQuadric(quadric);
    }
}
***/
