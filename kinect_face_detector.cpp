#include <ros/ros.h>


#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/PointCloud2.h"
#include "image_transport/image_transport.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <people_msgs/PositionMeasurement.h>
#include <people_msgs/PositionMeasurementArray.h>

//#include <pcl_ros/point_cloud.h>
//#include <pcl/point_types.h>


#include <sys/time.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <stdio.h>

#include <cv.h>
#include <highgui.h>

//message topic headers
#include "riddle/DetectedPersonsList.h"


#include <riddle/FindPersonAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<riddle::FindPersonAction> FindFaceServer;
namespace riddle {

class KinectFaceDetector {

public:
    KinectFaceDetector()
        :it_(_nh),_rgb_image_topic("/camera/rgb/image_color"), _face_det_topic("/face_detector/people_tracker_measurements_array"),
          as_(_nh, "find_face", boost::bind(&KinectFaceDetector::executeFindFace, this, _1), false),
          action_name_("find_face")
    {

        //subsribe to the topics
        _face_subscrb = _nh.subscribe(_face_det_topic, 1,
                                      &KinectFaceDetector::readDetectedFaces, this);

        _rgb_image_sub = it_.subscribe(_rgb_image_topic, 1,
                                       &KinectFaceDetector::rgbImageReader, this);

        _img_res_pub = it_.advertise("/kinect_person_detector/image", 1);
        _people_list_pub = _nh.advertise<riddle::DetectedPersonsList>("/kinect_person_detector/detected_people", 1);

        match_threshold = 10;
        tracker_ctr = 0;

        cx = 322.515987; cy = 259.055966; fx = 521.179233; fy = 493.033034;

        this->reset_counters();

        tracking_update_thresh = 3;

        position.resize(3);
        std::cout<<"initialization done!"<<std::endl;

        this->stop();
        as_.start(); //start actions server
    }

    ~KinectFaceDetector()
    {

    }

    void publishAsTfFrame(const std::vector<double> & pos)
    {
        _transform.setOrigin( tf::Vector3(pos[0], pos[1], pos[2]) );
        _transform.setRotation( tf::Quaternion(0, 0, 0) );
        _br.sendTransform(tf::StampedTransform(_transform, ros::Time::now(),
                                               "/head_mount_kinect_rgb_optical_frame", "face1"));
        //"/head_mount_kinect_rgb_optical_frame", "face1"));
    }

    void readDetectedFaces(const people_msgs::PositionMeasurementArray::ConstPtr& msg)
    {
        if (!det_status)
            return;

        printf(" Got the following faces: ");

        int closest_indx = -1;
        double close_target = 100.;

        for (size_t i = 0; i < msg->people.size(); i++)
        {
            people_msgs::PositionMeasurement person = msg->people[i];

            printf("--> at [%lf %lf %lf ] reliability [%f] \n",
                   person.pos.x,
                   person.pos.y,
                   person.pos.z,
                   person.reliability);
            if (person.pos.z < close_target)
            {
                closest_indx = i;
                close_target = person.pos.z;
            }
        }

        if (closest_indx != -1 )
        {
            printf("--> closest_indx [%d] : people_size [%d] \n", closest_indx, (int)msg->people.size());   fflush(stdout);

            this->closestPerson = msg->people[closest_indx];
            this->update_track(this->closestPerson);
        }
        fflush(stdout);

        was_face_detected = true;
    }

    void rgbImageReader(const sensor_msgs::ImageConstPtr& msg);

    void executeFindFace(const riddle::FindPersonGoalConstPtr &goal)
    {

        std::cout<<" Function executeFindPerson..."<<std::endl;

        // helper variables
        ros::Rate r(10);
        bool success = true;

        feedback_.currentCnt = this->tracking_update_ctr;

        tracking_update_thresh = goal->requiredCnt;

        this->start();

        this->rgb_image_loaded = false;
        
        double u, v, px, py, pz, dim;
        std::vector<double> frame_pos;
        frame_pos.resize(3,0.);
        // start executing the action
        while(!this->tracking_started)
        {
            printf("looping...\n"); fflush(stdout);
            if (this->rgb_image_loaded)
            {
                if (this->was_face_detected)
                {
                    px = this->closestPerson.pos.x;
                    py = this->closestPerson.pos.y;
                    pz = this->closestPerson.pos.z;

                    u = (px*fx/pz) + cx;
                    v = (py*fy/pz) + cy;

                    dim = fx*.4/pz;

                    cv::Rect r;
                    r.x = u - dim*0.5;
                    r.y = v - dim*0.42;
                    r.width = dim;
                    r.height = dim;

                    /* visualization
                    */
                    //draw face
                    cv::rectangle(cv_ptr->image, r, CV_RGB(0,255,0), 2);

                    frame_pos[0] = px;    frame_pos[1] = py;    frame_pos[2] = pz;

                    this->publishAsTfFrame(frame_pos);
                    this->was_face_detected = false;
                }

                //cv::imshow("IMG", cv_ptr->image);
                //cv::waitKey(50);
                this->rgb_image_loaded = false;

                //publish resulting image
                printf("publishing image...\n"); fflush(stdout);
                //_img_res_pub.publish(cv_ptr->toImageMsg());
            }

            // check that preempt has not been requested by the client
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                // set the action state to preempted
                as_.setPreempted();
                success = false;
                break;
            }
            feedback_.currentCnt = this->tracking_update_ctr;

            // publish the feedback
            printf("publishing feedback...\n"); fflush(stdout);
            as_.publishFeedback(feedback_);

            this->was_face_detected = false;

            ros::spinOnce();
            r.sleep();
        }

        this->stop();

        if(success)
        {
            //std::vector<double> & pos = v.getServicePosition();
            result_.position[0] = this->closestPerson.pos.x;
            result_.position[1] = this->closestPerson.pos.y;
            result_.position[2] = this->closestPerson.pos.z;

            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            as_.setSucceeded(result_);

            px = this->closestPerson.pos.x;
            py = this->closestPerson.pos.y;
            pz = this->closestPerson.pos.z;

            u = (px*fx/pz) + cx;
            v = (py*fy/pz) + cy;

            dim = fx*.4/pz;

            cv::Rect r;
            r.x = u - dim*0.5;
            r.y = v - dim*0.42;
            r.width = dim;
            r.height = dim;

            //cv::rectangle(cv_ptr->image, r, CV_RGB(0,255,0), 2);
            /* visualization
             */
            //draw face
            frame_pos[0] = px;    frame_pos[1] = py;    frame_pos[2] = pz;
            this->publishAsTfFrame(frame_pos);

            //cv::imshow("IMG", cv_ptr->image);
            //cv::waitKey(50);

            //publish resulting image
            //_img_res_pub.publish(cv_ptr->toImageMsg());
        }
    }

    bool start()     {   this->reset_counters(); det_status =true;}
    bool stop()     {    det_status =false; }
    bool is_on()    {   return det_status;  }

    void reset_counters()   {
        tracking_update_ctr = 0;
        useful_person_detected = false;
        tracking_started = false;
        is_first_face = true;
        was_face_detected = false;
    }

    void update_track(people_msgs::PositionMeasurement & p) {

        if (is_first_face)
        {
            prev_person = p;
            is_first_face = false;
            return;
        }

        double dist;

        dist = sqrt( (p.pos.x - prev_person.pos.x)*(p.pos.x - prev_person.pos.x) +
                     (p.pos.y - prev_person.pos.y)*(p.pos.y - prev_person.pos.y) +
                     (p.pos.z - prev_person.pos.z)*(p.pos.z - prev_person.pos.z)    );

        if (dist < 0.4)
            this->tracking_update_ctr++;
        else
            this->tracking_update_ctr = 0;

        prev_person = p;

        printf (":: Updating Track with this position :: ");

        printf("--> [%lf %lf %lf ] reliability [%f] \n",
               p.pos.x, p.pos.y, p.pos.z, p.reliability);
        fflush(stdout);

        printf("--> counter [%d :: %d ] thresh : dist [%lf] \n",
               this->tracking_update_ctr, this->tracking_update_thresh, dist);

        if (this->tracking_update_ctr > this->tracking_update_thresh)
        {
            this->tracking_started = true;
            this->det_status = false;
            printf (":: Hurray!!! Tracking officially started!!!:: \n\n");
        }
        fflush(stdout);

        prev_person = p;
    }

protected:
    ros::NodeHandle _nh;

    ros::Subscriber _face_subscrb;
    image_transport::ImageTransport it_;
    image_transport::Subscriber _rgb_image_sub;
    image_transport::Publisher _img_res_pub; //output image publisher
    ros::Publisher _people_list_pub; //output image publisher

    tf::TransformBroadcaster _br;
    tf::Transform _transform;

    //action server
    FindFaceServer as_;

    //input topic names (read)
    std::string _rgb_image_topic;
    std::string _face_det_topic;


    bool det_status;

    //float prev_pos[3];
    //float prev_x;
    //float prev_y;

    float match_threshold;
    int tracker_ctr;

    riddle::FindPersonFeedback feedback_;
    riddle::FindPersonResult result_;
    std::string action_name_;

    people_msgs::PositionMeasurement closestPerson;
    people_msgs::PositionMeasurement prev_person;

    //sensor_msgs::Image img_vis;
    cv_bridge::CvImagePtr cv_ptr;
    riddle::DetectedPersonsList detList;

    std::vector<double> position;
    std::vector<double> servicePosition;
    bool useful_person_detected;

    double cx, cy, fx, fy;

    bool rgb_image_loaded;
    bool was_face_detected;
    bool is_first_face;
    bool tracking_started;
    int tracking_update_ctr;
    int tracking_update_thresh;
};
}

using namespace riddle;

void KinectFaceDetector::rgbImageReader(const sensor_msgs::ImageConstPtr & msg)
{
    if (!det_status)
        return;

    try
    {
        this->cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    rgb_image_loaded = true;
    //    checkAndProceed2Detection();
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "kinect_face_detector");

    std::string data_path;
    {
        ros::NodeHandle nh_t("~");
        //nh_t.param("data_path", data_path,
        //           std::string("/home/aamekonn/catkin_ws/src/riddle/data/pdt/"));
        //g_data_root_path = data_path;
    }

    KinectFaceDetector pdt;

    //pdt.start();

    ros::spin();

    return 0;
}

