#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "amppl/gpu_hog.h"
 
static const std::string OPENCV_WINDOW = "Image window";
 
class ImageConverter
{
  
 public:
   ImageConverter()
     : it_(nh_), in_image_topic_("input"), out_image_topic_("output")
   {
     // Subscrbe to input video feed and publish output video feed
     image_sub_ = it_.subscribe(in_image_topic_, 1, 
       &ImageConverter::imageCb, this);
     image_pub_ = it_.advertise(out_image_topic_, 1);
 
     cv::namedWindow(OPENCV_WINDOW);
   }
 
   ~ImageConverter()
   {
     cv::destroyWindow(OPENCV_WINDOW);
   }
 
   void imageCb(const sensor_msgs::ImageConstPtr& msg)
   {
     cv_bridge::CvImagePtr cv_ptr;
     try
     {
       cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
     catch (cv_bridge::Exception& e)
     {
       ROS_ERROR("cv_bridge exception: %s", e.what());
       return;
     }
 
     cv::cvtColor(cv_ptr->image, img_aux, CV_BGR2BGRA);
     std::vector<cv::Rect> found;
     p_detector.detectP(img_aux, found);

            for (size_t i = 0; i < found.size(); i++)
            {
                cv::Rect r = found[i];
                cv::Point p = cv::Point(r.x, r.y);

                cv::rectangle(cv_ptr->image, r.tl(), r.br(), CV_RGB(0, 255, 0), 3);
            }


     // Draw an example circle on the video stream
     //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
     // cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
 
     // Update GUI Window
     cv::imshow(OPENCV_WINDOW, cv_ptr->image);
     cv::waitKey(3);
     
     // Output modified video stream
     image_pub_.publish(cv_ptr->toImageMsg());
   }

protected:

   ros::NodeHandle nh_;
   image_transport::ImageTransport it_;
   image_transport::Subscriber image_sub_; //input image subscriber
   image_transport::Publisher image_pub_; //output image publisher

   std::string in_image_topic_; //default input
   std::string out_image_topic_; //default output

   //ros::Subscriber image_sub_; 
   //ros::Publisher image_pub_;
   gpu_hog p_detector;
   cv::Mat img_aux;
};
  
int main(int argc, char** argv)
{
   ros::init(argc, argv, "image_display");
   ImageConverter ic;
   ros::spin();
   return 0;
}
