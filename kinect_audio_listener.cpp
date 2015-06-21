#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>

#include <GL/glut.h>

#include <ros/ros.h>
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int16MultiArray.h"

/* TODO */
//1. Encapsulate all these functionalities in a cpp class
//2. Break down the application to publisher and listener
//capture state;

int win_h, win_w;

void Reshape(int w, int h) {
    win_w = w;      win_h = h;
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0, (float)w, (float)h, 0.0, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void Keyboard(unsigned char key, int x, int y) {
    if(key == 'q') {
        exit(0);
    }
}

namespace riddle {


class RosKinectAudioListener
{
public:
    RosKinectAudioListener():_input_message_name("/audioBuffer32")
    {
		
		std::string nodeName = _nh.resolveName("kinect_audio_listener", false);
        //some bitrate..FIXME
        _bitrate = 192;
        //the bit rate at which the audio is encoded
        //TODO get the bitrate from the global params list as
        //set by the publisher
        //ros::param::param<int>("~bitrate", _bitrate, 192);

        //initialize KinectAudop subscriber
        _sub1 = _nh.subscribe(_input_message_name, 100,
                             &RosKinectAudioListener::readAudioBuffer32, this);
        _sub2 = _nh.subscribe("/audioBuffer16", 100,
                             &RosKinectAudioListener::readAudioBuffer16, this);

        //initialize drawing window
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_ALPHA );
        glutInitWindowSize(800, 600);
        glutInitWindowPosition(0, 0);
        glutCreateWindow(nodeName.c_str());
        glClearColor(0.0, 0.0, 0.0, 0.0);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        Reshape(800, 600);
        glutReshapeFunc(Reshape);
        glutKeyboardFunc(Keyboard);
        
        _displayLength=256*1500;
        _x=0;
    }
    
    ~RosKinectAudioListener()
    {
	}

    //get the kinectAudio data and plot it
    void readAudioBuffer32(const std_msgs::Int32MultiArray &msg)
    {
        // Draw:
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        float xIncr = (float)win_w / (_displayLength);
        //int base_idx = msg.base_indx;

        int i, mic;
        glBegin(GL_LINE_STRIP);
        glColor4f(1.0f, 1.0f, 1.0f, 0.7f);
        for(i = 0; i < msg.data.size(); i++) {
                glVertex3f(_x, (float)win_h/2. + (float)win_h/2.*(float)(msg.data[i])/2147483648. , 0);
                _x += xIncr;
                if (_x>=win_w)
                {
					
					_x=0;
					glEnd();
					glClear(GL_COLOR_BUFFER_BIT);
					glBegin(GL_LINE_STRIP);
				}
            }
            glEnd();
        glutSwapBuffers();
    }
    
    void readAudioBuffer16(const std_msgs::Int16MultiArray &msg)
    {
        // Draw:
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        float xIncr = (float)win_w / (_displayLength);
        //int base_idx = msg.base_indx;

        int i, mic;
        glBegin(GL_LINE_STRIP);
        glColor4f(1.0f, 1.0f, 1.0f, 0.7f);
        for(i = 0; i < msg.data.size(); i++) {
                glVertex3f(_x, (float)win_h/2. + (float)win_h/2.*(float)(msg.data[i])/32768. , 0);
                _x += xIncr;
                if (_x>=win_w)
                {
					
					_x=0;
					glEnd();
					glClear(GL_COLOR_BUFFER_BIT);
					glBegin(GL_LINE_STRIP);
				}
            }
            glEnd();
        glutSwapBuffers();
    }

private:
    ros::NodeHandle _nh;
    ros::Subscriber _sub1, _sub2;

    int _bitrate; //in case
    std::string _input_message_name;
    int _displayLength;
    float _x;
};

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinect_audio_listener"); //last argument could be argv[0]

    //init glut env for visualization
    glutInit(&argc, argv);

    ROS_INFO("Initializing kinect audio data listener (based on libfreenect demo...\n");

    riddle::RosKinectAudioListener listener;

    ROS_INFO("...init done!");

    ros::spin();

    return 0;
}
