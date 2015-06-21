/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2011 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>

#include <GL/glut.h>

#include "riddle/libfreenect.h"
#include "riddle/libfreenect-audio.h"

#include <ros/ros.h>
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int32MultiArray.h"

#define SCALING 65536

/* TODO */
//1. Destructor to deallocate all allocated memories

namespace riddle {


namespace coreaudio {
pthread_t freenect_thread;
volatile int die = 0;

static freenect_context* f_ctx;
static freenect_device* f_dev;

typedef struct {
    int32_t* buffers[4];
    int max_samples;
    int current_idx;  // index to the oldest data in the buffer (equivalently, where the next new data will be placed)
    int new_data;
} capture;

capture state;

pthread_mutex_t audiobuf_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t audiobuf_cond = PTHREAD_COND_INITIALIZER;

void in_callback(freenect_device* dev, int num_samples,
                 int32_t* mic1, int32_t* mic2,
                 int32_t* mic3, int32_t* mic4,
                 int16_t* cancelled, void * unknown) {
    pthread_mutex_lock(&audiobuf_mutex);
    capture* c = (capture*)freenect_get_user(dev);
    if(num_samples < c->max_samples - c->current_idx) {
        memcpy(&(c->buffers[0][c->current_idx]), mic1, num_samples*sizeof(int32_t));
        memcpy(&(c->buffers[1][c->current_idx]), mic2, num_samples*sizeof(int32_t));
        memcpy(&(c->buffers[2][c->current_idx]), mic3, num_samples*sizeof(int32_t));
        memcpy(&(c->buffers[3][c->current_idx]), mic4, num_samples*sizeof(int32_t));
    } else {
        int first = c->max_samples - c->current_idx;
        int left = num_samples - first;
        memcpy(&(c->buffers[0][c->current_idx]), mic1, first*sizeof(int32_t));
        memcpy(&(c->buffers[1][c->current_idx]), mic2, first*sizeof(int32_t));
        memcpy(&(c->buffers[2][c->current_idx]), mic3, first*sizeof(int32_t));
        memcpy(&(c->buffers[3][c->current_idx]), mic4, first*sizeof(int32_t));
        memcpy(c->buffers[0], &mic1[first], left*sizeof(int32_t));
        memcpy(c->buffers[1], &mic2[first], left*sizeof(int32_t));
        memcpy(c->buffers[2], &mic3[first], left*sizeof(int32_t));
        memcpy(c->buffers[3], &mic4[first], left*sizeof(int32_t));
    }
    c->current_idx = (c->current_idx + num_samples) % c->max_samples;
    c->new_data = 1;
    pthread_cond_signal(&audiobuf_cond);
    pthread_mutex_unlock(&audiobuf_mutex);
}

void* freenect_threadfunc(void* arg) {
    while(!die && freenect_process_events(f_ctx) >= 0) {
        // If we did anything else in the freenect thread, it might go here.
    }
    freenect_stop_audio(f_dev);
    freenect_close_device(f_dev);
    freenect_shutdown(f_ctx);
    return NULL;
}

}

class RosKinectAudioCapture
{
public:
    RosKinectAudioCapture()
    {
        this->is_initialized = false;

        if (freenect_init(&riddle::coreaudio::f_ctx, NULL) < 0) {
            ROS_ERROR("freenect_init() failed\n");
            exit(0);
        }
        freenect_set_log_level(riddle::coreaudio::f_ctx, FREENECT_LOG_INFO);
        freenect_select_subdevices(riddle::coreaudio::f_ctx, FREENECT_DEVICE_AUDIO);

        int nr_devices = freenect_num_devices (riddle::coreaudio::f_ctx);
        ROS_INFO ("Number of devices found: %d\n", nr_devices);
        if (nr_devices < 1)
            exit(0);

        int user_device_number = 0;
        if (freenect_open_device(riddle::coreaudio::f_ctx, &riddle::coreaudio::f_dev, user_device_number) < 0) {
            ROS_ERROR("Could not open device\n");
            exit(0);
        }

        riddle::coreaudio::state.max_samples = 256 * 60;
        riddle::coreaudio::state.current_idx = 0;

        riddle::coreaudio::state.buffers[0] = (int32_t *) malloc(riddle::coreaudio::state.max_samples * sizeof(int32_t));
        riddle::coreaudio::state.buffers[1] = (int32_t *) malloc(riddle::coreaudio::state.max_samples * sizeof(int32_t));
        riddle::coreaudio::state.buffers[2] = (int32_t *) malloc(riddle::coreaudio::state.max_samples * sizeof(int32_t));
        riddle::coreaudio::state.buffers[3] = (int32_t *) malloc(riddle::coreaudio::state.max_samples * sizeof(int32_t));
        memset(riddle::coreaudio::state.buffers[0], 0, riddle::coreaudio::state.max_samples * sizeof(int32_t));
        memset(riddle::coreaudio::state.buffers[1], 0, riddle::coreaudio::state.max_samples * sizeof(int32_t));
        memset(riddle::coreaudio::state.buffers[2], 0, riddle::coreaudio::state.max_samples * sizeof(int32_t));
        memset(riddle::coreaudio::state.buffers[3], 0, riddle::coreaudio::state.max_samples * sizeof(int32_t));

        freenect_set_user(riddle::coreaudio::f_dev, &riddle::coreaudio::state);

        freenect_set_audio_in_callback(riddle::coreaudio::f_dev, riddle::coreaudio::in_callback);
        freenect_start_audio(riddle::coreaudio::f_dev);

        int res = pthread_create(&riddle::coreaudio::freenect_thread, NULL,
                                 &riddle::coreaudio::freenect_threadfunc, this);
        if (res) {
            ROS_ERROR("pthread_create failed\n");
            exit(0);
        }

        //some bitrate..FIXME
        _bitrate = 192;
        _lastBaseIndx=0;
        //the bit rate at which the audio is encoded
        ros::param::param<int>("~bitrate", _bitrate, 192);

        //initialize publisher
        _pub[0] = _nh.advertise<std_msgs::Int16MultiArray>("mic1_16b", 100);
        _pub[1] = _nh.advertise<std_msgs::Int16MultiArray>("mic2_16b", 100);
        _pub[2] = _nh.advertise<std_msgs::Int16MultiArray>("mic3_16b", 100);
        _pub[3] = _nh.advertise<std_msgs::Int16MultiArray>("mic4_16b", 100);
        _pub[4] = _nh.advertise<std_msgs::Int32MultiArray>("mic1_32b", 100);
        _pub[5] = _nh.advertise<std_msgs::Int32MultiArray>("mic2_32b", 100);
        _pub[6] = _nh.advertise<std_msgs::Int32MultiArray>("mic3_32b", 100);
        _pub[7] = _nh.advertise<std_msgs::Int32MultiArray>("mic4_32b", 100);
        

        

        this->is_initialized = true;
    }
    
    ~RosKinectAudioCapture()
    {
		
		riddle::coreaudio::die = 1;
		//pthread_kill(riddle::coreaudio::freenect_thread, sig);
		//pthread_exit(NULL);

		// All the default sigint handler does is call shutdown()
		ros::shutdown();
		
		free(riddle::coreaudio::state.buffers[0]);
		free(riddle::coreaudio::state.buffers[1]);
		free(riddle::coreaudio::state.buffers[2]);
		free(riddle::coreaudio::state.buffers[3]);
	}

    void publish(const std_msgs::Int16MultiArray msg16b[], const std_msgs::Int32MultiArray msg32b[])
    {
		for (int i=0; i<4 ; i++)
		{
			_pub[i].publish(msg16b[i]);
			_pub[i+4].publish(msg32b[i]);
		}
	}

    void readAndPublishBuffer()
    {
        pthread_mutex_lock(&riddle::coreaudio::audiobuf_mutex);

        while(!riddle::coreaudio::state.new_data)
            pthread_cond_wait(&riddle::coreaudio::audiobuf_cond, &riddle::coreaudio::audiobuf_mutex);

        riddle::coreaudio::state.new_data = 0;
        
        int _buffSize = riddle::coreaudio::state.current_idx - _lastBaseIndx;
        for (int mic=0 ; mic<4 ; mic++)
        {
			if (_buffSize > 0) //if we read the buffer normally
			{
				_mic16b[mic].data.resize(_buffSize);
				_mic32b[mic].data.resize(_buffSize);
				int j=0;
				for (int i=_lastBaseIndx ; i<riddle::coreaudio::state.current_idx ; i++, j++)
				{
					_mic32b[mic].data[j] = int32_t(riddle::coreaudio::state.buffers[mic][(i % riddle::coreaudio::state.max_samples)]);
					_mic16b[mic].data[j] = int16_t(_mic32b[mic].data[j]/SCALING);
				}
			}
			else
			{
				_buffSize = riddle::coreaudio::state.current_idx + riddle::coreaudio::state.max_samples - _lastBaseIndx;
				_mic16b[mic].data.resize(_buffSize);
				_mic32b[mic].data.resize(_buffSize);
				int j=0;
				for (int i=_lastBaseIndx ; i<riddle::coreaudio::state.max_samples ; i++, j++)
				{
					_mic32b[mic].data[j] = int32_t(riddle::coreaudio::state.buffers[mic][(i % riddle::coreaudio::state.max_samples)]);
					_mic16b[mic].data[j] = int16_t(_mic32b[mic].data[j]/SCALING);
				}
				for (int i=0 ; i<riddle::coreaudio::state.current_idx ; i++, j++)
				{
					_mic32b[mic].data[j] = int32_t(riddle::coreaudio::state.buffers[mic][(i % riddle::coreaudio::state.max_samples)]);
					_mic16b[mic].data[j] = int16_t(_mic32b[mic].data[j]/SCALING);
				}
			}
		}
		_lastBaseIndx = riddle::coreaudio::state.current_idx;
        
        pthread_mutex_unlock(&riddle::coreaudio::audiobuf_mutex);
        
        this->publish(this->_mic16b, this->_mic32b);
    }

private:
    ros::NodeHandle _nh;
    ros::Publisher _pub[8];

    int _bitrate; //in case
    bool is_initialized;
    std_msgs::Int16MultiArray _mic16b[4];
    std_msgs::Int32MultiArray _mic32b[4];
    int _lastBaseIndx;
};

}

void mySigintHandler(int sig)
{
    // Do some custom action.
    riddle::coreaudio::die = 1;
    pthread_kill(riddle::coreaudio::freenect_thread, sig);
    pthread_exit(NULL);

    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinect_audio_publisher"); //last argument could be argv[0]

    ROS_INFO("Initializing kinect audio data publisher (based on adaptation of libfreenect demo...");

    riddle::RosKinectAudioCapture server; //all initialization will be carried out here

    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, mySigintHandler);

    ROS_INFO("...init done!");

    while (ros::ok) {
        server.readAndPublishBuffer();
        ros::spinOnce();
    }


    return 0;
}
