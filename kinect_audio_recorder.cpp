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

#include <ros/ros.h>
#include "std_msgs/Int16MultiArray.h"

namespace riddle {

typedef struct {
FILE* logfiles[4];
int samples;
} capture;

#define SCALING 65536/4
#define TYPE int16_t



class RosKinectAudioRecorder
{
public:
    RosKinectAudioRecorder():_input_message_name("/android_audioBuffer16b")
    {
        //some bitrate..FIXME
        _bitrate = 192;
        
        this->init();

        _sub = _nh.subscribe(_input_message_name, 1,
                             &RosKinectAudioRecorder::readAudioBuffer, this);
    }
    
    ~RosKinectAudioRecorder()
    {
		fclose(file);
	}
    
    //init wav files header
    void init()
    {
		char wavheader[] = {
		0x52, 0x49, 0x46, 0x46, // ChunkID = "RIFF"
		0x00, 0x00, 0x00, 0x00, // Chunksize (will be overwritten later)
		0x57, 0x41, 0x56, 0x45, // Format = "WAVE"
		0x66, 0x6d, 0x74, 0x20, // Subchunk1ID = "fmt "
		0x10, 0x00, 0x00, 0x00, // Subchunk1Size = 16
		0x01, 0x00, 0x01, 0x00, // AudioFormat = 1 (linear quantization) | NumChannels = 1
		0x80, 0x3e, 0x00, 0x00, // SampleRate = 16000 Hz
		//0x80, 0xbb, 0x00, 0x00, // SampleRate = 48000 Hz
		0x00, 0x7d, 0x00, 0x00, // ByteRate = SampleRate * NumChannels * BitsPerSample/8 = 32000
		//0x01, 0x77, 0x00, 0x00, // ByteRate = SampleRate * NumChannels * BitsPerSample/8 = 64000
		0x02, 0x00, 0x10, 0x00, // BlockAlign = NumChannels * BitsPerSample/8 = 4 | BitsPerSample = 32
		0x64, 0x61, 0x74, 0x61, // Subchunk2ID = "data"
		0x00, 0x00, 0x00, 0x00, // Subchunk2Size = NumSamples * NumChannels * BitsPerSample / 8 (will be overwritten later)
		};

		_numSamples = 0;
		
		//go to end of files
		file = fopen("/home/christophe/record.wav", "wb");
		fwrite(wavheader, 1, 44, file);
	}
	
	//write buffers and update wav headers
	void updateWavFiles()
	{
		fseek(file, 0, SEEK_END);
		fwrite(_buffer, 1, _buffSize*sizeof(TYPE), file);
		
		_numSamples+=_buffSize;
		
		ROS_INFO("Number of samples : %d\n", _buffSize);
		
		this->updateWavHeader(file);
	}
	
	//update wav header
	void updateWavHeader(FILE* file)
	{
		char buf[4];
		fseek(file, 4, SEEK_SET);
		// Write ChunkSize = 36 + subchunk2size
		int chunksize = _numSamples * 2 + 36;
		buf[0] = (chunksize & 0x000000ff);
		buf[1] = (chunksize & 0x0000ff00) >> 8;
		buf[2] = (chunksize & 0x00ff0000) >> 16;
		buf[3] = (chunksize & 0xff000000) >> 24;
		fwrite(buf, 1, 4,file);

		fseek(file, 40, SEEK_SET);
		// Write Subchunk2Size = NumSamples * NumChannels (1) * BitsPerSample/8 (4)
		int subchunk2size = _numSamples * 2;
		buf[0] = (subchunk2size & 0x000000ff);
		buf[1] = (subchunk2size & 0x0000ff00) >> 8;
		buf[2] = (subchunk2size & 0x00ff0000) >> 16;
		buf[3] = (subchunk2size & 0xff000000) >> 24;
		fwrite(buf, 1, 4,file);
	}

    //get the kinectAudio data and plot it
    void readAudioBuffer(const std_msgs::Int16MultiArray &msg)
    {
		//real size of buffer
		_buffSize = msg.data.size();

		//if (_buffSize != 512) return;
		
		_buffer = new TYPE[_buffSize];
		int j=0;
		for (int i=0 ; i < msg.data.size() ; i++, j++)
		{
			_buffer[j] = TYPE(msg.data[i]);
		}
		
		
		//write data in wav files
		this->updateWavFiles();
		
		delete[] _buffer;
    }

private:
    ros::NodeHandle _nh;
    ros::Subscriber _sub;

    int _bitrate; //in case
    std::string _input_message_name;
    
    TYPE *_buffer;
    int _buffSize; //buffer size (512 with freenect)
    int _numSamples;
    
    FILE* file;
};

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinect_audio_listener"); //last argument could be argv[0]

    //init glut env for visualization
    glutInit(&argc, argv);

    ROS_INFO("Initializing kinect audio data recorder (based on libfreenect demo...\n");

    riddle::RosKinectAudioRecorder recorder;

    ROS_INFO("...init done!");

    ros::spin();

    return 0;
}
