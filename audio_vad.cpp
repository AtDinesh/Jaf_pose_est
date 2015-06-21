#include <ros/ros.h>
#include <vector>
#include "sphinxbase/cont_ad.h"

#include "std_msgs/String.h"
#include "std_msgs/Int16MultiArray.h"
#include "riddle/VAD.h"


namespace Riddle
{
	class AudioVAD
	{
		public:
			AudioVAD(std::string msg_name):_input_message_name(msg_name)
			{
				this->initVAD();
				_sub = _nh.subscribe(_input_message_name, 100,
                             &AudioVAD::readAndPublishAudioBuffer, this);
                             
                _sub_reco = _nh.subscribe("allowReco", 100, &AudioVAD::isRecoAllowed, this);
                _isRecoAllowed = true;
                
                _pub = _nh.advertise<riddle::VAD>("vad_output", 1000);
                //_pub = _nh.advertise<std_msgs::String>("vad_output", 1000, true);
                //_pub_buff = _nh.advertise<std_msgs::Int16MultiArray>("vad_buffer16b", 100, true);
			}
			
			~AudioVAD()
			{
				if (_cont) cont_ad_close(_cont);
			}
			
			void isRecoAllowed(const std_msgs::String &msg)
			{
				if (!msg.data.compare("yes"))
				{
					ROS_INFO("VAD activated");
					_isRecoAllowed = true;
				}
				else if (!msg.data.compare("no"))
				{
					ROS_INFO("VAD deactivated");
					_isRecoAllowed = false;
				}
			}
			
			void initVAD()
			{
				_cont = NULL;
				_cont = cont_ad_init(NULL, NULL);
				ROS_INFO("calibrating silence : don't speak !!!");
				_calibrate = true;
				_uttStarted = false;

				_numSamples = 0;
			}
			
			void readAndPublishAudioBuffer(const std_msgs::Int16MultiArray &msg)
			{
				if (!_isRecoAllowed && !_calibrate) return;

				_buffSize = msg.data.size();
				_buffer = new int16_t[_buffSize];
				
				for (int i=0 ; i<_buffSize ; i++)
					_buffer[i] = msg.data[i];
				
				//ROS_INFO("message %d %d", msg.base_indx, msg.length);
				
				if (_calibrate)
				{
					int calib = cont_ad_calib_loop(_cont, _buffer, _buffSize);
					if (calib>0)
					{
						return;
					}
					else if (calib <0)
					{
						ROS_ERROR("calib failed !!!");
						return;
					}
					else
						ROS_INFO("delta_sil %d", _cont->delta_sil);
						ROS_INFO("delta_speech %d", _cont->delta_speech);
						ROS_INFO("calibrating finished");
						_calibrate = false;
				}
				
				int numFrame = cont_ad_read(_cont, _buffer, _buffSize);
				
				if (numFrame > 0 && !_uttStarted)
				{
					_uttStarted = true;
					_lastSpeechTs = _cont->read_ts;
					this->publish("start");
					ROS_INFO("utt started !");
					
					//this->updateWav();
				}
				else if (_uttStarted && ((_cont->read_ts - _lastSpeechTs) <= DEFAULT_SAMPLES_PER_SEC)*5)
				{
					this->publish("process");
					//this->updateWav();
					
				}
				else if (numFrame != 0 && _uttStarted && ((_cont->read_ts - _lastSpeechTs) > DEFAULT_SAMPLES_PER_SEC)*5)
				{
					this->publish("process");
					//this->updateWav();
					
				}
				else if (numFrame == 0 && _uttStarted && ((_cont->read_ts - _lastSpeechTs) > DEFAULT_SAMPLES_PER_SEC)*5)
				{
					_uttStarted = false;
					cont_ad_reset(_cont);
					this->publish("end");
					//this->updateWav();
					ROS_INFO("utt ended !");
				}

				delete[] _buffer;
			}
			
			void publish(std::string state)
			{
				_vad.buffers.resize(_buffSize);
				_vad.length = _buffSize;
				//std_msgs::Int16MultiArray buff;
				//buff.data.resize(_buffSize);
				for (int i=0 ; i < _buffSize ; i++)
				{
					_vad.buffers[i] = _buffer[i];
				}
				_vad.startTime = _lastSpeechTs;
				_vad.currTime = _cont->read_ts;
				_vad.state = state;
				_vad.header.stamp = ros::Time::now();
				
				//std_msgs::String str;
				//str.data = state;
				
				_pub.publish(_vad);
				//_pub.publish(str);
				//_pub_buff.publish(buff);
			}
			
		private:
			ros::NodeHandle _nh;
			ros::Subscriber _sub, _sub_reco;
			ros::Publisher _pub, _pub_buff;
			std::string _input_message_name;
			
			riddle::VAD _vad;
			
			cont_ad_t *_cont; //Speech detector
			int _buffSize; //buffer size (512 with freenect)
			int16_t *_buffer; //first buffer
			bool _uttStarted; //if speach is detected
			int _lastSpeechTs; //time of begining of utterance
			bool _calibrate; //is calibrating processing
			
			//for recording
			int _numSamples;
			
			bool _isRecoAllowed;
			
	};
};

int main (int argc, char **argv)
{
	ROS_INFO("Initializing audio VAD... (based on Sphinx)\n");
	
	ros::init(argc, argv, "audio_vad");
	
	Riddle::AudioVAD audioVAD("/audio_buffer_16b");
	
	ROS_INFO("audio VAD initialized\n");
	
	ros::spin();
	
	return 0;
}
