#include <ros/ros.h>
#include <vector>
#include "pocketsphinx/pocketsphinx.h"
#include "sphinxbase/cmd_ln.h"

#include "riddle/VAD.h"
#include "riddle/Utterance.h"

namespace Riddle
{
	static const arg_t cont_args_def[] = {
	POCKETSPHINX_OPTIONS,
	CMDLN_EMPTY_OPTION
};
	
	
	class AudioSTT
	{
		public:
			AudioSTT(std::string msg_name) : _input_message_name(msg_name)
			{
                ros::NodeHandle nh_t("~");
                nh_t.param("model_path", _model_files_path, std::string("../share/models/"));

				this->initReco();
				
				_sub = _nh.subscribe(_input_message_name, 100,
                             &AudioSTT::readAndPublish, this);
                             
                _pub = _nh.advertise<riddle::Utterance>("utterance", 10);



			}
			
			~AudioSTT()
			{
				if (_psDecoder) ps_free(_psDecoder);
			}
			
			void initReco()
			{
                /*
                char *argv[]={
                    (char*)"audioDecoder",
                    (char*)"-dict",
                    (char*)"../share/models/dict/frenchWords62K.dic",
                    (char*)"-fsgusefiller",
                    (char*)"no",
                    (char*)"-hmm",
                    (char*)"../share/models/acoustic/lium_french_f0/",
                    //(char*)"-lm",
                    //(char*)"/home/christophe/nao/riddle/models/language/french3g62K.lm.dmp",
                    (char*)"-jsgf",
                    (char*)"../share/models/DemoV0.jsgf"
                    //(char*)"-fsg",
                    //(char*)"/home/christophe/nao/riddle/DemoV0.fsg"
                };*/

				char ** argv;
				argv = new char*[9];
				argv[0] = (char*)"audioDecoder";
				argv[1] = (char*)"-dict";

                std::string tmp_buf = _model_files_path + "/dict/frenchWords62K.dic";
                argv[2] = new char[tmp_buf.length() + 1]; //+1 for null
                std::size_t length = tmp_buf.copy(argv[2], tmp_buf.length(), 0);
                argv[2][length]='\0';

				argv[3] = (char*)"-fsgusefiller";
				argv[4] = (char*)"no";
				argv[5] = (char*)"-hmm";

                tmp_buf = _model_files_path + "/acoustic/lium_french_f0/";
                argv[6] = new char[tmp_buf.length() + 1];
                length = tmp_buf.copy(argv[6], tmp_buf.length(), 0);
                argv[6][length]='\0';

				argv[7] = (char*)"-jsgf";

                tmp_buf = _model_files_path + "/DemoV0.jsgf";
                argv[8] = new char[tmp_buf.length() + 1];
                length = tmp_buf.copy(argv[8], tmp_buf.length(), 0);
                argv[8][length]='\0';
				

				int argc = 9;

				_cmd_ln = cmd_ln_parse_r(NULL, cont_args_def, argc, argv, FALSE);
				_psDecoder = NULL;
				_psDecoder = ps_init(_cmd_ln);
				
				_buffer = NULL;
			}
			
			void readAndPublish(const riddle::VAD &msg)
			{
				this->read(msg);
				
				if (!msg.state.compare("start"))
				{
					ps_start_utt(_psDecoder, NULL);
					//ps_process_raw(_psDecoder, _buffer, msg.length, 0, 0);
					ROS_INFO("start reco");
				}
				else if (!msg.state.compare("process"))
				{
					ps_process_raw(_psDecoder, _buffer, msg.length, 0, 0);
				}
				else if (!msg.state.compare("end"))
				{
					//ps_process_raw(_psDecoder, _buffer, msg.length, 0, 0);
					int res = ps_end_utt(_psDecoder);
					if (res < 0) ROS_WARN("Error reco");
					ROS_INFO("reco ended");
					
					ps_nbest_t * nbest=NULL;
					nbest = ps_nbest(_psDecoder, 0, -1, NULL, NULL);
					
					_utt.length = 0;
					_utt.utterances.clear();
					_utt.scores.clear();
					
					if (nbest != NULL)
					{
					for (nbest ; nbest != NULL ;)
					{
						const char *hyp;
						int score;
						ps_nbest_next(nbest);
						hyp = ps_nbest_hyp(nbest, &score);
						ROS_INFO("utt : %s , score : %d", hyp, score);
						if (hyp == NULL) break;
						_utt.utterances.push_back(hyp);
						_utt.scores.push_back(score);
						_utt.length++;
					}
					this->publish(msg.header.seq);
					}
					else
					{
						ROS_INFO("no hypothesis");
					}
					
				}
				
				delete[] _buffer;
			}
			
			void read(const riddle::VAD &msg)
			{
				_buffer = new int16_t[msg.length];
				for (int i=0 ; i<msg.length ; i++)
				{
					_buffer[i] = msg.buffers[i];
				}
			}
			
			void publish(int32_t id)
			{
				_utt.utt_id = id;
				_pub.publish(_utt);
			}
			
		private:
			ros::NodeHandle _nh;
			ros::Subscriber _sub;
			ros::Publisher _pub;
			std::string _input_message_name;
            std::string _model_files_path;
			
			ps_decoder_t *_psDecoder;
			cmd_ln_t* _cmd_ln;
			int16_t *_buffer;
			
			riddle::Utterance _utt;
	};
};

int main (int argc, char **argv)
{
	ros::init(argc, argv, "audio_stt");
	
	Riddle::AudioSTT audioSTT("vad");
	
	ros::spin();
	
	return 0;
}
