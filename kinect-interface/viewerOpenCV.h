#ifndef VIEWEROPENCV_H
#define VIEWEROPENCV_H

#include <opencv2/opencv.hpp>
#include "kinectInterface.h"

class ViewerOpenCV
{
	public:
		ViewerOpenCV(std::string recordPath="");
		~ViewerOpenCV();
		void update(KinectStruct& kinectData, std::string filePath="");


	protected:
		std::string _recordPath;
		cv::Mat _rgbMap;
		cv::Mat _depthMap;
		cv::VideoWriter *_recorder;
};

#endif
