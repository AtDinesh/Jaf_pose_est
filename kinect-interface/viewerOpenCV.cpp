#include "viewerOpenCV.h"

ViewerOpenCV::ViewerOpenCV(std::string recordPath) : _recordPath(recordPath)
{
	cv::namedWindow("depthMap");
	cv::namedWindow("colorMap");

	_recorder = NULL;

	if (!_recordPath.empty())
		_recorder = new cv::VideoWriter();
}

ViewerOpenCV::~ViewerOpenCV()
{
	if (_recorder) delete _recorder;

	cv::destroyAllWindows();
	cv::waitKey(30);
}

void ViewerOpenCV::update(KinectStruct& kinectData, std::string filePath)
{
	if (!_recordPath.empty())
	{
		if (!_recorder->isOpened())
		{
			_recorder->open(_recordPath, CV_FOURCC('M', 'J', 'P', 'G'), 25, cv::Size(kinectData.RGBWidth, kinectData.RGBHeight));
			cv::waitKey(20);
		}
	}

	_depthMap = cv::Mat(kinectData.depthHeight, kinectData.depthWidth, CV_16UC1, kinectData.depthMetaData.WritableData());
	cv::imshow("depthMap", _depthMap);

	_rgbMap = cv::Mat(kinectData.RGBHeight, kinectData.RGBWidth, CV_8UC3, kinectData.imageMetaData.WritableData());
	cv::cvtColor(_rgbMap, _rgbMap, CV_BGR2RGB);
	cv::imshow("colorMap", _rgbMap);

	if (!filePath.empty())
	{
		std::ostringstream oss;
		oss << (int)kinectData.timestamp/1000 << ".png";
		cv::imwrite(filePath + "RGB" + oss.str(), _rgbMap);
		cv::imwrite(filePath + "Depth" + oss.str(), _depthMap);
	}

	if (!_recordPath.empty())
		*_recorder << (_rgbMap);
}
