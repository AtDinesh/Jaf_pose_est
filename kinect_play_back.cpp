#include "kinect-interface/kinectInterface.h"
#include "kinect-interface/viewerOpenCV.h"

void displayMenu();

int main(int argc, char* argv[])
{

	KinectInterface kinect("", true, "/home/aamekonn/tmp/kinect/test.ONI");
	ViewerOpenCV viewer;

	KinectStruct data;
	KinectStruct* pData = &data;

	kinect.init();

	while(1)
	{
		kinect.updateAllMaps();
		kinect.getKinectData(pData);

		std::cout << "timestamp : " << pData->timestamp << std::endl;

		viewer.update(data, "/home/aamekonn/tmp/kinect/");

		if((cv::waitKey(25) != -1) || pData->isEOF) break;
	}

	return 0;
}

void displayMenu()
{
	using namespace std;

	cout << "Warning : no recording path specified" << endl;
	cout << "		   the stream is only played..." << endl;
	cout << "If you want to record type one of the following command" << endl;
	cout << "-ONI <record file path>" << endl;
	cout << "-AVI <record file path>" << endl;
}
