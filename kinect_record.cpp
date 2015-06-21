#include "kinect-interface/kinectInterface.h"
#include "kinect-interface/viewerOpenCV.h"

void displayMenu();

int main(int argc, char* argv[])
{
	if (argc == 1)
		displayMenu();

	KinectInterface kinect("/home/aamekonn/tmp/kinect/test.ONI", true);
	ViewerOpenCV viewer;

	KinectStruct data;
	KinectStruct* pData = &data;

	kinect.init();

	while(1)
	{
		kinect.updateAllMaps();
		kinect.getKinectData(pData);

		viewer.update(data);

		if(cv::waitKey(25) != -1) break;
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
