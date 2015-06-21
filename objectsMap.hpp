#ifndef OBJECTSMAP_HPP
#define OBJECTSMAP_HPP

#include <fstream>
#include <string>
#include <vector>
#include <iostream>
#include <math.h>

class ObjectsMap
{
	typedef struct ObjStruct
	{
		std::string objectName;
		std::string locationName;
		std::vector<double> pos;
		bool useObject;
	}ObjStruct;

	public:
		ObjectsMap(std::string filePath);
		~ObjectsMap();

		bool isInMap(std::string objectName);
		std::vector<double> getObjectPosition(std::string objectName);
		std::string getObjectLocationName(std::string objectName);
		double getObjetctAngle(std::string objectName);

	protected:
		void readObjectsFile(std::string& filePath);

	protected:
		std::string _filePath;
		std::vector<ObjStruct> _objList;

};

#endif
