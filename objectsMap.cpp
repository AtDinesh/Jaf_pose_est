#include "objectsMap.hpp"

ObjectsMap::ObjectsMap(std::string filePath)
{
    this->_objList.clear();
	this->_filePath = filePath;
	this->readObjectsFile(filePath);
}

ObjectsMap::~ObjectsMap()
{
}

bool ObjectsMap::isInMap(std::string objectName)
{
	for (int i=0 ; i<_objList.size() ; i++)
	{
		if (!_objList[i].objectName.compare(objectName) && _objList[i].useObject)
			return true;
	}
	return false;
}

std::vector<double> ObjectsMap::getObjectPosition(std::string objectName)
{

	for (int i=0 ; i<_objList.size() ; i++)
	{
		if (!_objList[i].objectName.compare(objectName) && _objList[i].useObject)
			return _objList[i].pos;
	}
	return std::vector<double>(3,0);
}

std::string ObjectsMap::getObjectLocationName(std::string objectName)
{
	for (int i=0 ; i<_objList.size() ; i++)
	{
		if (!_objList[i].objectName.compare(objectName) && _objList[i].useObject)
			return _objList[i].locationName;
	}
}

double ObjectsMap::getObjetctAngle(std::string objectName)
{
	std::vector<double> pos = this->getObjectPosition(objectName);
	double angle=0;

	angle = atan(pos[1]/pos[0])*180./3.14;
	if ((pos[0]<0) && (angle > 0)) angle -= 180;
	else if ((pos[0]<0) && (angle <= 0)) angle += 180;

	return angle;
}

void ObjectsMap::readObjectsFile(std::string& filePath)
{
    std::ifstream file(filePath.c_str(), std::ios::in);

	if (!file.good())
		throw "Unable to retrieve Objects map file";

	while (file.good())
	{
		ObjStruct obj;
		obj.pos.resize(3,0);

		file >> obj.objectName >> obj.pos[0] >> obj.pos[1] >> obj.pos[2] >> obj.locationName >> obj.useObject;
		if (!file.good()) break;
		_objList.push_back(obj);
	}

	file.close();
}
