#include "kinectException.h"

KinectException::KinectException(std::string error, std::string openniString)
{
	_msg = error + " : " + openniString;
}

KinectException::~KinectException() throw()
{
}

const char* KinectException::what() const throw()
{
	return this->_msg.c_str();
}
