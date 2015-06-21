#ifndef KINECTEXCEPTION_H
#define KINECTEXCEPTION_H

#include <exception>
#include <string>

class KinectException : public std::exception
{
	public:
		KinectException(std::string error, std::string openniString);
		~KinectException() throw();
		virtual const char * what() const throw();
		
	protected:
		std::string _msg;
};

#endif
