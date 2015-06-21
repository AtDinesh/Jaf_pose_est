#ifndef _DYNAMICS_H
#define _DYNAMICS_H

//#include <boost/random.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>

template <class P>
class _Dynamics
{
public:
    _Dynamics() {   }

    virtual P & apply(P & p) = 0;

    virtual void setRndGenerator(boost::mt19937 & rGen)
    {
        this->rng = rGen;
        this->initialized = true;
    }

    virtual bool is_init()  {   return this->initialized;   }

protected:
    bool initialized;
    boost::mt19937 rng;
};

#endif // _DYNAMICS_H
