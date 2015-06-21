#ifndef _PARTICLE_H
#define _PARTICLE_H

#include <stdio.h>
#include <vector>

//#include <boost/random.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/smart_ptr.hpp>

//T corresponds to state vector
#include <_setting.h>

class _Setting;

namespace tpso {
template <class S>
class _Particle
{
    typedef boost::shared_ptr<S> SPtr;

public:
    _Particle(unsigned int n=0):particleNo_(n), weight_(0.)
    {
        this->stateVector_    =   SPtr(new S());
    }

    //copy constructor
    _Particle(const _Particle<S> &p)
    {
        //create a particle identical to this one
        this->particleNo_ = p.particleNo_;
        this->weight_ = p.weight_;

        this->stateVector_    =   SPtr(new S(*p.stateVector_));
        //prevStateVec_   =   SPtr(new S(*p.getPrevStateVector()));

    }

    const SPtr & getStateVector() const {   return this->stateVector_;  }
    SPtr & getMutStateVector()          {   return this->stateVector_;  }

    const SPtr & getPrevStateVector() const {   return this->prevStateVec_;  }

    void setStateVector(S & vec)        {   stateVector_    =   SPtr(new S(vec));  }
    void updateStateVector(S & vec)     {   assert(stateVector_);
                                            prevStateVec_   =   stateVector_;
                                                                                stateVector_    =   SPtr(new S(vec));   }

    long getNo() const                  {   return this->particleNo_;         }
    void setNo(unsigned int n)          {   this->particleNo_ = n;            }
    double getWeight() const            {   return this->weight_;             }
    void setWeight(double w)            {   this->weight_ = w;                }

    inline virtual
    void adjustLimits(_Setting * setting)
    {   this->stateVector_->limits(setting->getMinBound(),
                                   setting->getMaxBound());     }

    inline virtual
    void randomSample(const std::vector<int> & minDiscr,
                      const std::vector<int> & maxDiscr,
                      const std::vector<int> & disProcessNoise,
                      const std::vector<double> & minCont,
                      const std::vector<double> & maxCont,
                      const std::vector<double> & contProcessNoise,
                      boost::mt19937 & rng)
    {
        this->stateVector_->randomSample(minDiscr, maxDiscr, disProcessNoise,
                                         minCont, maxCont, contProcessNoise, rng);
    }

    virtual
    _Particle<S> & operator/=(const double ws)  {   (*stateVector_) /= ws;  return *this;   }
    virtual
    _Particle<S> & operator/(const double ws)   {   (*stateVector_) = (*stateVector_)/ws;   return *this;   }
    virtual
    _Particle<S> & operator*=(const double ws)  {   (*stateVector_) *= ws;  return *this;   }

    virtual
    _Particle<S> & operator+=(const _Particle<S> & p1)
    {
        *(this->stateVector_) += *(p1.getStateVector());

        return *this;
    }

    //virtual
    //_Particle<S> & operator*(const double ws)   {   (*stateVector_) = (*stateVector_)*ws;   return *this;   }

    virtual
    _Particle<S> operator*(const double ws)
    {
        _Particle<S> tmpP(*this);

        *(tmpP.stateVector_)  = *(this->stateVector_)*ws;

        return tmpP;
    }


    virtual
    _Particle<S> operator+(const _Particle<S> & p1)
    {
        _Particle<S> tmpP(*this);

        *(tmpP.stateVector_)  = *(this->stateVector_)   +   *(p1.stateVector_);
        //*(tmpP.prevStateVec_) = *(this->prevStateVec_)  +   *(p1.prevStateVec_);

        return tmpP;
    }

    virtual
    _Particle<S> operator-(const _Particle<S> & p1)
    {
        _Particle<S> tmpP(*this);

        *(tmpP.stateVector_)  = *(this->stateVector_)   -   *(p1.stateVector_);
        //*(tmpP.prevStateVec_) = *(this->prevStateVec_)  -   *(p1.prevStateVec_);

        return tmpP;
    }

    virtual //assignment operator
    _Particle<S> & operator=(const _Particle<S> & p1)
    {
        this->particleNo_ = p1.particleNo_;
        this->weight_ = p1.weight_;

        *(this->stateVector_) = *(p1.stateVector_);

        return *this;
    }

    virtual
    void printInfo()
    {
        printf(" No. [%05d] weight [%.4lf] ", particleNo_, weight_);
        stateVector_->printInfo();
        printf("\n");   fflush(stdout);
    }

protected:
    SPtr stateVector_;
    SPtr prevStateVec_;

    unsigned int particleNo_;
    double weight_;
};

}
#endif // _PARTICLE_H
