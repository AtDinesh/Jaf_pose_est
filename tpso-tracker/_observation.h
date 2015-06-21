#ifndef _OBSERVATION_H
#define _OBSERVATION_H

#include <vector>

template <class P>
class _Observation
{
public:
    _Observation()  { }

    virtual double likelihood(P & p) = 0;
    virtual void setObs(std::vector<int> discrObs,
                        std::vector<double> contObs)
    {
        this->discrObserv = discrObs;
        this->contObserv = contObs;
    }

    virtual void setNoise(std::vector<int> & dNoise,
                          std::vector<double> & cNoise)
    {
        this->contNoise = cNoise;
        this->discrNoise = dNoise;
    }

    virtual
    std::vector<int> & getMutDiscrObserv()       {   return this->discrObserv;   }
    virtual
    std::vector<double> & getMutContObserv()    {   return this->contObserv;    }

    virtual const
    std::vector<int> & getDiscrObserv()     const   {   return this->discrObserv;   }
    virtual const
    std::vector<double> & getContObserv()   const   {   return this->contObserv;    }

protected:
    std::vector<int> discrObserv;
    std::vector<double> contObserv;
    std::vector<int> discrNoise;
    std::vector<double> contNoise;
};

/**
 * Trial observation class
 */

/*
template <class P>
class Obs : public _Observation<P>
{
public:
    Obs();
    virtual double likelihood(P & p);

};

template <class P>  Obs<P>::Obs() {}
template <class P>  double Obs<P>::likelihood(P & p) {  return 0.;  }
*/

#endif // _OBSERVATION_H
