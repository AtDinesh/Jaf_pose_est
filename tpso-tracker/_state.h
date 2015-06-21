#ifndef _STATE_H
#define _STATE_H

#include <stdio.h>
#include <vector>
#include <math.h>

//#include <boost/random.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/shared_ptr.hpp>

namespace tpso {

template <int nD, int nC>
class _State
{
public:
    _State()
    {
        this->nbDiscParams_ = nD;
        this->nbContParams_ = nC;
        this->discreteParams_.resize(nD, 0);
        this->continousParams_.resize(nC, 0.);
    }

    _State(const _State<nD, nC> & state)
    {
        this->nbDiscParams_ = nD;
        this->nbContParams_ = nC;
        this->discreteParams_  = state.discreteParams_;
        this->continousParams_ = state.continousParams_;
    }

    _State(std::vector<int> &discrParams,
           std::vector<double> &contParams)
    {
        this->nbContParams_ = (int)contParams.size();
        this->nbDiscParams_ = (int)discrParams.size();
        this->discreteParams_     =   discrParams;
        this->continousParams_    =   contParams;
    }

    _State(_State<nD,nC> &in)
    {
        this->nbContParams_     = in.nbContParams_;
        this->nbDiscParams_     = in.nbDiscParams_;
        this->continousParams_  = in.continousParams_;
        this->discreteParams_   = in.discreteParams_;
    }

    virtual void set(_State<nD,nC> &in)
    {
        this->nbContParams_     = in.nbContParams_;
        this->nbDiscParams_     = in.nbDiscParams_;
        this->continousParams_  = in.continousParams_;
        this->discreteParams_   = in.discreteParams_;
    }


    const std::vector<int>    &   getDiscrParams()  const       {       return discreteParams_;         }
    const std::vector<double> &   getContParams()   const       {       return continousParams_;        }

    std::vector<int>    &   getMutDiscrParams()      {       return discreteParams_;         }
    std::vector<double> &   getMutContParams()       {       return continousParams_;        }

    void setDiscrParams(const std::vector<int> & discrParams)   {       discreteParams_ = discrParams;  }
    void setContParams (const std::vector<double> & contParams) {       continousParams_= contParams ;  }
    int getNbDiscrParams()  const                               {       return nbDiscParams_;           }
    int getNbContParams()   const                               {       return nbContParams_;           }


    inline virtual double perturb(const std::vector<double> & var,
                                  boost::mt19937 & rng);
    inline virtual
    void randomSample(const std::vector<int> & minDiscr,
                      const std::vector<int> & maxDiscr,
                      const std::vector<int> & disProcessNoise,
                      const std::vector<double> & minCont,
                      const std::vector<double> & maxCont,
                      const std::vector<double> & contProcessNoise,
                      boost::mt19937 & rng);

    inline virtual
    void limits(const std::vector<double> & min,
                const std::vector<double> & max)
    {
        assert(     (this->continousParams_.size()==min.size()) &&
                    (this->continousParams_.size()==max.size())       );

        // A simple rule that clips data exceeding limits in each direction
        for (size_t i = 0; i < this->continousParams_.size(); i++)
        {
            //check max
            if (this->continousParams_[i] > max[i])   this->continousParams_[i] = max[i];

            //check min
            if (this->continousParams_[i] < min[i])   this->continousParams_[i] = min[i];
        }
    }

    virtual void printInfo() const
    {
        printf("(d) [");
        for(int i = 0; i < this->nbDiscParams_; i++)
            printf(" %03d ", this->discreteParams_[i]);
        printf("] (c) [ ");
        for (int i=0; i < this->nbContParams_; i++)
            printf("%4.3lf ", this->continousParams_[i]);
        printf("],");
    }

    virtual bool operator==(const _State<nD, nC>& s) const
    {
        //check and return if the discrete values of the two states are equivalent
        if(this->nbDiscParams_ != s.getNbDiscrParams())
            return false;

        const std::vector<int>& dParams = s.getDiscrParams();
        for(int i = 0; i < this->nbDiscParams_; i++)
        {
            if(this->discreteParams_[i] != dParams[i])
                return false;
        }

        return true;
    }

    virtual
    _State<nD, nC>& operator+=(const _State<nD, nC>& s)
    {
        for(int i = 0; i < this->nbContParams_; i++)
            this->continousParams_[i] += s.continousParams_[i];
        return *this;
    }

    virtual
    _State<nD, nC> operator*(double w)
    {
        _State<nD, nC> tmp;
        tmp.nbContParams_ = this->nbContParams_;
        tmp.nbDiscParams_ = this->nbDiscParams_;
        tmp.discreteParams_ = this->discreteParams_;

        for(int i = 0; i < this->nbContParams_; i++)
            tmp.continousParams_[i] = this->continousParams_[i]*w;
        return tmp;
    }

    virtual
    _State<nD, nC> operator/(double w)
    {
        assert (w != 0.);
        _State<nD, nC> tmp;
        tmp.nbContParams_ = this->nbContParams_;
        tmp.nbDiscParams_ = this->nbDiscParams_;
        tmp.discreteParams_ = this->discreteParams_;

        for(int i = 0; i < this->nbContParams_; i++)
            tmp.continousParams_[i] = this->continousParams_[i]/w;
        return tmp;
    }

    virtual
    _State<nD, nC>& operator*=(double w)
    {
        for(int i = 0; i < this->nbContParams_; i++)
            this->continousParams_[i] *= w;
        return *this;
    }
    virtual
    _State<nD, nC>& operator/=(double w)
    {
        assert (w != 0.);
        for(int i = 0; i < this->nbContParams_; i++)
            this->continousParams_[i] /= w;
        return *this;
    }

    virtual
    _State<nD, nC> operator+(const _State<nD, nC>& s1)
    {
        _State<nD, nC> tmp;

        tmp.nbContParams_ = this->nbContParams_;
        tmp.nbDiscParams_ = this->nbDiscParams_;

        tmp.discreteParams_ = this->discreteParams_;
        for(int i = 0; i < this->nbContParams_; i++)
            tmp.continousParams_[i] =
                    this->continousParams_[i] + s1.continousParams_[i];
        return tmp;
    }

    virtual
    _State<nD, nC> operator-(const _State<nD, nC>& s1)
    {
        _State<nD, nC> tmp;

        tmp.nbContParams_ = this->nbContParams_;
        tmp.nbDiscParams_ = this->nbDiscParams_;

        tmp.discreteParams_ = this->discreteParams_;
        for(int i = 0; i < this->nbContParams_; i++)
            tmp.continousParams_[i] =
                    this->continousParams_[i] - s1.continousParams_[i];
        return tmp;
    }

    virtual
    _State<nD, nC>& operator-=(const _State<nD, nC>& s)
    {
        for(int i = 0; i < this->nbContParams_; i++)
            this->continousParams_[i] -= s.continousParams_[i];
        return *this;
    }

    virtual
    _State<nD, nC>& operator=(const _State<nD, nC>& s)
    {

        this->nbContParams_ = s.nbContParams_;
        this->nbDiscParams_ = s.nbDiscParams_;
        this->continousParams_  =   s.continousParams_;
        this->discreteParams_   =   s.discreteParams_;
        return *this;
    }

protected:
    int nbContParams_;
    int nbDiscParams_;
    std::vector<double> continousParams_;
    std::vector<int>    discreteParams_;
};

// a state vector with 1 discrete value and 3 continous vals
typedef _State<0,1> StateC;
typedef _State<0,2> State2C;
typedef _State<0,3> State3C;

typedef _State<1,1> StateDC;
typedef _State<1,2> State1D2C;
typedef _State<1,3> State1D3C;

typedef _State<0,6> State6C;
}

using namespace tpso;

template <int nD, int nC>
inline double _State<nD, nC>::perturb(const std::vector<double> &var,
                                      boost::mt19937 & rng)
{
    std::vector<double> prevState = continousParams_;

    for(size_t i = 0; i < continousParams_.size(); i++)
    {
        boost::normal_distribution<double> gauss(0, var[i]);
        continousParams_[i] = continousParams_[i] + gauss(rng);
    }

    //compute the likelihood of the new sample (according to dynamics)

    double pi = (double)M_PI;
    double exponent = continousParams_.size()/2.0;

    double factor = pow(2*pi, - exponent);

    double factorCov=1, factorExp=0;
    for (size_t i=0 ; i < continousParams_.size() ; i++)
    {
        factorCov *= var[i];
        factorExp += (prevState[i]-continousParams_[i])*(prevState[i]-continousParams_[i])/var[i];
    }
    factorExp = exp(-factorExp/2.);
    factorCov = 1./sqrt(factorCov);
    factor= (factor*factorCov*factorExp); // == (1/sqrt((2pi)^k|sigma|))

    return factor;
}

template <int nD, int nC>
inline void _State<nD, nC>::randomSample(const std::vector<int> & minDiscr,
                                         const std::vector<int> & maxDiscr,
                                         const std::vector<int> & disProcessNoise,
                                         const std::vector<double> & minCont,
                                         const std::vector<double> & maxCont,
                                         const std::vector<double> & contProcessNoise,
                                         boost::mt19937 & rng)
{
    for (size_t i = 0; i < this->discreteParams_.size(); i++)
    {
        boost::uniform_int<int> sampler(minDiscr[i], maxDiscr[i]);
        this->discreteParams_[i] = sampler(rng)*disProcessNoise[i];
    }

    for(size_t i = 0; i < this->continousParams_.size(); i++)
    {
        //declare sampler
        boost::uniform_real<double> sampler(minCont[i], maxCont[i]);
        this->continousParams_[i] = sampler(rng)*contProcessNoise[i];
    }

}


#endif // _STATE_H
