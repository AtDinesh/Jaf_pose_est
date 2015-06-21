#ifndef RANDOMWALK_H
#define RANDOMWALK_H

#include <_dynamics.h>

template <class P>
class RandomWalk : public _Dynamics<P>
{
public:
    RandomWalk():_Dynamics<P>() {   }
    RandomWalk(double noise, unsigned int dim)
        :_Dynamics<P>()
    {
        this->processNoise = noise;
        this->dimension = dim;
        this->processNoiseDiag.resize(dimension, processNoise);
    }

    RandomWalk(std::vector<double> & noiseDiag):_Dynamics<P>()
    {
        this->processNoise = noiseDiag[0];
        this->processNoiseDiag = noiseDiag;
        this->dimension = (unsigned int)noiseDiag.size();
    }

    virtual P & apply(P & p)
    {
        assert(this->initialized &&
               "Please consider initializing _Dynamics via  setRndGenerator(...)");

        p.getMutStateVector()->perturb(processNoiseDiag, this->rng);
        return p;
    }

protected:
    unsigned int dimension;
    double processNoise;
    std::vector<double> processNoiseDiag;
};


#endif // RANDOMWALK_H
