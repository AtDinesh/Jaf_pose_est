#ifndef WAVEFORMOBS_H
#define WAVEFORMOBS_H

#include <_observation.h>

template <class P>
class WaveformObs : public _Observation<P>
{
public:

    WaveformObs() : _Observation<P>()   { }

    virtual double likelihood(P &p)
    {
        //result = prod(100*exp(-(abs(particles(:,:)-observation(:)).^2)/option.ObsNoise), 1);
        const std::vector<double> & contState = p.getStateVector()->getContParams();

        double l = 1.0;
        for (size_t i = 0; i <  contState.size(); i++)
        {
            double dx = abs(contState[i] - this->contObserv[i]);

            l *= ( 100.0*exp(-(dx*dx)/this->contNoise[i]) );
        }

        return l;
    }

};

#endif // WAVEFORMOBS_H
