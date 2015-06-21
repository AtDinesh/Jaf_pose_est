#ifndef HEADSHOULDEROBS_H
#define HEADSHOULDEROBS_H

#include <_observation.h>
#include <cmath>

template <class P>
class HeadShoulderObs : public _Observation<P>
{
public:
    HeadShoulderObs() : _Observation<P>()
    {
        this->factor = pow10(10.);
        this->variancePos = 10000.0;
        this->varianceAngle = 0.01;
    }

    virtual double likelihood(P &p)
    {
        assert((p.getStateVector()->getContParams().size() == 11) &&
               "This class is specialized for head shoulder pose estimation with state vector of 11 dimensions");

        //result = prod(100*exp(-(abs(particles(:,:)-observation(:)).^2)/option.ObsNoise), 1);
        const std::vector<double> & contState = p.getStateVector()->getContParams();


        double result = 1.0;

        //if no observation return a weight of 1
        if (this->contObserv.size() == 0)
            return result;

        // x
        double dx1, dx2, dy1, dy2, dz1, dz2;
        dx1 = contState[0] - this->contObserv[1]; // particles(1,:)-observation(2);
        dx2 = contState[0] - this->contObserv[4];
        dy1 = contState[1] - this->contObserv[2];
        dy2 = contState[1] - this->contObserv[5];
        dz1 = contState[2] - this->contObserv[3];
        dz2 = contState[2] - this->contObserv[6];
        /* x */
        result  = exp(-(dx1*dx1)/this->variancePos)*this->factor;
        result *= exp(-(dx2*dx2)/this->variancePos)*this->factor;
        /* y */
        result *= exp(-(dy1*dy1)/this->variancePos)*this->factor;
        result *= exp(-(dy2*dy2)/this->variancePos)*this->factor;
        /* z */
        result *= exp(-(dz1*dz1)/this->variancePos)*this->factor;
        result *= exp(-(dz2*dz2)/this->variancePos)*this->factor;

        double hp1, hp2, ht1, ht2, hr1, hr2, sp1, sp2;
        hp1 = contState[3] - cos(this->contObserv[7]*M_PI/180.0);
        hp2 = contState[4] - sin(this->contObserv[7]*M_PI/180.0);
        ht1 = contState[5] - cos(this->contObserv[8]*M_PI/180.0);
        ht2 = contState[6] - sin(this->contObserv[8]*M_PI/180.0);
        hr1 = contState[7] - cos(this->contObserv[9]*M_PI/180.0);
        hr2 = contState[8] - sin(this->contObserv[9]*M_PI/180.0);
        sp1 = contState[9] - cos(this->contObserv[10]*M_PI/180.0);
        sp2 = contState[10] - sin(this->contObserv[10]*M_PI/180.0);

        /* headPan */
        result *= exp(-(hp1*hp1)/this->varianceAngle)*this->factor;
        result *= exp(-(hp2*hp2)/this->varianceAngle)*this->factor;

        /* headTilt */
        result *= exp(-(ht1*ht1)/this->varianceAngle)*this->factor;
        result *= exp(-(ht2*ht2)/this->varianceAngle)*this->factor;

        /* headRoll */
        result *= exp(-(hr1*hr1)/this->varianceAngle)*this->factor;
        result *= exp(-(hr2*hr2)/this->varianceAngle)*this->factor;

        /* shoulderPan */
        result *= exp(-(sp1*sp1)/this->varianceAngle)*this->factor;
        result *= exp(-(sp2*sp2)/this->varianceAngle)*this->factor;

        return result;
    }

protected:
    double factor;
    double variancePos;
    double varianceAngle;

};

#endif // HEADSHOULDEROBS_H
