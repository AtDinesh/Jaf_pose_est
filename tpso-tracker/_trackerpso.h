#ifndef _TRACKERPSO_H
#define _TRACKERPSO_H

#include <vector>
#include <boost/shared_ptr.hpp>

//#include <boost/random.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/uniform_real.hpp>


#include <_setting.h>
#include <_observation.h>
#include <_dynamics.h>

namespace tpso {

template <class P>
class _TrackerPSO
{
public:
    typedef boost::shared_ptr<P> PPtr;
    typedef boost::shared_ptr<_Setting> SettingPtr;
    typedef boost::shared_ptr< _Observation<P> > ObservPtr;
    typedef boost::shared_ptr< _Dynamics<P> > DynamicsPtr;

    _TrackerPSO()
    {
        this->ptrUniDist    =    boost::shared_ptr<  boost::uniform_real<>   >
                (new boost::uniform_real<>(0,1));

        this->ptrUniDistM1P1 =    boost::shared_ptr<  boost::uniform_real<>   >
                (new boost::uniform_real<>(-1,1));
    }

    virtual void init(unsigned int _nbrP,
                      SettingPtr & _setPtr,
                      ObservPtr & _obsPtr,
                      DynamicsPtr & _dynamics)
    {
        this->nbParticles = _nbrP;

        this->setting   =   _setPtr;
        this->observ    =   _obsPtr;
        this->dynmxPtr  =   _dynamics;

        unsigned int dimension = this->setting->getDimension();
        this->w = 0.;
        this->phig = 0.;
        this->phip = 0.;

        this->sx.clear();
        this->sp.clear();
        this->sv.clear();

        //initialize particles
        std::vector<int> minD, maxD, dProcNoise;
        std::vector<double> minCont(dimension, -1.0);
        std::vector<double> maxCont(dimension, 1.0);
        std::vector<double> cProcNoise(dimension, 0.05);
        std::vector<double> cProcNoise2(dimension, 0.05*0.05);

        for (unsigned int i = 0; i < this->nbParticles; i++)
        {
            this->sx.push_back(boost::shared_ptr<P>(new P(i)));
            this->sx[i]->randomSample(minD, maxD, dProcNoise, minCont, maxCont, cProcNoise, this->rng);
            this->sx[i]->adjustLimits(this->setting.get());

            this->sv.push_back(boost::shared_ptr<P>(new P(i)));
            this->sv[i]->randomSample(minD, maxD, dProcNoise, minCont, maxCont, cProcNoise2, this->rng);

            this->sp.push_back(boost::shared_ptr<P>(new P(*(this->sx[i]))));
        }

        //initialize properly
        this->sg = boost::shared_ptr<P>(new P());
        this->mmse = boost::shared_ptr<P>(new P());
        this->zeroStateP = boost::shared_ptr<P>(new P());

        std::cout<<"TPSO Tracker initialized!"<<std::endl;

        this->dynmxPtr->setRndGenerator(this->rng);

        this->first_step = true;
        this->initialized =  true;
    }

    virtual void disperse()
    {
        if (!this->is_init())
            return;
        //initialize particles
        unsigned int dimension = this->setting->getDimension();

        std::vector<int> minD, maxD, dProcNoise;
        std::vector<double> minCont(dimension, -1.0);
        std::vector<double> maxCont(dimension, 1.0);
        std::vector<double> cProcNoise(dimension, 0.05);
        std::vector<double> cProcNoise2(dimension, 0.05*0.05);

        for (unsigned int i = 0; i < this->nbParticles; i++)
        {
            //this->sx.push_back(boost::shared_ptr<P>(new P(i)));
            this->sx[i]->randomSample(minD, maxD, dProcNoise, minCont, maxCont, cProcNoise, this->rng);
            this->sx[i]->adjustLimits(this->setting.get());

            //this->sv.push_back(boost::shared_ptr<P>(new P(i)));
            this->sv[i]->randomSample(minD, maxD, dProcNoise, minCont, maxCont, cProcNoise2, this->rng);

            this->sp[i]->randomSample(minD, maxD, dProcNoise, minCont, maxCont, cProcNoise, this->rng);
            this->sp[i]->adjustLimits(this->setting.get());
        }

        this->first_step = true;
    }
    //this function should do the work here!
    //         (at a higher level)
    virtual void step();

    virtual
    P & getMapEstimate()    {   return *(this->sg);    }

    virtual
    P & getMmseEstimate()    {   return *(this->mmse);    }

    virtual void setSetting(SettingPtr & _setPtr);
    virtual void setObservationModel(ObservPtr & _obsPtr);
    virtual void setDynamicsModel(DynamicsPtr & _dynamics);

public: //get and setters
    double getParamW() const    {   return w;       }
    double getParamPhig() const {   return phig;    }
    double getParamPhip() const {   return phip;    }

    virtual void setParamW(double _w)       {   this->w = _w;       }
    virtual void setParamPhig(double _pg)   {   this->phig = _pg;   }
    virtual void setParamPhip(double _pp)   {   this->phip = _pp;   }


    virtual
    const std::vector<PPtr> & getParticles() const    {   return this->sp; }

    virtual
    std::vector<PPtr> & getMutParticles()   {   return this->sp; }

    virtual bool is_init()      {   return this->initialized;         }


    /*
     *  operators to be overloaded
     */


protected:
    bool initialized;
    bool first_step;

    std::vector<PPtr> sx;   //swarm x
    std::vector<PPtr> sp;   //swarm p
    std::vector<PPtr> sv;   //swarm v

    PPtr sg; //map estimate
    PPtr mmse; //mmse estimate
    PPtr zeroStateP;

    unsigned int nbParticles;

    SettingPtr  setting;
    ObservPtr   observ;
    DynamicsPtr dynmxPtr;

    //weighting terms
    double w, phig, phip;
    double rg, rp;

    //random generators
    boost::mt19937 rng;
    boost::shared_ptr< boost::uniform_real<> > ptrUniDist;//(0,1);
    boost::shared_ptr< boost::uniform_real<> > ptrUniDistM1P1;//(-,1);
};

}


using namespace tpso;

template<class P>
void _TrackerPSO<P>::step()
{
    assert(this->initialized &&
           "Please initialize PSO Tracker using init() routine!");
    /*
     *  Algorithm:
     *
     *  (1) sample rg, rp ~ U(0,1)
     *  (2) v_i(t)  =   w*v_i(t-1) + phip*rp* ( d(p_i(t-1) - x_i(t-1) )
     *                             + phig*rg* ( d(g(t-1)   - x_i(t-1) )
     *  (3) x_i(t)  =   x_i(t-1) + v_i(t)
     *  (4) p_i(t)  =   ( f(x_i(t)) > f(p_i(t-1)) ) ? x_i(t-1) : p_i(t-1)
     *
     *  (5) this->mapEstimate();
     *  (6) this->mmseEstimate();
     */
    *(this->mmse) = *(this->zeroStateP);

    if(this->first_step)
    {
        //initialize map and mmse estimates
        unsigned int max_indx = 0;
        double max_val = 0.;
        double lik_score;

        double normalizer = 0.;

        for (unsigned int i = 0; i < this->nbParticles; i++)
        {
            PPtr & sp = this->sp[i];
            lik_score = this->observ->likelihood((*sp));

            normalizer += lik_score;

            (*(this->mmse)) += (*(sp))*lik_score;

            if (lik_score > max_val)
            {
                max_val = lik_score;
                max_indx = i;
            }
        }

        (*this->sg)     =   *(this->sp[max_indx]);
        this->sg->setWeight(max_val);

        *(this->mmse)  /=   normalizer;
        this->mmse->adjustLimits(this->setting.get());

        this->first_step = false;

        return;
    }


    //apply dynamics to previous map estimate
    this->dynmxPtr->apply((*this->sg));
    this->sg->setWeight(this->observ->likelihood((*this->sg)));

    //loop through the particles
    int map_indx = -1;
    double max_val = this->sg->getWeight();

    double normalizer = 0.;

    for (unsigned int i = 0; i < nbParticles; i++ )
    {
        PPtr & sx = this->sx[i];
        PPtr & sp = this->sp[i];
        PPtr & sv = this->sv[i];

        this->rg = this->ptrUniDist->operator ()(rng);
        this->rp = this->ptrUniDist->operator ()(rng);

        this->dynmxPtr->apply((*sp)); //apply limit correction afterwards
        sp->adjustLimits(this->setting.get());

        *(sv)     =   ((*sv)*this->w) +
                ((*sp)     - (*sx))*this->phip*this->rp +
                ((*this->sg) - (*sx))*this->phig*this->rg  ;

        *(sx)     =   (*sx) + (*sv); //apply limit correction afterwads
        sx->adjustLimits(this->setting.get());

        sx->setWeight(this->observ->likelihood((*sx)));
        sp->setWeight(this->observ->likelihood((*sp)));

        if (sx->getWeight() > sp->getWeight() )
            (*sp) = (*sx); //assign the value

        double like = sp->getWeight();

        if (like > max_val)
        {
            map_indx = (int) i;
            max_val = like;
        }

        *(this->mmse) += (*(sp))*like;

        normalizer += like;
    }

    if( map_indx != -1)
        *(this->sg) = (*(this->sp[map_indx]));

    *(this->mmse) /= normalizer;

    this->mmse->adjustLimits(this->setting.get());
}

template<class P>
void _TrackerPSO<P>::setSetting(SettingPtr & _setPtr)
{
    this->setting = _setPtr;
}

template<class P>
void _TrackerPSO<P>::setObservationModel(ObservPtr & _obsPtr)
{
    this->observ = _obsPtr;
}

template<class P>
void _TrackerPSO<P>::setDynamicsModel(DynamicsPtr & _dynamics)
{
    this->dynmxPtr = _dynamics;
    this->dynmxPtr->setRndGenerator(rng);
}
#endif // _TRACKERPSO_H
