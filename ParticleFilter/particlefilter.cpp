#include "particlefilter.h"
#include <assert.h>
#include <pthread.h>

struct samplingData {
        Particle* p;
        int method;
        Observation obs;
        double ft_st;
        double width;
        double height;
        Eigen::MatrixXd dynNoise;
        cv::Mat frame;
        int nbPart;
};

inline double distance(const double x1, const double y1, const double x2, const double y2, const double sigx, const double sigy)
{
    const double tmp1 = (x1-x2)/sigx;
    const double tmp2 = (y1-y2)/sigy;
    return std::exp(-0.5*(tmp1*tmp1 + tmp2*tmp2));
}

void* sample_Thread(void* thread_arg)
{
    struct samplingData *sd = (struct samplingData*) thread_arg;
    Particle *p = sd->p;

    double lambda = 1.;
    switch(sd->method) {
        case 0: // Sampling with dynamics
        {
            p->moveParticle(sd->width, sd->height);
        }
            break;

        case 1: // Sampling with detection
        {
            p->setPreviousContinuousParameters(p->getContinuousParameters());
            Eigen::VectorXd mean(p->getNumberOfContParams());
            const cv::Rect rec = sd->obs.getBoundingBox();
            mean(0) = (double)(rec.x);
            mean(1) = (double)(rec.y);
            if(p->getNumberOfContParams()==3)
                mean(2) = (double)rec.width/64.;
            else
            {
                mean(2) = (double)rec.width;
                mean(3) = (double)rec.height;
            }
            Gaussian detecGauss(mean, sd->dynNoise);
            int i=0;
            while(!p->setContinuousParameters(detecGauss.addNoise(mean), sd->width, sd->height) && i <= 10)
                i++;

            // Detector evaluation
            const double gt_st = detecGauss.evaluate(p->getContinuousParameters());

            // Dynamic evaluation
            if (gt_st)
                lambda = sd->ft_st/gt_st;
        }
            break;

        default: // Should never go through default
            break;
    }
    // Compute normalized particle weight

    // Color distribution
    cv::MatND oldHist = p->getHistogram();
    p->computeHistogram(sd->frame, p->getBoundingBox());
    double newWeight = p->bhattacharyyaDistance(oldHist)*lambda;

    // Distance
    newWeight *= distance(p->getContinuousParameters()(0), p->getContinuousParameters()(1),
                          p->getPreviousContinuousParameters()(0), p->getPreviousContinuousParameters()(1),
                          50., 50.);

    p->setWeight(newWeight/(double)sd->nbPart);
    pthread_exit(NULL);
}

ParticleFilter::ParticleFilter(const cv::Mat &frame, const unsigned int numberOfParticles) :
    trackParticle(frame)
{
    // Initialization of frame properties.
    width = frame.cols;
    height = frame.rows;

    // Initialization of the sampling mode thresholds.
    q = 0.6;

    // Creation of the particles.
    for (unsigned int i=0; i<numberOfParticles; i++)
        particlesList.push_back(Particle(frame));

    initDone = false;
    isActive = false;
}

inline int compareParticles(const Particle& p1, const Particle& p2)
{
    if( p1.getWeight() > p2.getWeight() )
        return -1;
    if( p1.getWeight() < p2.getWeight() )
        return 1;
    return 0;
}

void ParticleFilter::resampleIndexes()
{
    const unsigned int size = particlesList.size();
    for (unsigned int i=0; i<size; i++)
        for (unsigned int j=i; j<size; j++)
            if (compareParticles(particlesList.at(i), particlesList.at(j))==1)
            {
                Particle p = particlesList.at(i);
                particlesList.at(i) = particlesList.at(j);
                particlesList.at(j) = p;
            }
}

inline int findTheBestDetection(const std::vector<Observation>& detections, const cv::Rect& ref)
{
    const unsigned int detectionsSize = detections.size();
    if (detectionsSize==0)
        return -1;
    int res = 0;
    double minDist=0.;

    for (unsigned int i=0; i<detectionsSize; i++)
    {
        const cv::Rect rect = detections.at(i).getBoundingBox();
        const double x = ((double)rect.x+(double)rect.width/2.) - ((double)ref.x+(double)ref.width/2.);
        const double y = ((double)rect.y+(double)rect.height/2.) - ((double)ref.y+(double)ref.height/2.);
        const double dist = std::sqrt(x*x+y*y);
        if (i==0)
            minDist = dist;
        else if (dist < minDist)
        {
            minDist = dist;
            res = i;
        }
    }
    if (minDist < 50.)
        return res;
    return  res;
}

void ParticleFilter::step(const cv::Mat &frame, const std::vector<Observation> &detections)
{
    assert(initDone);

    // Resample the indexes of the particles by their weight.
    resampleIndexes();

    const unsigned int particlesListSize = particlesList.size();
    for (unsigned int i=0; i<particlesListSize; i++)
        particlesList.at(i) = trackParticle;

    // Particles prediction without noise. We normalize the current weight, set it as the previous one and
    // reinitialize the current weight.
    for(unsigned int i=0; i<particlesListSize; i++)
    {
        Particle& p = particlesList.at(i);
        // Normalize the particles weight.
        p.setWeight(1./(double)particlesListSize);

        // Reinitialize the particles weight
        p.setWeight(1./(double)particlesListSize);
    }

    // Filtering.
    int idx = -1;
    if (detections.size()>0)
        idx = findTheBestDetection(detections, trackParticle.getBoundingBox());
    double ft_st = 0.;
    for (unsigned int i=0; i<particlesListSize&&idx!=-1; i++)
    {
        const Particle& p = particlesList.at(i);
        Gaussian gaussEval1(p.getContinuousParameters(), dynamicNoiseMatrix);
        ft_st += p.getPrevWeight()*gaussEval1.evaluate(particlesList.at(idx).getContinuousParameters());
    }
    for (unsigned int i=0; i<particlesListSize; i++)
    {
        const double alpha = RAND;
        //double lambda;
        if (idx!=-1) // Case where a detection exist
        {
            if(alpha < q) // Sample with dynamics
            {
                pthread_t thread;
                struct samplingData sd;
                sd.p = &particlesList.at(i);
                sd.method = 0;
                sd.width = width;
                sd.height = height;
                sd.nbPart = particlesListSize;
                sd.frame = frame;
                int rc = pthread_create(&thread, NULL, sample_Thread, (void*)&sd);
                if (rc)
                    std::cout << "error 1 : " << rc << std::endl;
                pthread_join(thread, NULL);
                //lambda = sample(i, 0);
            } else {    // Sample with detection
                pthread_t thread;
                struct samplingData sd;
                sd.p = &particlesList.at(i);
                sd.method = 1;
                sd.width = width;
                sd.height = height;
                sd.dynNoise = dynamicNoiseMatrix;
                sd.ft_st = ft_st;
                sd.obs = detections.at((unsigned int)idx);
                sd.nbPart = particlesListSize;
                sd.frame = frame;
                int rc = pthread_create(&thread, NULL, sample_Thread, (void *)&sd);
                if (rc)
                    std::cout << "error 2 : " << rc << std::endl;
                pthread_join(thread, NULL);
                //lambda = sample(i, 1, detections.at(idx));
            }
        } else {
            pthread_t thread;
            struct samplingData sd;
            sd.p = &particlesList.at(i);
            sd.method = 0;
            sd.width = width;
            sd.height = height;
            sd.nbPart = particlesListSize;
            sd.frame = frame;
            int rc = pthread_create(&thread, NULL, sample_Thread, (void *)&sd);
            if (rc)
                std::cout << "error 3 : " << rc << std::endl;
            pthread_join(thread, NULL);
            //lambda = sample(i, 0);
        }

        // Compute particle weight.
        //computeParticleWeight(frame, i, lambda);

        // Normalize the particle weight.
        //particlesList.at(i).setWeight((double)(particlesList.at(i).getWeight()/particlesList.size()));
    }

    // Selection of the mean particle.
    updateTrackParticle();

    // Update the observation
    obs.computeHistogram(frame, trackParticle.getBoundingBox());
}

void ParticleFilter::computeParticleWeight(const cv::Mat &frame, const unsigned int idx, const double lambda)
{
    Particle& p = particlesList.at(idx);
    // Color distribution
    cv::MatND oldHist = p.getHistogram();
    p.computeHistogram(frame, p.getBoundingBox());
    double newWeight = p.bhattacharyyaDistance(oldHist)*lambda;

    // Distance
    newWeight *= distance(p.getContinuousParameters()(0), p.getContinuousParameters()(1),
                          p.getPreviousContinuousParameters()(0), p.getPreviousContinuousParameters()(1),
                          50., 50.);

    p.setWeight(newWeight, false);
}

void ParticleFilter::initParticles(const cv::Mat& frame, const cv::Rect& region, const int nbrOfContParams, const bool initNoise)
{
    assert((unsigned int)(region.x+region.width) <= width && (unsigned int)(region.y+region.height) <= height && region.x > 0 && region.y > 0 &&
           (nbrOfContParams==3 || nbrOfContParams == 4));

    if (initNoise)
    {
        if(nbrOfContParams==3)
        {
            dynamicNoiseMatrix.resize(3,3);
            dynamicNoiseMatrix << 40., 0., 0.,
                                  0., 40., 0.,
                                  0., 0., 0.1;
        } else
        {
            dynamicNoiseMatrix.resize(4,4);
            dynamicNoiseMatrix << 10., 0., 0., 0.,
                                  0., 10., 0., 0.,
                                  0., 0., 10., 0.,
                                  0., 0., 0., 10.;
        }
    }

    const unsigned int size = particlesList.size();
    for (unsigned int i=0; i<size; i++)
    {
        Eigen::VectorXd contParams(nbrOfContParams);
        Eigen::MatrixXd paramsRange(nbrOfContParams, 2);
        contParams(0) = (double)(region.x);
        contParams(1) = (double)(region.y);
        if(nbrOfContParams==3)
        {
            contParams(2) = (double)region.width/64.;
            paramsRange << 0., (double)width,
                           0., (double)height,
                           0.1, 1.;
        } else {
            contParams(2) = (double)region.width;
            contParams(3) = (double)region.height;
            paramsRange << 0., (double)width,
                           0., (double)height,
                           30., 100.,
                           30., 200.;
        }
        Gaussian priorGauss(contParams, dynamicNoiseMatrix);
        Particle& p = particlesList.at(i);
        p.init(priorGauss.addNoise(contParams), paramsRange);
        p.setPreviousContinuousParameters(contParams);
        p.computeHistogram(frame, region);
    }

    trackParticle = particlesList.at(0);
    initDone = true;
    isActive = true;
    obs.computeHistogram(frame, trackParticle.getBoundingBox());
}
