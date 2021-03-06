#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

#include <opencv2/opencv.hpp>
#include "observation.h"
#include "particle.h"
#include "gaussian.h"

/**
 * @brief In this class all the methods and objects necessary to achieve a particle filtering are implemented. It also
 * implements the ICondensation algorithm.
 * @author Elie MOUSSY
 * @date 2015
 */
class ParticleFilter
{
    public:
        /**
         * @brief Constructor.
         * @param frame : The first frame.
         * @param numberOfParticles : The number of particles in the particcle filter (50 by default).
         */
        ParticleFilter(const cv::Mat& frame, const unsigned int numberOfParticles=50);
        /**
          * @brief Destructor.
          */
        ~ParticleFilter() {}

        /**
         * @brief This method corresponds to a step of the algorithm. Calling step for each image achieves a RB-HS-SIR filtering.
         * @param frame : The image on which the step will be done.
         * @param detection : The vector of detected area.
         */
        void step(const cv::Mat& frame, const std::vector<Observation>& detections);
        /**
         * @brief This method initializes the particles using the detection area.
         * @param frame : The frame on which the detection was made.
         * @param region : The detection area.
         * @param nbrOfContParams : The number of continuous parameters. It should be 3 (by default) or 4.
         * If 3, the particles will have the x and y coordinates and the scale. If 4, The particles will
         * have the x and y coordinates, the width and the height.
         * @param initNoise : If true, the noise matrix will be initialized to a default value.
         */
        void initParticles(const cv::Mat &frame, const cv::Rect &region, const int nbrOfContParams=3, const bool initNoise=false);
        /**
         * @brief This method returns the bounding box of the track.
         * @return The bounding box of the track.
         */
        inline cv::Rect getBoundingBox() const
        {
            return trackParticle.getBoundingBox();
        }
        /**
         * @brief This method sets the dynamic noise matrix to noise.
         * @param noise : The new value of the dynamic noise matrix.
         */
        inline void setDynamicNoiseMatrix(const Eigen::MatrixXd& noise) { dynamicNoiseMatrix.noalias() = noise; }
        /**
         * @brief This method returns whether the track is activated or not.
         * @return Whether the track is active or not.
         */
        inline bool isActivated() const { return isActive; }
        /**
         * @brief This method activates/deactivates the track.
         * @param b : If true, the track will be activated. If false, it will be deactivated.
         */
        inline void activateTrack(bool b) { isActive = b; }
        /**
         * @brief This method returns the observation of the track.
         * @return The observation of the track.
         */
        inline Observation& getObservation() const { return (Observation&)obs; }
        /**
         * @brief This method returns the reactivation counter.
         * @return The reactivation counter.
         */
        inline int getReactivateCounter() const { return reactivate; }
        /**
         * @brief This method sets the reactivation counter to i.
         * @param i : The new value of the reactivation counter.
         */
        inline void setReactivateCounter(int i) { reactivate = i; }
        /**
         * @brief This method returns the ID of the track.
         * @return The ID of the track.
         */
        inline int getID() const { return ID; }
        /**
         * @brief This method sets the ID of the track to id.
         * @param id : The new ID of the track.
         */
        inline void setID(int id) { ID = id; }

    private:
        /**
         * @brief This method samples the continuous dynamic parameters of a particle.
         * @param idx : The index of the particle to sample.
         * @param method : The method of sampling (0 = condensation; 1 = detection+prior; 2 = hybrid function).
         * @param detection : The detection bounding box;
         * @return A coefficient that will help to compute the particle weight.
         */
        inline double sample(const unsigned int idx, const int method, const Observation &detection=Observation())
        {
            Particle& p = particlesList.at(idx);
            switch(method) {
                case 0: // Sampling with dynamics
                {
                    p.moveParticle(width, height);
                    return 1.;
                }
                    break;

                case 1: // Sampling with detection
                {
                    p.setPreviousContinuousParameters(p.getContinuousParameters());
                    Eigen::VectorXd mean(p.getNumberOfContParams());
                    const cv::Rect rec = detection.getBoundingBox();
                    mean(0) = (double)(rec.x);
                    mean(1) = (double)(rec.y);
                    if(trackParticle.getNumberOfContParams()==3)
                        mean(2) = (double)rec.width/64.;
                    else
                    {
                        mean(2) = (double)rec.width;
                        mean(3) = (double)rec.height;
                    }
                    Gaussian detecGauss(mean, dynamicNoiseMatrix);
                    int i=0;
                    while(!p.setContinuousParameters(detecGauss.addNoise(mean), width, height) && i <= 10)
                        i++;

                    // Detector evaluation
                    double gt_st = detecGauss.evaluate(p.getContinuousParameters());

                    // Dynamic evaluation
                    double ft_st = 0.;
                    const unsigned int particlesListSize = particlesList.size();
                    for (unsigned int i=0; i<particlesListSize; i++)
                    {
                        const Particle& part = particlesList.at(i);
                        Gaussian gaussEval1(part.getContinuousParameters(), dynamicNoiseMatrix);
                        ft_st += part.getPrevWeight()*gaussEval1.evaluate(p.getContinuousParameters());
                    }
                    if (gt_st)
                        return ft_st/gt_st;
                    return 1.;
                }
                    break;

                default: // Should never go through default
                    break;
            }

            return 1.;
        }
        /**
         * @brief This method computes a given particle weight using a color, motion and shape observation models.
         * @param frame : The frame on which the weight will be computed.
         * @param idx : The index of the particle in the particles list.
         */
        void computeParticleWeight(const cv::Mat &frame, const unsigned int idx, const double lambda);

        /**
         * @brief This method updates the track particle by computing the mean particle.
         */
        inline void updateTrackParticle()
        {
            trackParticle.setPreviousContinuousParameters(trackParticle.getContinuousParameters());

            double sumWeight = particlesList.at(0).getWeight();
            Eigen::VectorXd sum = particlesList.at(0).getContinuousParameters()*sumWeight;

            const unsigned int size = particlesList.size();
            for(unsigned int i=1; i<size; i++)
            {
                const Particle& p = particlesList.at(i);
                sum.noalias() += p.getContinuousParameters()*p.getWeight();
                sumWeight += p.getWeight();
            }

            if (sumWeight)
                sum /= sumWeight;
            trackParticle.setContinuousParameters(sum, width, height);
        }
        /**
         * @brief This method sorts the particles in the particles list by weight.
         */
        void resampleIndexes();
        /**
         * @brief The list of particles in the particle filter.
         */
        std::vector<Particle> particlesList;
        /**
         * @brief The proportion of sampling with dynamic. 1-q is the proportion of sampling with detection.
         */
        double q;
        /**
         * @brief The dynamic noise matrix.
         */
        Eigen::MatrixXd dynamicNoiseMatrix;
        /**
         * @brief The mean particle. This will contain the continuous parameters of the track.
         */
        Particle trackParticle;
        /**
         * @brief The width of the frame.
         */
        unsigned int width;
        /**
         * @brief The height of the frame.
         */
        unsigned int height;
        /**
         * @brief Flag that indicates whether the initialization of the particles has been done or not.
         */
        bool initDone;
        /**
         * @brief Flag that indicates whether the track is activated or not (only used for MOT).
         */
        bool isActive;
        /**
         * @brief The observation model of the particle filter.
         */
        Observation obs;
        /**
         * @brief Counter used to reactivate the track (for MOT).
         */
        int reactivate;
        /**
         * @brief The ID of the track.
         */
        int ID;
};

#endif // PARTICLEFILTER_H
