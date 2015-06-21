#ifndef GAUSSIANMIXTURE_H
#define GAUSSIANMIXTURE_H

#include <vector>
#include <assert.h>
#include "gaussian.h"

/**
 * @brief The GaussianMixture class is composed by a set of gaussians with coefficients in order to represent a mixture of gaussians.
 * @author Elie MOUSSY
 * @date 2015
 */
class GaussianMixture
{
    public:
        /**
         * @brief Constructor
         * @param nbrOfGaussians : The number of gaussians in the mixture.
         */
        GaussianMixture(const int nbrOfGaussians);
        /**
         * @brief Constructor
         * @param nbrOfGaussians : The number of gaussians in the mixture.
         * @param mean : The mean vector to initialize the gaussians.
         * @param covariance : The covariance vector to initialize the gaussians.
         */
        GaussianMixture(const int nbrOfGaussians, const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance);
        /**
          * @brief Destructor.
          */
        ~GaussianMixture() {}

        /**
         * @brief This method initializes all the gaussians using a mean vector and a covariance matrix.
         * @param mean : The mean vector to initialize the gaussians.
         * @param covariance : The covariance matrix to initialize the gaussians.
         */
        void init(const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance);
        /**
         * @brief This method initializes the first nb gaussians using a mean vector and a covariance matrix.
         * @param nb : The number of gaussians to initialize. It should be superior to 0 otherwise no gaussian will be initialized.
         * @param mean : The mean vector to initialize the gaussians.
         * @param covariance : The covariance matrix to initialize the gaussians.
         */
        void init(unsigned int nb, const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance);
        /**
         * @brief This method initializes all the gaussians using a given one.
         * @param other : The given Gaussian object that will help for the initialization.
         */
        void init(const Gaussian& other);
        /**
         * @brief This method initializes the first nb gaussians using a given one.
         * @param nb : The number of gaussians to initialize. It should be superior to 0 otherwise no gaussian will be initialized.
         * @param other : The given Gaussian object that will help for the initialization.
         */
        void init(unsigned int nb, const Gaussian& other);
        /**
         * @brief This method creates a Gaussian object from a given mean vector and a covariance matrix and adds it to the mixture.
         * @param mean : The mean vector of the new Gaussian object.
         * @param covariance : The covariance matrix of the new Gaussian object.
         */
        void add(const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance);
        /**
         * @brief This method creates a Gaussian object from a given one and adds it to the mixture.
         * @param other : The given Gaussian object that will help for the initialization.
         */
        void add(const Gaussian& other);
        /**
         * @brief This method evaluates a given vector.
         * @param vect : The vector to evaluate.
         * @return The vector evaluation.
         */
        double evaluate(const Eigen::VectorXd& vect);
        /**
         * @brief This method returns whether all the gaussians are equiprobable or not.
         * @return True if the gaussians are equiprobable, false otherwise.
         */
        bool gaussiansAreEquiprobable() const { return areEquiprob; }
        /**
         * @brief This method normalizes the weights of the gaussians by dividing them by weightSum
         * @param weightSum : The sum of all the weights on the exit of the method.
         */
        void updateMixtureWeight(const double weightSum);
        /**
         * @brief This method returns the gaussians list.
         * @return The gaussians list.
         */
        inline std::vector<Gaussian>& getGaussiansList() { return gList; }
        /**
         * @brief This method returns the gaussian at the index idx.
         * @param idx : The index of the gaussian to be returned.
         * @return The gaussian at the index idx.
         */
        inline Gaussian& getGaussian(const unsigned int idx)
        {
            assert( idx < gList.size() );
            return gList.at(idx);
        }

    private:
        /**
         * @brief Probability evaluation.
         * @param vect : The vector to evaluate.
         * @return Probability evaluation.
         */
        inline double evalEquiProb(const Eigen::VectorXd& vect);
        /**
         * @brief Probability evaluation.
         * @param vect : The vector to evaluate.
         * @return Probability evaluation.
         */
        inline double evalWeighted(const Eigen::VectorXd& vect);
        /**
         * @brief A vector of gaussians.
         */
        std::vector< Gaussian > gList;
        /**
         * @brief A vector of weights that correspond to the weights of the gaussians.
         */
        std::vector<double> weightList;
        /**
         * @brief This flag indicates that all the gaussians are equiprobable.
         */
        bool areEquiprob;
};

#endif // GAUSSIANMIXTURE_H
