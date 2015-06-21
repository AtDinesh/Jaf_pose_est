#include "gaussianmixture.h"

GaussianMixture::GaussianMixture(const int nbrOfGaussians)
{
    for (int i=0; i<nbrOfGaussians; i++)
    {
        Gaussian newG;
        gList.push_back(newG);
        weightList.push_back( 1./(double)nbrOfGaussians );
    }
    areEquiprob = true;
}

GaussianMixture::GaussianMixture(const int nbrOfGaussians, const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance)
{
    for (int i=0; i<nbrOfGaussians; i++)
    {
        Gaussian newG(mean, covariance);
        gList.push_back(newG);
        weightList.push_back( 1./(double)nbrOfGaussians );
    }
    areEquiprob = true;
}

void GaussianMixture::init(const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance)
{
    const unsigned int size = gList.size();
    for(unsigned int i=0; i<size; i++)
        gList.at(i).init(mean, covariance);
}

void GaussianMixture::init(unsigned int nb, const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance)
{
    const unsigned int size = gList.size();
    for(unsigned int i=0; i<nb && i<size; i++)
        gList.at(i).init(mean, covariance);
}

void GaussianMixture::init(const Gaussian &other)
{
    const unsigned int size = gList.size();
    for(unsigned int i=0; i<size; i++)
        gList.at(i).init(other);
}

void GaussianMixture::init(unsigned int nb, const Gaussian& other)
{
    const unsigned int size = gList.size();
    for(unsigned int i=0; i<nb && i<size; i++)
        gList.at(i).init(other);
}

void GaussianMixture::add(const Eigen::VectorXd &mean, const Eigen::MatrixXd &covariance)
{
    Gaussian newG(mean, covariance);
    gList.push_back(newG);
    if(areEquiprob)
    {
        weightList.clear();
        const unsigned int size = gList.size();
        for(unsigned int i=0; i<size; i++)
            weightList.push_back(1./(double)size);
    }
}

void GaussianMixture::add(const Gaussian& other)
{
    Gaussian newG(other);
    gList.push_back(newG);
    if(areEquiprob)
    {
        weightList.clear();
        const unsigned int size = gList.size();
        for(unsigned int i=0; i<size; i++)
            weightList.push_back(1./(double)size);
    }
}

double GaussianMixture::evaluate(const Eigen::VectorXd& vect)
{
    if (areEquiprob)
        return evalEquiProb(vect);
    return evalWeighted(vect);
}

double GaussianMixture::evalEquiProb(const Eigen::VectorXd &vect)
{
    double sum=0.;

    // Compute probability for each gaussian.
    const unsigned int size = gList.size();
    for (unsigned int i=0; i<size; i++)
        sum += gList.at(i).evaluate(vect);

    return  1./( (double)size*sum );
}

double GaussianMixture::evalWeighted(const Eigen::VectorXd &vect)
{
    double prob=0.;

    // Compute probability for each guassian.
    const unsigned int size = gList.size();
    for(unsigned int i=0; i<size; i++)
        prob+=weightList.at(i)*gList.at(i).evaluate(vect);

    return prob;
}

void GaussianMixture::updateMixtureWeight(const double weightSum)
{
    if (weightSum<=0.)
        return;

    const unsigned int size = weightList.size();
    for (unsigned int i=0; i<size; i++)
        weightList.at(i) /= weightSum;

    areEquiprob=false;
}
