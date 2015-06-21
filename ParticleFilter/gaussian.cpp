#include "gaussian.h"
#include <assert.h>

Gaussian::Gaussian()
{
    srand(time(NULL));
}

Gaussian::Gaussian(const Eigen::VectorXd& mean_vector, const Eigen::MatrixXd& covariance_matrix)
{
    assert(mean_vector.size()==covariance_matrix.rows() && mean_vector.size()==covariance_matrix.cols());

    dim = mean_vector.size();
    mean.noalias() = mean_vector;
    cov.noalias() = covariance_matrix;
    Eigen::LLT<Eigen::MatrixXd> lltOfcov(cov);
    cholesky_cov = lltOfcov.matrixL();
    covDet = cov.determinant();
    if (covDet)
        coeff = 1.0/(2*PI*std::sqrt(covDet));
    else
        coeff = 1.;
    srand(time(NULL));
}

Gaussian::Gaussian(const Gaussian &other)
{
    dim = other.getDim();
    mean.noalias() = other.getMeanVector();
    cov.noalias() = other.getCovarianceMatrix();
    Eigen::LLT<Eigen::MatrixXd> lltOfcov(cov);
    cholesky_cov = lltOfcov.matrixL();
    covDet = cov.determinant();
    coeff = 1.0/(2*PI*std::sqrt(covDet));
    srand(time(NULL));
}

Gaussian& Gaussian::operator =(const Gaussian& other)
{
    dim = other.getDim();
    mean.noalias() = other.getMeanVector();
    cov.noalias() = other.getCovarianceMatrix();
    cholesky_cov.noalias() = other.getCholeskyCovarianceMatrix();
    covDet = cov.determinant();
    coeff = 1.0/(2*PI*sqrt(covDet));

    return *this;
}

void Gaussian::init(const Eigen::VectorXd &mean_vector, const Eigen::MatrixXd &covariance_matrix)
{
    assert(mean_vector.size()==covariance_matrix.rows() && mean_vector.size()==covariance_matrix.cols());

    dim = mean_vector.size();
    mean.noalias() = mean_vector;
    cov.noalias() = covariance_matrix;
    Eigen::LLT<Eigen::MatrixXd> lltOfcov(cov);
    cholesky_cov = lltOfcov.matrixL();
    covDet = cov.determinant();
    coeff = 1.0/(2*PI*std::sqrt(covDet));
}

void Gaussian::init(const Gaussian &other)
{
    dim = other.getDim();
    mean.noalias() = other.getMeanVector();
    cov.noalias() = other.getCovarianceMatrix();
    cholesky_cov.noalias() = other.getCholeskyCovarianceMatrix();
    covDet = cov.determinant();
    coeff = 1.0/(2*PI*sqrt(covDet));
}

void Gaussian::updateAllFromCholeskyCov()
{
    cov.noalias() = cholesky_cov*cholesky_cov.transpose();
    covDet = cov.determinant();
    coeff = 1.0/(2*PI*std::sqrt(covDet));
}

Eigen::VectorXd Gaussian::addNoise(const Eigen::VectorXd &vin)
{
    assert(vin.size() == dim);

    const int size = vin.size();
    Eigen::VectorXd noise(size);
    for(int i=0; i<size; i++)
        noise(i) = RANDN;

    return vin + (cholesky_cov*noise);
}
