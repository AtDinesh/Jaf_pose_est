#ifndef GAUSSIAN_H
#define GAUSSIAN_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <math.h>
#include <time.h>

#define PI 3.14159265358979323846264338327950288419716939937510

/**
  \def RAND
  \brief A macro  which returns a pseudo-random number from a uniform distribution on the interval [0 1].
*/
#define RAND ((double) rand())/((double)RAND_MAX)
/**
  \def TWOPI
  \brief 2.0*pi.
*/
#define TWOPI 2.0*3.14159265358979323846264338327950288419716939937510
/**
  \def RANDN
  \brief A macro which returns a pseudo-random number from a normal distribution with mean zero and standard
  deviation one. This macro uses Box Muller's algorithm
*/
#define RANDN std::sqrt(-2.0*std::log(RAND))*cos(TWOPI*RAND)

extern "C" {
void dgesvd_(char const* jobu, char const* jobvt,
             int const* m, int const* n, double* a, int const* lda,
             double* s, double* u, int const* ldu,
             double* vt, int const* ldvt,
             double* work, int const* lwork, int* info);
}

/**
 * @brief This function computes the pseudo inverse of a matrix.
 * @param A : The matrix of which the pseudo inverse will be computed.
 * @param inv : The output pseudo inverse of A.
 * @param threshold : The precision of the computed pseudo inverse.
 */
static inline void pseudoInverse(const Eigen::MatrixXd& A, Eigen::MatrixXd& inv, const double threshold = 1e-4)
{
    bool toTranspose = false;
    Eigen::MatrixXd mat = A;
    if (mat.rows() < mat.cols())
    {
        mat.transposeInPlace();
        toTranspose = true;
    }
    const unsigned int NR = mat.rows();
    const unsigned int NC = mat.cols();
    Eigen::MatrixXd U, VT;
    U.resize(NR,NR);
    U.setZero();
    VT.resize(NC,NC);
    VT.setZero();
    Eigen::VectorXd s;
    s.resize(std::min(NR,NC));
    s.setZero();
    char Jobu='A';
    char Jobvt='A';
    const int m = NR;
    const int n = NC;
    int linfo;
    int lda = std::max(m,n);
    int lw=-1;
    {
        double vw;
        dgesvd_(&Jobu, &Jobvt, &m, &n,
                mat.data(), &lda,
                0, 0, &m, 0, &n, &vw, &lw, &linfo);
        lw = int(vw)+5;
    }
    Eigen::VectorXd w;
    w.resize(lw);
    w.setZero();
    int lu = U.rows();
    int lvt = VT.rows();
    dgesvd_(&Jobu, &Jobvt, &m, &n,
            mat.data(), &lda,
            s.data(),
            U.data(),
            &lu,
            VT.data(), &lvt,
            w.data(), &lw, &linfo);
    Eigen::MatrixXd S;
    S.resize(mat.cols(), mat.rows());
    S.setZero();
    const int cols = mat.cols();
    const int rows = mat.rows();
    for (int i=0;i<cols;i++)
    {
        for (int j=0; j<rows;j++)
        {
            if ((i==j) && (fabs(s(i))>threshold))
                S(i,i) = 1./s(i);
            else
                S(i,j) = 0;
        }
    }
    Eigen::MatrixXd tmp1;
    tmp1 = S*(U.transpose());
    inv = (VT.transpose())*tmp1;
    if (toTranspose)
        inv.transposeInPlace();
}

/**
 * @brief The Gaussian class implements an abstract class to create multidimentional gaussians.
 */
class Gaussian
{
    public:
        /**
         * @brief Constructor.
         */
        Gaussian();
        /**
         * @brief Constructor.
         * @param mean_vector : The two dimentional mean vector.
         * @param covariance_matrix : The covariance matrix.
         */
        Gaussian(const Eigen::VectorXd& mean_vector, const Eigen::MatrixXd& covariance_matrix);
        /**
         * @brief This constructor initializes a Gaussian object from another one.
         * @param other : The other Gaussian object.
         */
        Gaussian(const Gaussian& other);
        /**
          * @brief Destructor.
          */
        ~Gaussian() {}

        /**
         * @brief This method initializes the Gaussian object using a mean vector and a covariance matrix.
         * @param mean_vector : The mean vector.
         * @param covariance_matrix : The covariance matrix.
         */
        void init(const Eigen::VectorXd& mean_vector, const Eigen::MatrixXd& covariance_matrix);
        /**
         * @brief This method initializes the Gaussian2D object using another one.
         * @param other : The other Gaussian2D object.
         */
        void init(const Gaussian& other);

        /**
         * @brief This operator implements the affectation of a Gaussian object to another one.
         * @param other : The other Gaussian object.
         * @return The newly affected object.
         */
        Gaussian& operator=(const Gaussian& other);

        /**
         * @brief This method returns the mean vector of the gaussian.
         * @return The mean vector of the gaussian.
         */
        inline const Eigen::VectorXd getMeanVector() const { return mean; }
        /**
         * @brief This method returns teh covariance matrix of the gaussian.
         * @return The covariance matrix of the gaussian.
         */
        inline const Eigen::MatrixXd getCovarianceMatrix() const { return cov; }
        /**
         * @brief This method returns the cholesky transformation of the covariance matrix.
         * @return The cholesky transformation ct of the covariance matrix cov : \f$ cov = ct * ct' \f$
         */
        inline const Eigen::MatrixXd getCholeskyCovarianceMatrix() const { return cholesky_cov; }
        /**
         * @brief This method evaluates a given vector.
         * @param vect : The given vector to evaluate.
         * @return \f$ \frac{1.0}{2* \pi * \sqrt{covDet} } * \exp{-0.5 * (invCov(0,0) * (vect(0)-mean(0))² +  invCov(1,1) * (vect(1)-mean(1))² + (invCov(0,1)+invCov(1,0) * (vect(0)-mean(0)*(vect(1)-mean(1)))} \f$
         */
        inline double evaluate(const Eigen::VectorXd& vect)
        {
            assert(vect.size() == dim);
            Eigen::VectorXd diff = vect - mean;
            Eigen::MatrixXd invCov(cov.rows(), cov.cols());
            invCov.setZero();
            if (cov.determinant())
                invCov = cov.inverse();
            else
                pseudoInverse(cov, invCov);
            double res = 0.;

            for(int i=0; i < dim; i++)
                for(int j=0; j<dim; j++)
                    res+=diff(i)*diff(j)*invCov(i,j);

            return coeff*std::exp(-0.5*res);
        }
        /**
         * @brief This method adds noise to the vector vin and returns the results.
         * @param vin : The input vector on which the noise will be added.
         * @return The vector vin + noise.
         */
        Eigen::VectorXd addNoise(const Eigen::VectorXd& vin);
        /**
         * @brief This method returns the dimension of the gaussian
         * @return The dimension of the gaussian.
         */
        inline int getDim() const { return dim; }

    private:
        /**
         * @brief This method updates the covariance matrix from the cholesky_cov. It also updates covDet and coeff.
         */
        void updateAllFromCholeskyCov();

        /**
         * @brief The mean vector.
         */
        Eigen::VectorXd mean;
        /**
         * @brief The covariance matrix.
         */
        Eigen::MatrixXd cov;
        /**
         * @brief cov = cholesky_cov * cholesky_cov'
         */
        Eigen::MatrixXd cholesky_cov;
        /**
         * @brief Determinant of the covariance matrix.
         */
        double covDet;
        /**
         * @brief \f$ coeff = \frac{1.0}{2* \pi * \sqrt{covDet} } \f$
         */
        double coeff;
        /**
         * @brief The dimension of the gaussian.
         */
        int dim;
};

#endif // GAUSSIAN_H
