#include "motiondistribution.h"
#include <assert.h>
#include "cudaimplementation.h"

// conversion tab used to speed conversion from motion level (0 .. 255) to bin value
static Eigen::VectorXi bins_tab(256);

// flag that indicates that the initialization is done
static bool binsOK=false;

static void InitBinsTbl(int nbbins)
{
    if(binsOK)
        return;

    // Init conversion table of a component (0..255) to a bin value (0..7)
    for (int i = 0; i < 256; i++)
        bins_tab[i] = (int)(((float)i/256.0)*(float)nbbins);

    binsOK=true;
}

MotionDistribution::MotionDistribution(const int nbOfBins, const int nbOfParts, const int width, const int height)
{
    data.resize(nbOfParts, nbOfBins);
    uniformHist.resize(nbOfBins);
    uniformHist.setConstant(1.0/(double)nbOfBins);
    binsMask.resize(height, width);
    binsMask.setConstant(-1);
    InitBinsTbl(nbOfBins);
}

void MotionDistribution::calcFromRect(const cv::Mat& img, const cv::Rect& rec, const int histNum, const double increment)
{
    assert(histNum >= 0 && histNum < data.rows() && rec.x+rec.width < binsMask.cols());

    data.row(histNum).setConstant(0.);

    const int x = rec.x;
    const int y = rec.y;
    const int imax = y+rec.height;
    const int jmax = x+rec.width;
    for(register int i=y;i<imax;i++)
    {
        register unsigned char * ptr = (unsigned char*)img.data+i*img.step+x;

        for (register int j=x;j<jmax;j++)
        {
            // Compute the bins for each color
            if(binsMask(i-y,j-x)==-1)
                binsMask(i-y,j-x) = bins_tab[(int)(*(ptr))];

            data(histNum,binsMask(i-y,j-x)) += increment;
            ptr++;
        }
    }
}

void MotionDistribution::calcFromRectList(const cv::Mat &img, const cv::Rect& rec, const Eigen::VectorXi& nbPix)
{
    const int rows = data.rows();
    assert(nbPix.size() == rows);

    for (int i=0; i< rows; i++)
        calcFromRect(img, rec, i, 1./(double)nbPix(i));
}

double MotionDistribution::bhattacharayaDistance(const double sigma)
{
    double res=1.;
    const int rows = data.rows();
    const int cols = data.cols();
    for (register int j=0; j<rows; j++)
    {
        double final=0;

        for(register int i=0;i<cols;i++)
            final += std::sqrt(data(j,i)*uniformHist(i));

        res *= std::exp(-((1.0-final)/sigma));
    }

    return res;
}
