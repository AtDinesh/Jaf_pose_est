#include "particle.h"
#include "gaussian.h"
#include <assert.h>

Particle::Particle(const cv::Mat &frame) :
    md(8, 2, frame.cols, frame.rows)
{
    weight = 0.;
    prevWeight = 0.;
    hist_initialized = false;
}

void Particle::resetContinuousParameters()
{
    continuousParams.setZero();
    previousContinuousParams.setZero();
    hist_initialized = false;
}

void Particle::init(const Eigen::VectorXd &continuousParams, const Eigen::MatrixXd &continuousParamsRange, const double weight)
{
    assert(continuousParams.rows() == continuousParamsRange.rows() && continuousParamsRange.cols() == 2);

    this->continuousParams.noalias() = continuousParams;
    this->continuousParamsRange.noalias() = continuousParamsRange;
    firstContParams.noalias() = continuousParams;
    previousContinuousParams.noalias() = continuousParams;
    this->weight = weight;
}

cv::MatND Particle::computeHistogram(const cv::Mat &frame, const cv::Rect &region)
{
    cv::Mat grey;
    const int x1 = (region.x >=0 && region.x < frame.cols) ? region.x : ((region.x < 0) ? 1 : frame.cols-1);
    const int y1 = (region.y >=0 && region.y < frame.rows) ? region.y : ((region.y < 0) ? 1 : frame.rows-1);
    const int width1 = (region.width + x1 < frame.cols) ? region.width : frame.cols-x1-1;
    const int height1 = (region.height + y1 < frame.rows) ? region.height : frame.rows-y1-1;
    const cv::Rect rect1(x1,y1,width1,height1);
    cv::cvtColor(cv::Mat(frame, rect1), grey, cv::COLOR_BGR2HSV);

    // Using 50 bins for hue and 60 for saturation
    const int h_bins = 50; const int s_bins = 60;
    int histSize[] = { h_bins, s_bins };

    // hue varies from 0 to 179, saturation from 0 to 255
    float h_ranges[] = { 0, 180 };
    float s_ranges[] = { 0, 256 };

    const float* ranges[] = { h_ranges, s_ranges };

    // Use the o-th and 1-st channels
    int channels[] = { 0, 1 };

    // Calculate the histograms for the HSV images
    cv::calcHist( &grey, 1, channels, cv::Mat(), hist, 2, histSize, ranges, true, hist_initialized );
    cv::normalize( hist, hist, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );

    Eigen::VectorXi nbPix(2);
    nbPix(0) = region.width*region.height;
    nbPix(1) = region.width*region.height;
    //md.calcFromRectList(frame, region, nbPix);

    hist_initialized = true;

    return hist;
}

double Particle::bhattacharyyaDistance(const cv::MatND &otherHist)
{
    return (1.-cv::compareHist( hist, otherHist, CV_COMP_BHATTACHARYYA ))/**(md.bhattacharayaDistance(0.08))*/;
}

inline double maxi(const double a, const double b)
{
    return (a < b) ? b : a;
}

inline double mini(const double a, const double b)
{
    return (a<b) ? a : b;
}

void Particle::moveParticle(const double width, const double height)
{
    //double x = CST1 * (continuousParams(0) - firstContParams(0)) + CST2 * (previousContinuousParams(0) - firstContParams(0)) + RANDN + firstContParams(0);
    //double y = CST1 * (continuousParams(1) - firstContParams(1)) + CST2 * (previousContinuousParams(1) - firstContParams(1)) + RANDN + firstContParams(1);
    const double x = CST1*continuousParams(0)+ CST2*previousContinuousParams(0)+RANDN;
    const double y = CST1*continuousParams(1)+ CST2*previousContinuousParams(1)+RANDN;

    previousContinuousParams.noalias() = continuousParams;

    continuousParams(0) = maxi(0., mini( (double)width - 1., x));
    continuousParams(1) = maxi(0., mini( (double)height - 1., y));

    if (continuousParams.size()==3)
    {
        if (continuousParams(0)+continuousParams(2)*64.>width)
            continuousParams(2) = (width-continuousParams(0))/64.;
        if (continuousParams(1)+continuousParams(2)*128.>height)
            continuousParams(2) = (height-continuousParams(1))/128.;
    } else {
        if (continuousParams(0)+continuousParams(2)>width)
            continuousParams(2) = width-continuousParams(0);
        if (continuousParams(1)+continuousParams(3)>height)
            continuousParams(3) = height-continuousParams(1);
    }
}
