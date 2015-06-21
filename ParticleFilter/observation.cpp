#include "observation.h"

Observation::Observation(const cv::Mat &frame, const cv::Rect &boundingBox) :
    bb(boundingBox),
    hist_initialized(false)
{
    computeHistogram(frame, boundingBox);
}

cv::MatND Observation::computeHistogram(const cv::Mat &frame, const cv::Rect &region)
{
    this->frame = frame;
    cv::Mat grey;
    const int x1 = (region.x >=0 && region.x < frame.cols) ? region.x : ((region.x < 0) ? 1 : frame.cols-1);
    const int y1 = (region.y >=0 && region.y < frame.rows) ? region.y : ((region.y < 0) ? 1 : frame.rows-1);
    const int width1 = (region.width + x1 < frame.cols) ? region.width : frame.cols-x1-1;
    const int height1 = (region.height + y1 < frame.rows) ? region.height : frame.rows-y1-1;
    const cv::Rect rect1(x1,y1,width1,height1);
    cv::cvtColor(cv::Mat(frame, rect1), grey, cv::COLOR_BGR2HSV);

    bb=region;

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

    hist_initialized = true;

    return hist;
}

double Observation::bhattacharyyaDistance(const cv::MatND &otherHist) const
{
    if (hist_initialized && otherHist.cols==hist.cols && otherHist.rows==hist.rows && otherHist.cols==hist.cols)
        return 1.-cv::compareHist( hist, otherHist, CV_COMP_BHATTACHARYYA );
    return 0.;
}
