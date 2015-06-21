#include "hogpeopledetector.h"

HOGPeopleDetector::HOGPeopleDetector()
{
    setID(-1);
    HOGdesc.setSVMDetector(cv::gpu::HOGDescriptor::getDefaultPeopleDetector());
}

HOGPeopleDetector::HOGPeopleDetector(const cv::Mat &frame, const int ID)
{
    setWidth(frame.cols);
    setHeight(frame.rows);
    setID(ID);
    HOGdesc.setSVMDetector(cv::gpu::HOGDescriptor::getDefaultPeopleDetector());
}

GaussianMixture HOGPeopleDetector::detect(const cv::Mat &frame)
{
    setWidth(frame.cols);
    setHeight(frame.rows);

    clearDetections();

    static const cv::Size s = cv::Size((int)(frame.cols), (int)(frame.rows));
    cv::Mat frame_resize;
    cv::resize(frame,  frame_resize, s);
    cv::Mat frame4(1,1,CV_8UC4);
    cv::cvtColor(frame_resize,frame4,CV_BGR2BGRA);

    cv::gpu::GpuMat m(frame4);
    std::vector<cv::Rect> det;

    HOGdesc.detectMultiScale(m, det);
    std::vector<Observation> detObs;
    const unsigned int detSize = det.size();
    for (unsigned int i=0; i<detSize; i++)
    {
        Observation obs(frame, det[i]);
        detObs.push_back(obs);
    }
    setDetections(detObs);

    GaussianMixture gm(detSize);
    for (unsigned int i=0; i<detSize; i++)
    {
        Eigen::VectorXd mean(2);
        mean[0] = det[i].x + det[i].width/2;
        mean[1] = det[i].y + det[i].height/2;
        Eigen::MatrixXd cov(2,2);
        cov.setZero();

        const int end = det[i].x+det[i].width;
        for(int j=det[i].x; j < end; j++)
        {
            const int end1 = det[i].y+det[i].height;
            for(int k=det[i].y; k < end1; k++)
            {
                Eigen::MatrixXd point(2,1);
                point(0,0) = j - mean[0];
                point(1,0) = k - mean[1];
                cov.noalias() += point*point.transpose();
            }
        }
        cov *= 1./((double)(det[i].width*det[i].height));
        gm.getGaussian(i).init(mean, cov);
    }
    return gm;
}
