#include "hogpeopledetector.h"
#include "particlefilterdemonstrator.h"
#include <ctime>
#include <fstream>

//#define SEQ_PATH "/home/emoussy/dataset/1/lbugtrial%04d.jpg"
#define SEQ_PATH "/home/emoussy/dataset/PETS/DataSet1/images/EnterExitCrossingPaths1cor%04d.jpg"
//#define SEQ_PATH "/home/emoussy/dataset/PETS/TUD_Stadtmitte/DaMuliview-seq%04d.png"
//#define SEQ_PATH "/home/emoussy/dataset/PETS/DataSet2/OneShopOneWait1cor.mpg"
//#define SEQ_PATH "/home/emoussy/dataset/PETS/Crowd_PETS09/S2/L1/Time_12-34/View_001/frame_%04d.jpg"
//#define SEQ_PATH "/home/emoussy/dataset/une_porte/image-%05d.png"
//#define SEQ_PATH "/home/emoussy/dataset/PETS/Crowd_PETS09/S2/L1/Time_12-34/View_001/frame_%04d.jpg"
//#define SEQ_PATH "/home/emoussy/dataset/ADL-Rundle-6/img1/%06d.jpg"
//#define SEQ_PATH "/home/emoussy/dataset/ADL-Rundle-8/img1/%06d.jpg"

#define MOT
#define HOG

#ifndef HOG
#include <QFile>
#include <QTextStream>
#include <QString>

#define FILENAME "/home/emoussy/Bureau/Paper_Results/EnterExitCrossingPaths1cor_ACF.txt"

std::vector<Observation> detect(const cv::Mat& f, const int img, const QString& fileName)
{
    std::vector<Observation> detecs ;
    QFile file(fileName);
    if(file.open(QFile::ReadOnly))
    {
        QTextStream is(&file);

        int frame;
        double x, y, w, h;

        QString line = is.readLine();

        while(!line.isNull())
        {
            frame = line.left(line.indexOf(QString(";"))).toInt();
            line.remove(0, line.indexOf(QString(";"))+1);
            if (frame == img)
            {
                x = line.left(line.indexOf(QString(";"))).toDouble();
                line.remove(0, line.indexOf(QString(";"))+1);
                y = line.left(line.indexOf(QString(";"))).toDouble();
                line.remove(0, line.indexOf(QString(";"))+1);
                w = line.left(line.indexOf(QString(";"))).toDouble();
                line.remove(0, line.indexOf(QString(";"))+1);
                h = line.left(line.indexOf(QString(";"))).toDouble();
                line.remove(0, line.indexOf(QString(";"))+1);
                int x1 = (x >=0 && x < f.cols) ? x : ((x < 0) ? 1 : f.cols-1);
                int y1 = (y >=0 && y < f.rows) ? y : ((y < 0) ? 1 : f.rows-1);
                int width1 = (w + x1 < f.cols) ? w : f.cols-x1-1;
                int height1 = (h + y1 < f.rows) ? h : f.rows-y1-1;
                cv::Rect rect(x1,y1,width1,height1);
                Observation rec(f, rect);
                detecs.push_back(rec);
            } else if (frame > img)
                return detecs;
            line = is.readLine();
        }
    }
    return detecs;
}

void adjustValues(std::vector<Observation>& detections, const cv::Mat& frame)
{
    const unsigned int size = detections.size();
    for (unsigned int i=0; i<size; i++)
    {
        Observation& obs = detections[i];
        const cv::Rect rec = obs.getBoundingBox();
        int x1 = (rec.x >=0 && rec.x < frame.cols) ? rec.x : ((rec.x < 0) ? 0 : frame.cols-1);
        int y1 = (rec.y >=0 && rec.y < frame.rows) ? rec.y : ((rec.y < 0) ? 0 : frame.rows-1);
        int width1 = (rec.width + x1 < frame.cols) ? rec.width : frame.cols-x1;
        int height1 = (rec.height + y1 < frame.rows) ? rec.height : frame.rows-x1;
        obs.setX(x1);
        obs.setY(y1);
        obs.setWidth(width1);
        obs.setHeight(height1);
        obs.setFrame(frame);
    }
}

#endif

#ifndef MOT

int main()
{
    cv::VideoCapture cap(SEQ_PATH);
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    int nbframe = 1;
    cv::Mat frame;
    cap >> frame;
    int codec = CV_FOURCC('X','V','I','D');
    cv::VideoWriter writer("new_PF.avi", codec, 5, cv::Size(frame.cols, frame.rows));

    HOGPeopleDetector detector;
    std::vector<Observation> detections;

    int i=0;

    while (i!=14 && cap.read(frame))
    {
        detector.detect(frame);
        detections = detector.getDetections();
        i++;
    }

    ParticleFilter pf(frame, 50);
    pf.initParticles(frame, detections.at(0).getBoundingBox(), 4, true);
    cv::Rect boundingBox = pf.getBoundingBox();
    cv::rectangle(frame, boundingBox, cv::Scalar(255, 0, 0));

    writer << frame;

    cv::namedWindow("output", cv::WINDOW_AUTOSIZE);
    cv::imshow("output", frame);

    clock_t start = clock();
    while(cap.read(frame) && cv::waitKey(30) == -1 && nbframe++)
    {
        detector.detect(frame);
        detections.clear();
        detections = detector.getDetections();
        pf.step(frame, detections);

        boundingBox = pf.getBoundingBox();
        cv::rectangle(frame, boundingBox, cv::Scalar(255, 0, 0));
        for (unsigned int i=0; i<detections.size(); i++)
            cv::rectangle(frame, detections.at(i).getBoundingBox(), cv::Scalar(0, 0, 255));

        writer << frame;

        cv::namedWindow("output", cv::WINDOW_AUTOSIZE);
        cv::imshow("output", frame);
    }

    clock_t finish = clock();
    std::cout << "time = " << (float)(finish - start)/CLOCKS_PER_SEC << std::endl;
    std::cout << "FPS = " << (float)(finish - start)/(CLOCKS_PER_SEC*(float)nbframe) << std::endl;

    return 0;
}

#else

int main()
{
    cv::VideoCapture cap(SEQ_PATH);
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    int nbframe = 0;
    cv::Mat frame;
    cap >> frame;
    std::ofstream os;
    os.open("EnterExitCrossingPaths1cor_HOG_PF125.txt");
    int codec = CV_FOURCC('X','V','I','D');
    cv::VideoWriter writer("EnterExitCrossingPaths1cor_HOG_PF125.avi", codec, 25, cv::Size(frame.cols, frame.rows));
#ifdef HOG
    HOGPeopleDetector detector;
#endif
    std::vector<Observation> detections;

    clock_t start = clock();
    do {
#ifdef HOG
        detector.detect(frame);
        detections = detector.getDetections();
#else
        detections = detect(frame, nbframe, FILENAME);
#endif

        os << "frame : " << nbframe << std::endl;
    }while (detections.size()==0 && cap.read(frame) && nbframe++);

    ParticleFilterDemonstrator demo;
    demo.step(frame, detections);
    std::vector<ParticleFilter> boundingBox = demo.getTracks();
    for (unsigned int i=0; i< boundingBox.size(); i++)
    {
        os << "ID=" << boundingBox[i].getID() << "; X=" << boundingBox[i].getBoundingBox().x << "; Y="
           << boundingBox[i].getBoundingBox().y << "; W=" << boundingBox[i].getBoundingBox().width << "; H="
           << boundingBox[i].getBoundingBox().height << std::endl;
        cv::rectangle(frame, boundingBox[i].getBoundingBox(), cv::Scalar(0, 0, 255));
    }

    std::stringstream ss;
    ss << nbframe;
    cv::putText(frame, ss.str(), cv::Point(10,50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,255,0));
    writer << frame;

    cv::namedWindow("output", cv::WINDOW_AUTOSIZE);
    cv::imshow("output", frame);

    while(cap.read(frame) && cv::waitKey(30) == -1)
    {
        nbframe++;
        os << "frame : " << nbframe << std::endl;
#ifdef HOG
        detector.detect(frame);
#endif
        detections.clear();
#ifdef HOG
        detections = detector.getDetections();
#else
        detections = detect(frame, nbframe, FILENAME);
#endif
        demo.step(frame, detections);

        demo.showActiveTracks(frame);
        std::vector<ParticleFilter> boundingBox = demo.getTracks();
        const unsigned int bbSize = boundingBox.size();
        for (unsigned int i=0; i< bbSize; i++)
        {
            const cv::Rect rec = boundingBox[i].getBoundingBox();
            os << "ID=" << boundingBox[i].getID() << "; X=" << rec.x << "; Y="
               << rec.y << "; W=" << rec.width << "; H=" << rec.height << std::endl;
        }
        const unsigned int dSize = detections.size();
        for (unsigned int i=0; i<dSize; i++)
            cv::rectangle(frame, detections.at(i).getBoundingBox(), cv::Scalar(0, 0, 255));

        std::stringstream ss;
        ss << nbframe;
        cv::putText(frame, ss.str(), cv::Point(10,50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,255,0));
        writer << frame;

        cv::namedWindow("output", cv::WINDOW_AUTOSIZE);
        cv::imshow("output", frame);
    }
    os.close();
    clock_t finish = clock();
    std::cout << "time = " << (float)(finish - start)/CLOCKS_PER_SEC << std::endl;
    std::cout << "FPS = " << (float)(CLOCKS_PER_SEC*(float)nbframe)/(finish - start) << std::endl;

    return 0;
}

#endif
