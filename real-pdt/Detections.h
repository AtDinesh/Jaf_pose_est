/*
 * Copyright 2012 Dennis Mitzel
 *
 * Authors: Dennis Mitzel
 * Computer Vision Group RWTH Aachen.
 */

#ifndef _DENNIS_DETECTIONS_H
#define	_DENNIS_DETECTIONS_H

#include "camera/Camera.h"
#include "math/Math.h"
#include <CImg/CImg.h>
using namespace cimg_library;

//***************************************************************************************
// Detections are stored in detC as follows
//
//         frame                   detections
//          | |                    | |
//          | |                    | |
//          | |                    | |                      infos about det i.e. 3D pos
// Vector   | |----------> Vector  | |-------------------->| | | | | | | | | | |
//          | |                    | |
//          | |
//***************************************************************************************


//+++++++++++++++++++++++++++++++ Definition +++++++++++++++++++++++++++++++++
class Detections
{
public:

    Detections(int x, const int flag);

    ~Detections();

    //*******************************************************
    // getter methods
    //*******************************************************
    int numberDetectionsAtFrame( int frame);
    void getPos3D( int frame,  int detec, Vector<double>& pos);
    void getBBox( int frame,  int detec, Vector<double>& bbox);
    double getScore( int frame,  int detec);
    double getHeight( int frame,  int detec);
    int getCategory(int frame, int detec);

    //*************************************************************
    // Methode for adding online Detections
    //*************************************************************

    void addHOGdetOneFrame(Vector<Vector <double> >& det, int frame, CImg<unsigned char>& imageLeft, Camera cam);

    int prepareDet(Vector<double> &detContent, Vector<Vector <double> >& det, int i, bool leftDet,
                   Camera cam, Matrix<double> &covariance);

    //*****************************************************************
    // Compute 3D Position out of BBox
    //*****************************************************************
    void compute3DPosition(Vector<double>& detection, Camera cam);
    //*****************************************************************
    // Compute 3D Position out of BBox
    //*****************************************************************

    void computeColorHist(Volume<double>& colHist, Vector<double>& bbox, int nBins, CImg<unsigned char>& imageLeft);

    void getColorHist(int frame, int pos, Volume<double>& colHist);
    //*****************************************************************
    // Compute the 3D uncertainty for a point
    //*****************************************************************
    void compute3DCov(Vector<double> pos3d, Matrix<double> &cov);
    void get3Dcovmatrix(int frame, int pos, Matrix<double>& covariance);

    Vector<double> fromCamera2World(Vector<double> posInCamera, Camera cam);
    bool improvingBBoxAlignment_libelas(Vector<double>& vbbox, double var, Camera camera, Matrix<double>& depthMap);
    Vector<double> projectPlaneToCam(Vector<double> p, Camera cam);

    double get_mediandepth_inradius(Vector<double>& bbox, int radius, Matrix<double>& depthMap, double var, double pOnGp);

protected:

    Vector< Vector < Vector  <double> > > detC;
    Vector< Vector < Matrix  <double> > > cov3d;
    Vector<Vector<Volume<double> > > colHists;

    Camera camLCov;
    Camera camRCov;

    int offSet;
    int img_num, hypo_num, center_x, center_y, scale, categ, bbox, initscore,
    score, dist, height, rot, pos, numberAllAccDetections, numberOfFrames, nrColinDetFile, carOrient, det_id;

};

#endif	/* _DENNIS_DETECTIONS_H */

