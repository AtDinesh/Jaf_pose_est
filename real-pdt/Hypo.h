/*
 * Copyright 2012 Dennis Mitzel
 *
 * Authors: Dennis Mitzel
 * Computer Vision Group RWTH Aachen.
 */

#ifndef _HYPO_DENNIS_H
#define	_HYPO_DENNIS_H
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include "Vector.h"
#include "Matrix.h"
#include "Globals.h"
#include "FrameInlier.h"
#include "Volume.h"
#include "camera/Camera.h"
using namespace std;

//+++++++++++++++++++++++++++++++ Definition +++++++++++++++++++++++++++++++++
class Hypo
{
    
public:
    Hypo();
    Hypo(const Hypo& copy);
    
    ~Hypo();

    void setIdx(const Vector <FrameInlier>& srcO);
    void getIdx(Vector <FrameInlier>& targetO);

    void setScoreW(const double scoreO);
    double getScoreW();

    void setScoreMDL(const double scoreO);
    double getScoreMDL();

    void setNW(const double nwO);
    double getNW();

    void setSpeed(const double speedO);
    double getSpeed();

    void setHeight(const double heightO);
    double getHeight();

    void setMoving(bool moveO);
    bool isMoving();

    void setCategory(int categO);
    int getCategory();

    void setV(const Vector<double>& srcO);
    void getV(Vector<double>& targetO);

    void setR(const Vector<double>& srcO);
    void getR(Vector<double>& targetO);

    void setStart(const Vector<double>& srcO);
    void getStart(Vector<double>& targetO);

    void setEnd(const Vector<double>& srcO);
    void getEnd(Vector<double>& targetO);

    void setXProj(const Matrix<double>& srcO);
    void getXProj(Matrix<double>& targetO);

    void setRot4D(const Matrix<double>& srcO);
    void getRot4D(Matrix<double>& targetO);

    void setDir(const Vector<double>& srcO);
    void getDir(Vector<double>& targetO);

    void setStartRect(const Matrix<double>& srcO);
    void getStartRect(Matrix<double>& targetO);

    void setEndRect(const Matrix<double>& srcO);
    void getEndRect(Matrix<double>& targetO);

    void setTrajRect(const Vector<Matrix <double> >& srcO);
    void getTrajRect(Vector< Matrix<double> >& targetO);

    void setTrajPts(const Vector<Vector <double> >& srcO);
    void getTrajPts(Vector<Vector<double> >& targetO);

    void setTrajT(const Vector<int>& srcO);
    void getTrajT(Vector<int>& targetO);

    void setBBox4D(const Matrix<double>& srcO);
    void getBBox4D(Matrix<double>& targetO);

    int getHypoID();
    void setHypoID(int hypoId);

    int getParentID();
    void setParentID(int parentId);

    bool isTerminated();
    void setAsTerminated(bool terminated);

    int getLastSelected();
    void setLastSelected(int frame);

    Hypo& operator=(const Hypo &hypo);

    //Kalman

    void getColHists(Vector<Volume<double> >& colHists);
    void setColHists(Vector<Volume<double> >& colHists);

    void getStateCovMats(Vector<Matrix<double> >& covMats);
    void setStateCovMats(Vector<Matrix<double> >& covMats);

protected:

    void generateInitialModel(Vector<Vector<double> >& pts, Vector<Vector<int> >& occBins , Camera cam);
    void updateGCT(Vector<Vector<double> >& pointsForReg, Vector<Vector<int> > &occBins, Camera camcnt, int frame);


    Vector <FrameInlier> m_vvIdxC;
    int m_nCateg;
    Vector <double> m_vV;
    Vector <double> m_vR;
    double m_dScoreW;
    double m_dScoreMDL;
    double m_dNW;
    double m_dSpeed;
    Vector<double> m_dHeight;
    bool m_bMoving;
    Vector <double> m_vStart;
    Vector <double> m_vEnd;
    Matrix <double> m_mXProj;
    Matrix <double> m_mRot4D;
    Vector <double> m_vDir;

    Matrix<double> m_mStartRect;
    Matrix<double> m_mEndRect;

    Vector <Matrix <double> > m_vmTrajRect;
    Vector<Vector <double> > m_mTrajPts;
    Vector <int> m_vTrajT;

    Matrix <double> m_mBbox4D;

    //Kalman results

    Vector<Matrix<double> > m_stateCovMats;
    Vector<Volume<double> > m_colHists;

    int n_HypoID;
    int n_ParentID;

    bool b_terminationFlag;
    int n_lastSelected;

};

#endif	/* _HYPO_DENNIS_H */

