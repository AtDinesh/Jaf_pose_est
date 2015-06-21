/*
 * Copyright 2012 Dennis Mitzel
 *
 * Authors: Dennis Mitzel
 * Computer Vision Group RWTH Aachen.
 */

#include "Hypo.h"
#include "AncillaryMethods.h"

Hypo& Hypo::operator=(const Hypo &hypo)
{
    if (this == &hypo)
        return *this;

    m_vvIdxC = hypo.m_vvIdxC;
    m_nCateg = hypo.m_nCateg;
    m_vV = hypo.m_vV;
    m_vR = hypo.m_vR;
    m_dScoreW = hypo.m_dScoreW;
    m_dScoreMDL = hypo.m_dScoreMDL;
    m_dNW = hypo.m_dNW;
    m_dSpeed = hypo.m_dSpeed;
    m_dHeight = hypo.m_dHeight;
    m_bMoving = hypo.m_bMoving;
    m_vStart = hypo.m_vStart;
    m_vEnd = hypo.m_vEnd;
    m_mXProj = hypo.m_mXProj;
    m_mRot4D = hypo.m_mRot4D;
    m_vDir = hypo.m_vDir;
    m_mStartRect = hypo.m_mStartRect;
    m_mEndRect = hypo.m_mEndRect;
    m_vmTrajRect = hypo.m_vmTrajRect;
    m_mTrajPts = hypo.m_mTrajPts;
    m_vTrajT = hypo.m_vTrajT;
    m_mBbox4D = hypo.m_mBbox4D;
    n_HypoID = hypo.n_HypoID;
    n_ParentID = hypo.n_ParentID;
    b_terminationFlag = hypo.b_terminationFlag;
    n_lastSelected = hypo.n_lastSelected;
    m_colHists = hypo.m_colHists;
    m_stateCovMats = hypo.m_stateCovMats;
    return *this;
}


Hypo::Hypo()
{
    b_terminationFlag = false;
    m_bMoving = false;
    n_ParentID = -1;
    n_lastSelected = 0;
}

Hypo::Hypo(const Hypo& hypo)
{
    m_vvIdxC = hypo.m_vvIdxC;
    m_nCateg = hypo.m_nCateg;
    m_vV = hypo.m_vV;
    m_vR = hypo.m_vR;
    m_dScoreW = hypo.m_dScoreW;
    m_dScoreMDL = hypo.m_dScoreMDL;
    m_dNW = hypo.m_dNW;
    m_dSpeed = hypo.m_dSpeed;
    m_dHeight = hypo.m_dHeight;
    m_bMoving = hypo.m_bMoving;
    m_vStart = hypo.m_vStart;
    m_vEnd = hypo.m_vEnd;
    m_mXProj = hypo.m_mXProj;
    m_mRot4D = hypo.m_mRot4D;
    m_vDir = hypo.m_vDir;
    m_mStartRect = hypo.m_mStartRect;
    m_mEndRect = hypo.m_mEndRect;
    m_vmTrajRect = hypo.m_vmTrajRect;
    m_mTrajPts = hypo.m_mTrajPts;
    m_vTrajT = hypo.m_vTrajT;
    m_mBbox4D = hypo.m_mBbox4D;
    n_HypoID = hypo.n_HypoID;
    n_ParentID = hypo.n_ParentID;
    b_terminationFlag = hypo.b_terminationFlag;
    n_lastSelected = hypo.n_lastSelected;
    m_colHists = hypo.m_colHists;
    m_stateCovMats = hypo.m_stateCovMats;
}

Hypo::~Hypo()
{

}

// ***********************************************************************
//   Idx
// ***********************************************************************

void Hypo::setIdx(const Vector <FrameInlier>& valO)
{
    m_vvIdxC = (valO);
}

void Hypo::getIdx(Vector <FrameInlier>& target)
{
    target = (m_vvIdxC);
}

// ***********************************************************************
//   ScoreW
// ***********************************************************************

void Hypo::setScoreW(const double scoreO)
{
    m_dScoreW = scoreO;
}

double Hypo::getScoreW()
{
    return m_dScoreW;
}

// ***********************************************************************
//   ScoreMDL
// ***********************************************************************

void Hypo::setScoreMDL(const double scoreO)
{
    m_dScoreMDL = scoreO;
}
double Hypo::getScoreMDL()
{
    return m_dScoreMDL;
}

// ***********************************************************************
//   NW
// ***********************************************************************

void Hypo::setNW(const double nwO)
{
    m_dNW = nwO;
}

double Hypo::getNW()
{
    return m_dNW;
}

// ***********************************************************************
//   Speed
// ***********************************************************************

void Hypo::setSpeed(const double speedO)
{
    m_dSpeed = speedO;
}

double Hypo::getSpeed()
{
    return m_dSpeed;
}

// ***********************************************************************
//   Height
// ***********************************************************************

void Hypo::setHeight(const double heightO)
{
    m_dHeight.pushBack(heightO);
}

double Hypo::getHeight()
{
    return (m_dHeight.sum()/(double(m_dHeight.getSize())));
}

// ***********************************************************************
//   Moving
// ***********************************************************************

void Hypo::setMoving(bool moveO)
{
    m_bMoving = moveO;
}

bool Hypo::isMoving()
{
    return m_bMoving;
}

// ***********************************************************************
//   Category
// ***********************************************************************

void Hypo::setCategory(int categO)
{
    m_nCateg = categO;
}

int Hypo::getCategory()
{
    return m_nCateg;
}

// ***********************************************************************
//   V
// ***********************************************************************

void Hypo::setV(const Vector<double>& srcO)
{
    m_vV = srcO;
}

void Hypo::getV(Vector<double>& targetO)
{
    targetO = m_vV;
}

// ***********************************************************************
//   R
// ***********************************************************************

void Hypo::setR(const Vector<double>& srcO)
{
    m_vR = srcO;
}

void Hypo::getR(Vector<double>& targetO)
{
    targetO = m_vR;
}

// ***********************************************************************
//   Start
// ***********************************************************************

void Hypo::setStart(const Vector<double>& srcO)
{
    m_vStart = srcO;
}

void Hypo::getStart(Vector<double>& targetO)
{
    targetO = m_vStart;
}

// ***********************************************************************
//   End
// ***********************************************************************

void Hypo::setEnd(const Vector<double>& srcO)
{
    m_vEnd = srcO;
}

void Hypo::getEnd(Vector<double>& targetO)
{
    targetO = m_vEnd;
}

// ***********************************************************************
//   XProj
// ***********************************************************************

void Hypo::setXProj(const Matrix<double>& srcO)
{
    m_mXProj = srcO;
}

void Hypo::getXProj(Matrix<double>& targetO)
{
    targetO = m_mXProj;
}

// ***********************************************************************
//   Rot4D
// ***********************************************************************

void Hypo::setRot4D(const Matrix<double>& srcO)
{
    m_mRot4D = srcO;
}

void Hypo::getRot4D(Matrix<double>& targetO)
{
    targetO = m_mRot4D;
}

// ***********************************************************************
//   Dir
// ***********************************************************************

void Hypo::setDir(const Vector<double>& srcO)
{
    m_vDir = srcO;
}

void Hypo::getDir(Vector<double>& targetO)
{
    targetO = m_vDir;
}

// ***********************************************************************
//   StartRect
// ***********************************************************************

void Hypo::setStartRect(const Matrix<double>& srcO)
{
    m_mStartRect = srcO;
}

void Hypo::getStartRect(Matrix<double>& targetO)
{
    targetO = m_mStartRect;
}

// ***********************************************************************
//   EndRect
// ***********************************************************************

void Hypo::setEndRect(const Matrix<double>& srcO)
{
    m_mEndRect = srcO;
}

void Hypo::getEndRect(Matrix<double>& targetO)
{
    targetO = m_mEndRect;
}

// ***********************************************************************
//   TrajRect
// ***********************************************************************

void Hypo::setTrajRect(const Vector<Matrix <double> >& srcO)
{
    m_vmTrajRect = srcO;
}

void Hypo::getTrajRect(Vector< Matrix<double> >& targetO)
{
    targetO = m_vmTrajRect;
}

// ***********************************************************************
//   TrajPts
// ***********************************************************************

void Hypo::setTrajPts(const Vector<Vector <double> >& srcO)
{
    m_mTrajPts = srcO;
}

void Hypo::getTrajPts(Vector<Vector <double> >& targetO)
{
    targetO = m_mTrajPts;
}

// ***********************************************************************
//   TrajT
// ***********************************************************************

void Hypo::setTrajT(const Vector<int>& srcO)
{
    m_vTrajT = srcO;
}

void Hypo::getTrajT(Vector<int>& targetO)
{
    targetO = m_vTrajT;
}

// ***********************************************************************
//   BBOX
// ***********************************************************************

void Hypo::setBBox4D(const Matrix<double>& srcO)
{
    m_mBbox4D = srcO;
}

void Hypo::getBBox4D(Matrix<double>& targetO)
{
    targetO = m_mBbox4D;
}

// ***********************************************************************
//   HypoID
// ***********************************************************************
int Hypo::getHypoID()
{
    return n_HypoID;
}

void Hypo::setHypoID(int hypoId)
{
    n_HypoID = hypoId;
}

// ***********************************************************************
//   ParentID
// ***********************************************************************
int Hypo::getParentID()
{
    return n_ParentID;
}

void Hypo::setParentID(int parentId)
{
    n_ParentID = parentId;
}
// ***********************************************************************
//   Termination Flag
// ***********************************************************************
bool Hypo::isTerminated()
{
    return b_terminationFlag;
}

void Hypo::setAsTerminated(bool terminated)
{
    b_terminationFlag = terminated;
}

// ***********************************************************************
//   Frame when Hypo was Last Selected.
// ***********************************************************************
int Hypo::getLastSelected()
{
    return n_lastSelected;
}

void Hypo::setLastSelected(int frame)
{
    n_lastSelected = frame;
}


void Hypo::getColHists(Vector<Volume<double> >& colHists)
{
    colHists.clearContent();
    for( int i = 0; i < m_colHists.getSize(); i++)
    {
        colHists.pushBack(m_colHists(i));
    }

}

void Hypo::setColHists(Vector<Volume<double> >& colHists)
{
    m_colHists.clearContent();
    for( int i = 0; i < colHists.getSize(); i++)
    {
        m_colHists.pushBack(colHists(i));
    }
}

void Hypo::getStateCovMats(Vector<Matrix<double> >& covMats)
{
    covMats = m_stateCovMats;
}

void Hypo::setStateCovMats(Vector<Matrix<double> >& covMats)
{
    m_stateCovMats = covMats;
}
