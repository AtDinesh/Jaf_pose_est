/*
 * Copyright 2012 Dennis Mitzel
 *
 * Authors: Dennis Mitzel
 * Computer Vision Group RWTH Aachen.
 */

#include "MDL.h"
#include "AncillaryMethods.h"


double MDL::solve_mdl_greedy(Matrix<double>& Q, Vector<double>& m, Vector < Hypo >& HyposMDL, Vector<int>& HypoIdx, Vector < Hypo >& HypoAll)
{
    HyposMDL.clearContent();
    HypoIdx.clearContent();
    int x = Q.x_size();

    m.setSize(x, 0.0);
    Vector<double> result(x);
    Vector<double> bDropped(x, 0.0);

    double bestScore = 0;
    double optScore = 0;
    double oldScore = 0;
    pair<double, int> maxOfResult;

    Matrix<double> copyQ;
    Vector<double> copyM;

    for(int i = 0; i < x; i++)
    {
        copyM = m;
        for(int j = 0; j < x; j++)
        {
            if(m(j) > 0.0 || bDropped(j) > 0) continue;
            copyM(j) = 1.0;

            copyQ = Q;
            copyQ *= copyM;
            copyQ.Transpose();
            copyQ *= copyM;

            result(j) = copyQ(0,0);

            if(result(j) < 0) bDropped(j) = 1.0;
            if(m(j) != 1.0) copyM(j) = 0.0;
        }

        maxOfResult = result.maxim();
        optScore = maxOfResult.first;
        result.fill(0.0);

        if(oldScore < optScore)
        {
            m(maxOfResult.second) = 1.0;
            HyposMDL.pushBack(HypoAll(maxOfResult.second));
            HyposMDL(HyposMDL.getSize()-1).setScoreMDL( optScore - oldScore );
            HypoIdx.pushBack(maxOfResult.second);
            oldScore = optScore;
            bestScore = optScore;
        }
        else
        {
            break;
        }
    }

    return bestScore;
}


void MDL::build_mdl_matrix(Matrix<double>& Q, Vector<Hypo>& hypos, int t, double normfct) {

    //******************************************************************
    // Preparation
    //******************************************************************
    double dTau = Globals::dTau;
    double k1 = Globals::k1;
    double k2 = Globals::k2;
    double k3 = Globals::k3;
    double k4 = Globals::k4;

    int nrHypos = hypos.getSize();

    //*******************************************************************
    // Build up an interaction matrix between hypotheses
    //*******************************************************************

    Q.set_size(nrHypos, nrHypos, 0.0);
    if(Globals::verbose){
        cout << "MDL hypothesis selection" << endl;
        cout << "   Building up interaction matrix..." << endl;
    }

    //*******************************************************************
    // Enter the diagonal terms => simple hypothesis scores
    //*******************************************************************

    double nw;
    double scoreW;
    double k_dur;
    Vector<double> vEnd;

    if (k4 < 0) {
        for (int i = 0; i < nrHypos; i++) {
            nw = hypos(i).getNW();
            scoreW = hypos(i).getScoreW();
            Q(i, i) = -k1 + ((1 - k2) * nw + k2 * scoreW);
        }
    } else {
        for (int i = 0; i < nrHypos; i++) {

            hypos(i).getEnd(vEnd);
            k_dur = exp(-((t - vEnd(3)) / k4));
            nw = hypos(i).getNW();
            scoreW = hypos(i).getScoreW();
            Q(i, i) = -k1 + k_dur * ((1 - k2) * nw + k2 * scoreW);
        }
    }

    //*******************************************************************
    // Enter the interaction terms for overlapping hypotheses
    //*******************************************************************

    Vector < FrameInlier > intersection;
    Vector < FrameInlier > idx1;
    Vector < FrameInlier > idx2;

    Matrix<double> bbox1;
    Matrix<double> bbox2;

    Vector<double> start1;
    Vector<double> start2;
    Vector<double> end1;
    Vector<double> end2;
    Matrix<double> startRect1;
    Matrix<double> startRect2;
    Matrix<double> endRect1;
    Matrix<double> endRect2;

    Matrix<double> mDi;
    Matrix<double> mDj;

    Matrix<double> mPi0;
    Matrix<double> mPj0;

    Matrix<double> mPi;
    Matrix<double> mPj;

    Vector<double> vXi;
    Vector<double> vYi;
    Vector<double> vXj;
    Vector<double> vYj;

    Vector<int> inlier;
    Vector<double> weights;

    int t0;
    double ar;
    double overlap3D = 0;
    double error = 0;
    double NW;
    double d1;
    double d2;
    double Sinter;

    map<int, int, greater<int> > trI;
    map<int, int, greater<int> > trJ;
    map<int, int>::iterator it;

    Vector<int> TrajTi;
    Vector< Matrix <double> > TrajRectI;
    Vector<int> TrajTj;
    Vector< Matrix <double> > TrajRectJ;

    int tLeni;
    int tLenj;

    for (int i = 0; i < nrHypos; i++) {
        for (int j = i + 1; j < nrHypos; j++) {
            if (hypos(i).getCategory() == hypos(j).getCategory()) {
                //+++++++++++++++++++++++++++++++++++++++++++++++++++++
                // compute the number of intersecting points
                //+++++++++++++++++++++++++++++++++++++++++++++++++++++
                intersection.clearContent();
                hypos(i).getIdx(idx1);
                hypos(j).getIdx(idx2);

                AncillaryMethods::intersectIdx(idx1, idx2, intersection);

            }
            overlap3D = 0.0;
            //+++++++++++++++++++++++++++++++++++++++++++++++++++++
            // check if the two hypo bboxes intersect
            //+++++++++++++++++++++++++++++++++++++++++++++++++++++

            hypos(i).getBBox4D(bbox1);
            hypos(j).getBBox4D(bbox2);
            hypos(i).getStart(start1);
            hypos(j).getStart(start2);
            hypos(i).getEnd(end1);
            hypos(j).getEnd(end2);

            if (checkBBoxOverlap(bbox1, bbox2) && (t - start1(3)) >= 0 && (t - start2(3)) >= 0)
            {
                tLeni = max(1, int (end1(3) - start1(3) - 1));
                tLenj = max(1, int (end2(3) - start2(3) - 1));

                hypos(i).getStartRect(startRect1);
                hypos(j).getStartRect(startRect2);
                hypos(i).getEndRect(endRect1);
                hypos(j).getEndRect(endRect2);

                mDi = endRect1;
                mDi -= startRect1;
                mDi *= (1.0 / double(tLeni));

                mDj = endRect2;
                mDj -= startRect2;
                mDj *= (1.0 /  double(tLenj));

                t0 = max(start1(3), start2(3));

                mPi0 = mDi;
                mPi0 *= (t0 - start1(3));
                mPi0 += startRect1;

                mPj0 = mDj;
                mPj0 *= (t0 - start2(3));
                mPj0 += startRect2;


                hypos(i).getTrajT(TrajTi);
                hypos(j).getTrajT(TrajTj);
                hypos(i).getTrajRect(TrajRectI);
                hypos(j).getTrajRect(TrajRectJ);

                trI.clear();
                trJ.clear();

                for ( int s = 0; s < TrajTi.getSize(); s++) {
                    trI[TrajTi(s)] = s;
                }

                for ( int s = 0; s < TrajTj.getSize(); s++) {
                    trJ[TrajTj(s)] = s;
                }

                for (int k = t0; k <= t; k++) {

                    it = trI.find(k);
                    if (it != trI.end()) {
                        mPi = TrajRectI((*it).second);
                    } else {
                        mPi = mDi;
                        mPi *= (k - t0);
                        mPi += mPi0;
                    }

                    it = trJ.find(k);
                    if (it != trJ.end()) {
                        mPj = TrajRectJ((*it).second);
                    } else {
                        mPj = mDj;
                        mPj *= (k - t0);
                        mPj += mPj0;
                    }

                    mPi.getColumn(0, vXi);
                    vXi.swap();
                    mPi.getColumn(1, vYi);
                    vYi.swap();
                    mPj.getColumn(0, vXj);
                    vXj.swap();
                    mPj.getColumn(1, vYj);
                    vYj.swap();


                    ar = Math::polyintersection(vXi, vYi, vXj, vYj);

                    overlap3D += ar * exp(-(double(t) - double(k)) / double(dTau));
                }
            }


            error = 0.0;
            NW = 0.0;

            for ( int k = 0; k < intersection.getSize(); k++) {
                inlier = intersection(k).getInlier();
                weights = intersection(k).getWeight();
                int fr = intersection(k).getFrame();

                for ( int l = 0; l < inlier.getSize(); l++) {
                    NW += normfct;
                    d1 = weights(l * 2);
                    d2 = weights(l * 2 + 1);

                    error += min(d1, d2)* exp(-(double(t) - double(fr)) / double(dTau));
                }
            }

            Sinter = (1.0 - k2) * NW + k2*error + k3*overlap3D;

            Q(i, j) = - Sinter / 2.0;
            Q(j, i) = Q(i, j);
        }
    }
}


bool MDL::checkInsideIoU(Vector<double>& bbox1, Vector<double>& bbox2)
{
    bool res = false;

    if((bbox1(0) >= bbox2(0) && bbox1(1) >= bbox2(1) && (bbox1(0)+bbox1(2)) <= (bbox2(0)+bbox2(2)) && (bbox1(1)+bbox1(3)) <= (bbox2(1)+bbox2(3))) ||
            (bbox2(0) >= bbox1(0) && bbox2(1) >= bbox1(1) && (bbox2(0)+bbox2(2)) <= (bbox1(0)+bbox1(2)) && (bbox2(1)+bbox2(3)) <= (bbox1(1)+bbox1(3))))
    {
        res = true;
        return res;
    }

    double bbox1surf = bbox1(2)*bbox1(3);
    double bbox2surf = bbox2(2)*bbox2(3);
    Vector<double> rect3;

    double overlapsurface = Math::intersetRectSurface(bbox1, bbox2, rect3);
    overlapsurface = overlapsurface / (bbox1surf + bbox2surf - overlapsurface);

    if(overlapsurface > 0.8) res = true;

    return res;
}

bool MDL::checkBBoxOverlap(Matrix<double>& bbox1, Matrix<double>& bbox2)
{
    double Rx1 = max(bbox1(0,0), bbox2(0,0));
    double Ry1 = max(bbox1(1,0), bbox2(1,0));
    double Rx2 = max(bbox1(0,1), bbox2(0,1));
    double Ry2 = max(bbox1(1,1), bbox2(1,1));

    if ((Rx2 >= Rx1) && (Ry2 >= Ry1))
    {
        return true;
    }
    return false;
}

double MDL::solve_mdl_exactly(Matrix<double>& Q, Vector<double>& m, Vector < Hypo >& HyposMDL, Vector<int>& HypoIdx, Vector < Hypo >& HypoAll)
{
    //**************************************************************
    // Q is  Cost matrix
    // m  Best indices
    // HyposMDL are the best selected Hypos
    // HyposAll are the hypos
    // HypoIdx are the indices of the selected Hypos in HyposAll
    //**************************************************************

    HyposMDL.clearContent();
    HypoIdx.clearContent();
    int ex_dim = Q.x_size(); // Dimensions of cost matrix

    m.setSize(ex_dim, 0.0);

    int STEP_SIZE = 6;

    int ex_steps[STEP_SIZE];
    double *ex_a; // Cost matrix
    Vector< int > ex_bestind; // Best indices
    double ex_bestscore; // Best score
    int ex_bestsize = 0;

    //*****************************************************************
    // set the number of potential search branches to a tractable value
    //*****************************************************************
    ex_steps[2] = 5;
    ex_steps[3] = 2;
    ex_steps[4] = ex_steps[5] = 1;
    if (ex_dim < 50) { // 50000 paths max
        ex_steps[0] = ex_steps[1] = 50;
    } else if (ex_dim < 70) { // 25000 paths max
        ex_steps[0] = 70;
        ex_steps[1] = 25;
    } else if (ex_dim < 100) { // 10000 paths max
        ex_steps[0] = 50;
        ex_steps[1] = 20;
    } else if (ex_dim < 250) { // 5000 paths max
        ex_steps[0] = 50;
        ex_steps[1] = 10;
    } else { // 2500 paths max
        ex_steps[0] = 50;
        ex_steps[1] = 5;
    }

    Vector< pair< double, int > > models(ex_dim);
    for (int i = 0; i < ex_dim; i++)
        models[i].second = i;

    ex_bestind.clearContent();
    ex_bestscore = 0;
    ex_a = Q.data();
    findmax(models, 0, 0, ex_steps, ex_a, ex_bestind, ex_bestscore, ex_bestsize, ex_dim, STEP_SIZE);

    for(int i = 0; i < ex_bestind.getSize(); i++)
    {
        m(ex_bestind(i)) = 1.0;
        HyposMDL.pushBack(HypoAll(ex_bestind(i)));
        HyposMDL(i).setScoreMDL(Q(ex_bestind(i), ex_bestind(i)));
    }

    HypoIdx = ex_bestind;

    return ex_bestscore;

}

void MDL::findmax(Vector< pair< double, int > >& models, int start, double score, int *ex_steps, double *ex_a, Vector< int >& ex_bestind, double &ex_bestscore, int ex_bestsize, int ex_dim, int STEP_SIZE)
{


    if (score > ex_bestscore) {
        ex_bestscore = score;
        ex_bestind.setSize(start);
        for (int i = 0; i < start; i++) {
            ex_bestind[i] = models[i].second;
        }
        ex_bestsize = ex_bestind.getSize();
    }
    if (start >= ex_dim)
        return;

    for (int i = start; i < ex_dim; i++) {
        // Calculate effect of this model
        int midx = models[i].second;
        double inc = 0;
        for (int j = 0; j < start; j++)
            inc += ex_a[ex_dim * midx + models[j].second];
        inc = 2 * inc + ex_a[ex_dim * midx + midx];
        models[i].first = inc;
    }

    // Sort remaining models according to their merit
    if (start < ex_dim - 1) {

        vector<pair< double, int > > modelsCopy = models.getStl();
        sort(modelsCopy.begin() + start, modelsCopy.end(), greater< pair< double, int > >());
        models.clearContent();

        for(unsigned int i = 0; i < modelsCopy.size(); i++)
            models.pushBack(modelsCopy.at(i));
    }

    // Try selecting remaining models
    int stepno = 1;
    if (start < STEP_SIZE)
        stepno = ex_steps[start];
    if (start + stepno > ex_dim)
        stepno = ex_dim - start;
    for (int i = start; i < start + stepno; i++) {
        if (models[i].first > 0) {
            // Follow this branch recursively
            double inc = models[i].first;
            int idx = models[i].second;
            swap(models[start], models[i]);

            Vector< pair< double, int > > modelsX = models;
            findmax(modelsX, start + 1, inc + score,  ex_steps,  ex_a, ex_bestind,  ex_bestscore, ex_bestsize, ex_dim, STEP_SIZE);

            for (int j = start + 1; j < ex_dim; j++)
                if (models[j].second == idx) {
                    swap(models[start], models[j]);
                    break;
                }
        } else
            break;
    }
}
