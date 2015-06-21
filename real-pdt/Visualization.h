#ifndef _VISUALIZATION_H
#define	_VISUALIZATION_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <map>
#include "camera/Camera.h"
#include <algorithm>
#include "math/Math.h"
#include "Globals.h"

#include <CImg/CImg.h>
using namespace cimg_library;

using namespace std;


class Visualization
{
public:
    Visualization();
    ~Visualization();


    void render_hypos(Camera cam, int frame,  map<int, int, greater<int> >& assignedBBoxCol,
                      map<int, int, greater<int> >& hypoLastSelForVis, Matrix<unsigned char> &colors, CImg<unsigned char>& image, double speed,
                      double objMinVel, double objWidth, double objLength, double objHeight, int hypoID, Vector<Vector<double> > vvHypoTrajPts,
                      double WORLD_SCALE, Vector<double> vHypoX, Vector<double> vHypoDir, Vector<unsigned char> &currHypoColor);
    void render_bbox_2D(Vector<double> bbox, CImg<unsigned char>& image, int r, int g, int b, int lineWidth);
    void render_lines(Camera cam, Vector<double>& p1, Vector<double>& p2, Vector<unsigned char> &color, CImg<unsigned char>& image, double WORLD_SCALE);

    void smoothTraj(Vector<Vector<double> >& TrajPts, int nSmoothSize);


private:
    void setHypoBBoxColor(Vector<unsigned char> &currHypoColor,  map<int, int, greater<int> >& assignedBBoxCol, Matrix<unsigned char> &colors,
                          map<int, int, greater<int> >& hypoLastSelForVis, int t, int hypoID);
};



#endif
