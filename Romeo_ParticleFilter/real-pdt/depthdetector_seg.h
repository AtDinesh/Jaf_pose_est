#ifndef DEPTHDETECTOR_SEG_H
#define DEPTHDETECTOR_SEG_H

#include "detector_seg.h"

class DepthDetector_Seg : public Detector_Seg
{
public:
    DepthDetector_Seg();

protected:
    virtual Vector<Vector<double> > EvaluateTemplate(const Matrix<double> &upper_body_template, const Matrix<double> &depth_map,
                                             Vector<Vector<double> > &close_range_BBoxes, Vector<Vector<double> > distances);
};

#endif // DEPTHDETECTOR_SEG_H
