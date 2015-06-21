#ifndef DEPTHDETECTOR_H
#define DEPTHDETECTOR_H

#include "detector.h"

class DepthDetector : public Detector
{
public:
    DepthDetector();
protected:
    virtual Vector<Vector<double> > EvaluateTemplate(const Matrix<double> &upper_body_template, const Matrix<double> &depth_map,
                                                     Vector<Vector<double> > &close_range_BBoxes, Vector<Vector<double> > distances);
};

#endif // DEPTHDETECTOR_H
