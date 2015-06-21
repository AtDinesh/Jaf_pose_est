#ifndef DEPTHDETECTOR_LM_H
#define DEPTHDETECTOR_LM_H

#include "detector.h"

class DepthDetector_LM : public Detector
{
public:
    DepthDetector_LM();

protected:
    virtual Vector<Vector<double> > EvaluateTemplate(const Matrix<double> &upper_body_template, const Matrix<double> &depth_map,
                                                     Vector<Vector<double> > &close_range_BBoxes, Vector<Vector<double> > distances);
};

#endif // DEPTHDETECTOR_LM_H
