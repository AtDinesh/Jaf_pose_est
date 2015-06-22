#ifndef SegmentedObj_H
#define SegmentedObj_H

#include "Matrix.h"
#include "camera/Camera.h"
#include "Volume.h"
#include "QImage"

class SegmentedObj
{

public:

    SegmentedObj();

     bool get2DBoundingBoxInImagePlane(Vector<double>& bbox, const Camera& camera);

     Vector<double> pos3DMinX;
     Vector<double> pos3DMaxX;
     Vector<double> pos3DMinY;
     Vector<double> pos3DMaxY;
     Vector<double> pos3DMinZ;
     Vector<double> pos3DMaxZ;
};

#endif // SegmentedObj
