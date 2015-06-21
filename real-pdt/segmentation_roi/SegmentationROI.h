#ifndef SegmentationROI_H
#define SegmentationROI_H


#include "SegmentedObj.h"
#include "ROI_SEG.h"

class SegmentationROI
{
public:
    SegmentationROI();

    static void RunSegmentation(Vector<SegmentedObj> &allObj, const Matrix<double>& histogram, Matrix<int>& labeled_hist, Vector<ROI_SEG> &rois, const Matrix<int> &mat_2D_pos_x, const Matrix<int> &mat_2D_pos_y, const Vector<double> &mat_3D_points_x, const Vector<double> &mat_3D_points_y, const Vector<double> &mat_3D_points_z);
};

#endif // SegmentationROI_H
