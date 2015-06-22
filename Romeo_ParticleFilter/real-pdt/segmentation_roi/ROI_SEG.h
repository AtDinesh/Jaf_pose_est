#ifndef ROI_SEG_H
#define ROI_SEG_H

#include "Matrix.h"
#include "camera/Camera.h"
#include "QImage"

class ROI_SEG
{

public:
    ROI_SEG();
    ROI_SEG(const ROI_SEG& copy);
    ROI_SEG(int min_x_pos, int max_x_pos, int min_z_pos, int max_z_pos) :
        d_min_X_pos(min_x_pos), d_max_X_pos(max_x_pos), d_min_Z_pos(min_z_pos), d_max_Z_pos(max_z_pos)
    {}

    static void ExtractROIs(Vector<ROI_SEG>& all_ROIs, const Matrix<int> &labeled_ROIs, int number_of_ROIs);

    ROI_SEG& operator=(const ROI_SEG &roi);

    int get_d_min_X_pos(){return d_min_X_pos;}
    int get_d_min_Z_pos(){return d_min_Z_pos;}
    int get_d_max_X_pos(){return d_max_X_pos;}
    int get_d_max_Z_pos(){return d_max_Z_pos;}

protected:
    Vector< Vector <double> >  m_region;

    int d_min_X_pos;
    int d_max_X_pos;
    int d_min_Z_pos;
    int d_max_Z_pos;

};

#endif // ROI_SEG_H
