#include "ROI_SEG.h"
#include "camera/Camera.h"
#include "Globals.h"
#include "AncillaryMethods.h"

ROI_SEG::ROI_SEG() :
    d_min_X_pos(1e5), d_max_X_pos(-1e5), d_min_Z_pos(1e5), d_max_Z_pos(-1e5)
{
}

void ROI_SEG::ExtractROIs(Vector<ROI_SEG>& all_ROIs, const Matrix<int>& labeled_ROIs, int number_of_ROIs)
{
    int z_size = labeled_ROIs.y_size();

    all_ROIs.setSize(number_of_ROIs);
    int ind;

    for(int x = 0; x < labeled_ROIs.x_size(); x++)
    {
        for(int z = 0; z < z_size; z++)
        {
            if(labeled_ROIs(x,z) > 0)
            {
                ind = labeled_ROIs(x,z)-1;
                if(all_ROIs(ind).d_min_X_pos > x) all_ROIs(ind).d_min_X_pos = x;
                if(all_ROIs(ind).d_min_Z_pos > z) all_ROIs(ind).d_min_Z_pos = z;
                if(all_ROIs(ind).d_max_X_pos < x) all_ROIs(ind).d_max_X_pos = x;
                if(all_ROIs(ind).d_max_Z_pos < z) all_ROIs(ind).d_max_Z_pos = z;
            }
        }
    }
}

ROI_SEG& ROI_SEG::operator=(const ROI_SEG &roi)
{
    if (this == &roi)
        return *this;
    m_region = roi.m_region;

    d_min_X_pos = roi.d_min_X_pos;
    d_max_X_pos= roi.d_max_X_pos;
    d_min_Z_pos= roi.d_min_Z_pos;
    d_max_Z_pos= roi.d_max_Z_pos;

    return *this;
}

ROI_SEG::ROI_SEG(const ROI_SEG& roi)
{
    m_region = roi.m_region;
    d_min_X_pos = roi.d_min_X_pos;
    d_max_X_pos= roi.d_max_X_pos;
    d_min_Z_pos= roi.d_min_Z_pos;
    d_max_Z_pos= roi.d_max_Z_pos;
}
