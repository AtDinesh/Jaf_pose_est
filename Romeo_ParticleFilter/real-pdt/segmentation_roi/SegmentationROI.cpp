#include "SegmentationROI.h"
#include "AncillaryMethods.h"
#include "CFilter.h"

using namespace NFilter;

SegmentationROI::SegmentationROI()
{
}

struct ObjectData
{
    ObjectData():min_x(1e10), max_x(-1e10), min_y(1e10), max_y(-1e10), min_z(1e10), max_z(-1e10) {}
    int number_of_points;
    int min_x_ind, max_x_ind;
    int min_y_ind, max_y_ind;
    int min_z_ind, max_z_ind;
    double min_x, max_x;
    double min_y, max_y;
    double min_z, max_z;
};

void SegmentationROI::RunSegmentation(Vector<SegmentedObj> &allObj,const Matrix<double>& histogram, Matrix<int>& labeled_hist, Vector<ROI_SEG> &rois,
                                      const Matrix<int>& mat_2D_pos_x, const Matrix<int>& mat_2D_pos_y,
                                      const Vector<double>& mat_3D_points_x, const Vector<double>& mat_3D_points_y, const Vector<double>& mat_3D_points_z)
{
    double sigmaX = Globals::sigmaX;
    double precisionX = Globals::precisionX;
    double sigmaZ = Globals::sigmaZ;
    double precisionZ = Globals::precisionZ;

    // Go over all ROIs and try to segment them in individual objects using quick shift.

    Vector<Matrix<int> > roi_reshaped_eqt(rois.getSize());
    Vector<Vector<int> > roi_labels(rois.getSize()), roi_ll(rois.getSize());
    Vector<Vector<ObjectData> > objects_data(rois.getSize());

    Vector<double> kernel_x = AncillaryMethods::getGaussian1D(sigmaX, precisionX);
    Vector<double> kernel_z = AncillaryMethods::getGaussian1D(sigmaZ, precisionZ);

    for(int i = 0; i < rois.getSize(); ++i)
    {
        Matrix<double> m_roisInHist(histogram, rois(i).get_d_min_X_pos(), rois(i).get_d_max_X_pos(),
                                    rois(i).get_d_min_Z_pos(), rois(i).get_d_max_Z_pos());

        // Smooth Gaussian (Observation the smoothing in z requires a larger sigma)
        m_roisInHist = AncillaryMethods::conv1D(m_roisInHist, kernel_x, true);
        m_roisInHist = AncillaryMethods::conv1D(m_roisInHist, kernel_z, false);

        // Generate the equivalence table
        Vector<int> eqt(m_roisInHist.x_size()*m_roisInHist.y_size(), 0.0);

        int xMaxPos = -1;
        int yMaxPos = -1;
        for(int j = 0; j < m_roisInHist.x_size(); ++j)
        {
            for(int k = 0; k < m_roisInHist.y_size(); ++k)
            {
                Matrix<double> mm(m_roisInHist, max(0, j-1), min(j+1, m_roisInHist.x_size()-1), max(0, k-1), min(k+1, m_roisInHist.y_size()-1));
                maxOfMatrix(mm, xMaxPos, yMaxPos);
                if(j > 0){ xMaxPos -= 1;}
                if(k > 0){ yMaxPos -= 1;}
                eqt((k)*m_roisInHist.x_size() + j) = (k + yMaxPos)*m_roisInHist.x_size() + j+xMaxPos ;
            }
        }

        // Connected components
        for(int j = 0; j < eqt.getSize(); ++j)
        {
            while (eqt(j) != eqt(eqt(j)))
            {
                eqt(j) = eqt(eqt(j));
            }
        }

        // Reshape eqt to form a matrix
        roi_reshaped_eqt(i) = Matrix<int>(m_roisInHist.x_size(), m_roisInHist.y_size(), eqt.data());
        roi_labels(i) = Vector<int>(roi_reshaped_eqt(i).x_size()*roi_reshaped_eqt(i).y_size(), roi_reshaped_eqt(i).data());
        roi_labels(i).make_unique();
        pair<int, int> mp = roi_labels(i).maxim();
        roi_ll(i) = Vector<int>(mp.first+1,-1);
        for(int ii=0;ii<roi_labels(i).getSize();++ii)
            roi_ll(i)(roi_labels(i)(ii))=ii;

        objects_data(i).setSize(roi_labels(i).getSize());
    }

    int sub_roi_ind;
    double x,y,z;
    double x2d, y2d;
    int sz = mat_2D_pos_x.total_size();
    for(int j=0; j<sz; ++j)
    {
        x2d = mat_2D_pos_x.data()[j];
        if(x2d>=0)
        {
            y2d = mat_2D_pos_y.data()[j];
            int i = labeled_hist(x2d, y2d)-1;
            if(i>-1)
            {
                sub_roi_ind = roi_ll(i)(roi_reshaped_eqt(i)(x2d-rois(i).get_d_min_X_pos(), y2d-rois(i).get_d_min_Z_pos()));
                if(sub_roi_ind >= 0)
                {
                    ObjectData* obj_data = &(objects_data(i)(sub_roi_ind));
                    ++(obj_data->number_of_points);

                    x = mat_3D_points_x(j);
                    y = mat_3D_points_y(j);
                    z = mat_3D_points_z(j);

                    if(obj_data->min_x > x)
                    {
                        obj_data->min_x_ind = j;
                        obj_data->min_x = x;
                    }
                    if(obj_data->min_y > y)
                    {
                        obj_data->min_y_ind = j;
                        obj_data->min_y = y;
                    }
                    if(obj_data->min_z > z)
                    {
                        obj_data->min_z_ind = j;
                        obj_data->min_z = z;
                    }
                    if(obj_data->max_x < x)
                    {
                        obj_data->max_x_ind = j;
                        obj_data->max_x = x;
                    }
                    if(obj_data->max_y < y)
                    {
                        obj_data->max_y_ind = j;
                        obj_data->max_y = y;
                    }
                    if(obj_data->max_z < z)
                    {
                        obj_data->max_z_ind = j;
                        obj_data->max_z = z;
                    }
                }
            }
        }
    }

    Vector<Vector<int> > indices(rois.getSize());
    int allObj_size = 0;
    for(int i=0; i<rois.getSize(); ++i)
    {
        for(int j=0; j<roi_labels(i).getSize(); ++j)
        {
            if(objects_data(i)(j).number_of_points>=300)
            {
                indices(i).pushBack(j);
                ++allObj_size;
            }
        }
    }


    allObj.setSize(allObj_size);
    int cnt_obj = 0;

    Vector<double>  aux(3);
    SegmentedObj* obj_ptr;
    int ind;
    for(int i = 0; i < indices.getSize(); ++i)
    {
        for(int j = 0; j < indices(i).getSize(); ++j)
        {
            int ii = indices(i)(j);
            obj_ptr = &allObj(cnt_obj++);

            ind = objects_data(i)(ii).min_x_ind;
            aux(0)  = mat_3D_points_x(ind);
            aux(1)  = mat_3D_points_y(ind);
            aux(2)  = mat_3D_points_z(ind);
            obj_ptr->pos3DMinX = aux;
            ind = objects_data(i)(ii).min_y_ind;
            aux(0)  = mat_3D_points_x(ind);
            aux(1)  = mat_3D_points_y(ind);
            aux(2)  = mat_3D_points_z(ind);
            obj_ptr->pos3DMinY = aux;
            ind = objects_data(i)(ii).min_z_ind;
            aux(0)  = mat_3D_points_x(ind);
            aux(1)  = mat_3D_points_y(ind);
            aux(2)  = mat_3D_points_z(ind);
            obj_ptr->pos3DMinZ = aux;

            ind = objects_data(i)(ii).max_x_ind;
            aux(0)  = mat_3D_points_x(ind);
            aux(1)  = mat_3D_points_y(ind);
            aux(2)  = mat_3D_points_z(ind);
            obj_ptr->pos3DMaxX = aux;
            ind = objects_data(i)(ii).max_y_ind;
            aux(0)  = mat_3D_points_x(ind);
            aux(1)  = mat_3D_points_y(ind);
            aux(2)  = mat_3D_points_z(ind);
            obj_ptr->pos3DMaxY = aux;
            ind = objects_data(i)(ii).max_z_ind;
            aux(0)  = mat_3D_points_x(ind);
            aux(1)  = mat_3D_points_y(ind);
            aux(2)  = mat_3D_points_z(ind);
            obj_ptr->pos3DMaxZ = aux;

        }
    }
}
