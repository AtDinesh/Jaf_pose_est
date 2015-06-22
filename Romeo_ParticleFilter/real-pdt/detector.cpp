#include "detector.h"
#include "KConnectedComponentLabeler.h"

Detector::Detector()
{
}

void Detector::ProcessFrame(const Camera &camera, const Matrix<double> &depth_map, const PointCloud &point_cloud,
                            const Matrix<double> &upper_body_template, Vector<Vector<double> > &detected_bounding_boxes)
{
    int width = depth_map.x_size();
    int height = depth_map.y_size();

    //    Matrix<int> labeledROIs;
    Matrix<int> mat_2D_pos_x(width, height, -1);
    Matrix<int> mat_2D_pos_y(width, height, -1);

    // Compute 2D_positions, and occ_Map matrices
    ComputeFreespace(camera, labeledROIs, mat_2D_pos_x, mat_2D_pos_y, point_cloud);


    Vector<ROI> all_ROIs;
    PreprocessROIs(labeledROIs, all_ROIs);

    ExtractPointsInROIs(/*points3D_in_ROIs*/all_ROIs, mat_2D_pos_x, mat_2D_pos_y, point_cloud, labeledROIs);

    double scaleZ = Globals::freespace_scaleZ;

    Camera camera_origin = AncillaryMethods::GetCameraOrigin(camera);
    Vector<double> plane_in_camera = camera_origin.get_GP();


    Vector<Vector<double> > close_range_BBoxes;
    Vector<Vector<double > > distances;

    Vector<double> lower_point(3);

    for(int l = 0; l < all_ROIs.getSize(); l++)
    {
        if(all_ROIs(l).has_any_point)
        {
            lower_point(0) = point_cloud.X(all_ROIs(l).ind_min_y);
            lower_point(1) = point_cloud.Y(all_ROIs(l).ind_min_y);
            lower_point(2) = point_cloud.Z(all_ROIs(l).ind_min_y);

            double height = AncillaryMethods::DistanceToPlane(lower_point, plane_in_camera);

            if((all_ROIs(l).center_y()-all_ROIs(l).width()/2.0)/scaleZ < Globals::distance_range_accepted_detections
                    && height < Globals::max_height && height > Globals::min_height)
            {
                Vector<double> bbox(4, 0.0);
                if(all_ROIs(l).GetRegion2D(bbox, camera_origin, point_cloud) )
                {
                    close_range_BBoxes.pushBack(bbox);

                    Vector<double> distance(2);
                    distance(0) = all_ROIs(l).center_y()/scaleZ;
                    distance(1) = all_ROIs(l).height()/scaleZ;

                    distances.pushBack(distance);
                }
            }
        }
    }

    detected_bounding_boxes = EvaluateTemplate(upper_body_template, depth_map, close_range_BBoxes, distances);
}

void Detector::ComputeFreespace(const Camera& camera,
                                Matrix<int>& occ_map_binary,
                                Matrix<int>& mat_2D_pos_x, Matrix<int>& mat_2D_pos_y,
                                const PointCloud &point_cloud)
{
    // Set Freespace Parameters
    double scale_z_ = Globals::freespace_scaleZ;
    double scale_x_ = Globals::freespace_scaleX;
    double min_x_ = Globals::freespace_minX;
    double min_z_ = Globals::freespace_minZ;
    double max_x_ = Globals::freespace_maxX;
    double max_z_ = Globals::freespace_maxZ;

    int x_bins = (int)round((max_x_ - min_x_)*scale_x_)+1;
    int z_bins = (int)round((max_z_ - min_z_)*scale_z_)+1;

    Matrix<double> occ_map(x_bins, z_bins, 0.0);
    occ_map_binary.set_size(x_bins, z_bins);
    double step_x = (max_x_ - min_x_)/double(x_bins-1);
    double step_z = (max_z_ - min_z_)/double(z_bins-1);

    Vector<double> plane_parameters = AncillaryMethods::PlaneToCam(camera);
    double plane_par_0 = plane_parameters(0);
    double plane_par_1 = plane_parameters(1);
    double plane_par_2 = plane_parameters(2);
    double plane_par_3 = plane_parameters(3)*Globals::WORLD_SCALE;

    double log_2 = log(2);

    for(int j = 0; j < point_cloud.X.getSize(); j++)
    {
        double zj = point_cloud.Z(j);

        if(zj < Globals::freespace_max_depth_to_cons && zj >= 0.1)
        {
            double xj = point_cloud.X(j);
            double yj = point_cloud.Y(j);

            double distance_to_plane = plane_par_0*xj + plane_par_1*yj + plane_par_2*zj + plane_par_3;

            // Only accept points in upper_body height range (0.1m to 2.5m)
            if(distance_to_plane < Globals::freespace_max_height && distance_to_plane > 0.1)
            {
                double x = xj - min_x_;
                double z = zj - min_z_;

                int pos_x = (int)round(x / step_x);
                int pos_z = (int)round(z / step_z);

                if(Globals::log_weight)
                    occ_map(pos_x, pos_z) += z*(log(z) / log_2);
                else
                    occ_map(pos_x, pos_z) += z;

                mat_2D_pos_x.data()[j] = pos_x;
                mat_2D_pos_y.data()[j] = pos_z;
            }
        }
    }


    int occ_map_length = x_bins*z_bins;
    for(int i = 0; i < occ_map_length; i++)
    {
        occ_map_binary.data()[i] = (occ_map.data()[i] < Globals::freespace_threshold) ? 0 : 1;
    }
}

void Detector::PreprocessROIs(Matrix<int> &labeled_ROIs, Vector<ROI> &all_ROIs)
{
    KConnectedComponentLabeler ccl(Globals::region_size_threshold, labeled_ROIs.data(), labeled_ROIs.x_size(), labeled_ROIs.y_size());

    ccl.Process(); // Main function

    ROI::ExtractROIs(all_ROIs, labeled_ROIs, ccl.m_ObjectNumber);
}

struct ROIBound
{
    ROIBound()
    {
        min_x = min_y = min_z = 1e10;
        max_x = max_y = max_z = -1e10;
    }

    double min_x, max_x;
    double min_y, max_y;
    double min_z, max_z;
};

void Detector::ExtractPointsInROIs(Vector<ROI> &all_ROIs, const Matrix<int> &mat_2D_pos_x, const Matrix<int> &mat_2D_pos_y, const PointCloud& point_cloud, const Matrix<int> &labeled_ROIs)
{
    int number_of_ROIs = all_ROIs.getSize();

    Vector<ROIBound> min_maxs(number_of_ROIs);
    ROIBound* roi_bound;
    int roi_ind;
    int mat_size = mat_2D_pos_x.x_size()*mat_2D_pos_x.y_size();
    for(int i = 0; i < mat_size; i++)
    {
        if(mat_2D_pos_x.data()[i] >= 0)
        {
            roi_ind = labeled_ROIs(mat_2D_pos_x.data()[i], mat_2D_pos_y.data()[i])-1;
            if(roi_ind > -1)
            {
                all_ROIs(roi_ind).has_any_point = true;
                double x = point_cloud.X(i), y = point_cloud.Y(i), z = point_cloud.Z(i);
                roi_bound = &min_maxs(roi_ind);
                if(roi_bound->min_x > x)
                {
                    roi_bound->min_x = x;
                    all_ROIs(roi_ind).ind_min_x = i;
                }
                if(roi_bound->max_x < x)
                {
                    roi_bound->max_x = x;
                    all_ROIs(roi_ind).ind_max_x = i;
                }
                if(roi_bound->min_y > y)
                {
                    roi_bound->min_y = y;
                    all_ROIs(roi_ind).ind_min_y = i;
                }
                if(roi_bound->max_y < y)
                {
                    roi_bound->max_y = y;
                    all_ROIs(roi_ind).ind_max_y = i;
                }
                if(roi_bound->min_z > z)
                {
                    roi_bound->min_z = z;
                    all_ROIs(roi_ind).ind_min_z = i;
                }
                if(roi_bound->max_z < z)
                {
                    roi_bound->max_z = z;
                    all_ROIs(roi_ind).ind_max_z = i;
                }
            }
        }
    }
}
