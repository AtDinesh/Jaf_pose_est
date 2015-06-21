#include "detector_seg.h"
#include "AncillaryMethods.h"
#include "KConnectedComponentLabeler.h"

Detector_Seg::Detector_Seg()
{
}

void Detector_Seg::GetROIs(const Camera &camera, const Matrix<double> &depth_map, Matrix<int>& occ_map_binary, const PointCloud &point_cloud)          //Added
{
    int width = depth_map.x_size();
    int height = depth_map.y_size();

    Matrix<int> mat_2D_pos_x(width, height, -1);
    Matrix<int> mat_2D_pos_y(width, height, -1);

    // Compute 2D_positions, and occ_Map matrices
    ComputeFreespace(camera, occ_map_binary, mat_2D_pos_x, mat_2D_pos_y, point_cloud, occ_map);

    Vector<ROI_SEG> all_ROIs;
    PreprocessROIs(occ_map_binary, all_ROIs);

    Vector<SegmentedObj> all_objs; //all objects in image
    SegmentationROI::RunSegmentation(all_objs, occ_map, occ_map_binary, all_ROIs, mat_2D_pos_x, mat_2D_pos_y, point_cloud.X, point_cloud.Y, point_cloud.Z);
    std::cout << "size all_objs vector : " << all_objs.getSize() << std::endl;
    std::cout << "size all_ROIs vector : " << all_ROIs.getSize() << std::endl;

}

void Detector_Seg::ProcessFrame(const Camera &camera, const Matrix<double> &depth_map, const PointCloud &point_cloud,
                            const Matrix<double> &upper_body_template, Vector<Vector<double> > &detected_bounding_boxes)
{
    int width = depth_map.x_size();
    int height = depth_map.y_size();

    Matrix<int> mat_2D_pos_x(width, height, -1);
    Matrix<int> mat_2D_pos_y(width, height, -1);

    // Compute 2D_positions, and occ_Map matrices
    ComputeFreespace(camera, labeledROIs, mat_2D_pos_x, mat_2D_pos_y, point_cloud, occ_map);


    Vector<ROI_SEG> all_ROIs;
    PreprocessROIs(labeledROIs, all_ROIs);

    Vector<SegmentedObj> all_objs;
    //SegmentationROI::RunSegmentation(all_objs, occ_map, labeledROIs, all_ROIs, mat_2D_pos_x, mat_2D_pos_y, point_cloud.X, point_cloud.Y, point_cloud.Z);
    SegmentationROI::RunSegmentation(all_objs,roiInHist_view,roi_reshaped, occ_map, labeledROIs, all_ROIs, mat_2D_pos_x, mat_2D_pos_y, point_cloud.X, point_cloud.Y, point_cloud.Z);

    Camera camera_origin = AncillaryMethods::GetCameraOrigin(camera);
    Vector<double> plane_in_camera = camera_origin.get_GP();

    Vector<Vector<double> > close_range_BBoxes;
    Vector<Vector<double > > distances;
    Vector<double> bb(4);
    Vector<double> distance(2);
    for(int i=0; i<all_objs.getSize(); ++i)
    {
        double height = AncillaryMethods::DistanceToPlane(all_objs(i).pos3DMinY, plane_in_camera);
        double mi = all_objs(i).pos3DMinZ(2), mx = all_objs(i).pos3DMaxZ(2);
        double cz = (mx+mi)/2, sz = mx-mi;

        if((cz-sz/2.0) < Globals::distance_range_accepted_detections && height < Globals::max_height && height > Globals::min_height)
        {
            if(all_objs(i).get2DBoundingBoxInImagePlane(bb,camera_origin))
            {
                close_range_BBoxes.pushBack(bb);

                distance(0) = cz;
                distance(1) = sz;
                distances.pushBack(distance);
            }
        }
    }

    detected_bounding_boxes = EvaluateTemplate(upper_body_template, depth_map, close_range_BBoxes, distances);

//    // Trial to change roi_image to show ground plane with detected upper bodies
//        //Upper_body positions shall be in  Vector<Vector<double> > max_pos;
//        Vector<double> pos2D;
//        Vector<double> pos3D(3);
//        for (nb_objects = 0;nb_objects < detected_bounding_boxes.getSize(); ++nb_objects)
//        {
//            pos2D.pushBack(detected_bounding_boxes(nb_objects)(0)+detected_bounding_boxes(nb_objects)(2)/2.0);
//            pos2D.pushBack(detected_bounding_boxes(nb_objects)(1)+detected_bounding_boxes(nb_objects)(3)/2.0);
//            pos2D.pushBack(1);
//
//
//        }
//    //

//    std::cout << "mat_2D_pos_x.x_size() : " << mat_2D_pos_x.y_size() << std::endl;
//    std::cout << "mat_2D_pos_x.y_size() : " << mat_2D_pos_x.y_size() << std::endl;
//    std::cout << "roi_image.x_size() : " << roi_image.x_size() << std::endl;
//    std::cout << "roi_image.y_size() : " << roi_image.y_size() << std::endl;
//
//    for (int i = 0; i < close_range_BBoxes.getSize(); ++i)
//    {
//    int cropped_height = (int)(close_range_BBoxes(i)(3)/2.0);
//    cropped_height += (close_range_BBoxes(i)(3) * Globals::evaluation_inc_height_ratio)/2.0;
//    int start_column = (int)close_range_BBoxes(i)(0);
//    int end_column = (int)(close_range_BBoxes(i)(0) + close_range_BBoxes(i)(2));
//    int start_row = (int)max(0.0, close_range_BBoxes(i)(1));
//    int end_row = (int)close_range_BBoxes(i)(1) + cropped_height;
//                //////////////// just for test (must be removed)
//    if(visualize_roi)
//    for(int tmpx=start_column, tmpxx=0; tmpxx<mat_2D_pos_x.x_size(); ++tmpx,++tmpxx)
//    {
//        for(int tmpy=start_row, tmpyy=0; tmpyy<mat_2D_pos_x.y_size(); ++tmpy,++tmpyy)
//        {
//            if(tmpyy==0 || tmpyy==mat_2D_pos_x.y_size()-1 || tmpxx==0 || tmpxx==mat_2D_pos_x.x_size()-1) //if on the border of the image
//                roi_image(tmpx,tmpy)=i+1;
//
//            if(mat_2D_pos_x(tmpxx,tmpyy)!=0)
//                roi_image(tmpx,tmpy)=i+1;
//        }
//    }
//    }


}

void Detector_Seg::ComputeFreespace(const Camera& camera,
                     Matrix<int>& occ_map_binary,
                     Matrix<int>& mat_2D_pos_x, Matrix<int>& mat_2D_pos_y,
                     const PointCloud &point_cloud,
                                    Matrix<double>& occ_map)
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

    occ_map.set_size(x_bins, z_bins, 0.0);
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
        if(occ_map.data()[i] < Globals::freespace_threshold)
        {
            occ_map_binary.data()[i] = 0;
            occ_map.data()[i] = 0;
        }
        else
            occ_map_binary.data()[i] = 1;
    }
}

void Detector_Seg::PreprocessROIs(Matrix<int> &labeled_ROIs, Vector<ROI_SEG> &all_ROIs)
{
    KConnectedComponentLabeler ccl(Globals::region_size_threshold, labeled_ROIs.data(), labeled_ROIs.x_size(), labeled_ROIs.y_size());

    ccl.Process(); // Main function

    ROI_SEG::ExtractROIs(all_ROIs, labeled_ROIs, ccl.m_ObjectNumber);
}

void Detector_Seg::ExtractPointsInROIs(Vector<Vector<Vector<double> > > &all_points_in_ROIs, const Matrix<int> &mat_2D_pos_x, const Matrix<int> &mat_2D_pos_y, const PointCloud& point_cloud, const Matrix<int> &labeled_ROIs)
{
    int number_of_ROIs = all_points_in_ROIs.getSize();

    for(int i = 0; i < number_of_ROIs; i++)
    {
        all_points_in_ROIs(i).reserveCapacity(50000);
    }

    int pos_in_proj;
    int mat_size = mat_2D_pos_x.x_size()*mat_2D_pos_x.y_size();
    for(int i = 0; i < mat_size; i++)
    {
        if(mat_2D_pos_x.data()[i] >= 0)
        {
            pos_in_proj = labeled_ROIs(mat_2D_pos_x.data()[i], mat_2D_pos_y.data()[i])-1;
            if(pos_in_proj > -1)
            {
                all_points_in_ROIs(pos_in_proj).pushBack(Vector<double>(point_cloud.X(i), point_cloud.Y(i), point_cloud.Z(i)));
            }
        }
    }
}
