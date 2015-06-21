#ifndef DETECTOR_SEG_H
#define DETECTOR_SEG_H

#include "Vector.h"
#include "Matrix.h"
#include "camera/Camera.h"
#include "segmentation_roi/ROI_SEG.h"
#include "segmentation_roi/SegmentedObj.h"
#include "segmentation_roi/SegmentationROI.h"
#include "pointcloud.h"

class Detector_Seg
{
public:
    Detector_Seg();

    bool visualize_roi;
    Matrix<int> roi_image;
    Matrix<int> labeledROIs;
    Matrix<double> occ_map;
    ////////////////////////////////////////////////////////////////////////
    // ProcessFrame:
    //      Processes a single frame and detects all upper bodies in a frame.
    //
    // prarameters:
    //      input:
    //          camera              -   Camera settings related to the frame.
    //          depth_map           -   Depth map image of the frame.
    //          upper_body_template -   Template of upper body.
    //      output:
    //          detected_bounding_boxes -   Resulting bounding boxes around
    //                                      detected upper bodies.
    ////////////////////////////////////////////////////////////////////////
    void ProcessFrame(const Camera& camera, const Matrix<double>& depth_map, const PointCloud& point_cloud,
                      const Matrix<double>& upper_body_template, Vector<Vector<double> >& detected_bounding_boxes);

protected:
    ///////////////////////////////////////////////////////////////////////
    // EvaluateTemplate:
    //      Slides template of upper body over Depth-Map only over
    //      Close-Range-Bounding-Boxes and returns bounding boxes around
    //      detected upperbodies.
    //
    // parameters:
    //      input:
    //          upper_body_template -   template of upper body
    //          depth_map           -   Depth map image
    //          close_range_BBoxes  -   Bounding boxes over ROIs
    //          distances           -
    ///////////////////////////////////////////////////////////////////////
    virtual Vector<Vector<double> > EvaluateTemplate(const Matrix<double> &upper_body_template, const Matrix<double> &depth_map,
                                             Vector<Vector<double> > &close_range_BBoxes, Vector<Vector<double> > distances) = 0;
private:
    void ComputeFreespace(const Camera &camera, Matrix<int> &occ_map_binary,
                          Matrix<int> &mat_2D_pos_x, Matrix<int> &mat_2D_pos_y, const PointCloud& point_cloud, Matrix<double> &occ_map);

    ///////////////////////////////////////////////////////////////////////
    // PreprocessROIs:
    //
    // parameters:
    //      input:
    //          labeled_ROIs    -   binary occ_map
    //      output:
    //          labeled_ROIs    -   Labeled ROIs
    //          all_ROIs        -   All ROIs
    ///////////////////////////////////////////////////////////////////////
    void PreprocessROIs(Matrix<int>& labeled_ROIs, Vector<ROI_SEG>& all_ROIs);

    ///////////////////////////////////////////////////////////////////////
    // ExtractPointsInROIs:
    //      Extracts all 2D points inside ROIs.
    //
    // parameters:
    //      input:
    //          mat_2D_pos_x    -
    //          mat_2D_pos_y    -
    //          mat_3D_points_x -
    //          mat_3D_points_y -
    //          mat_3D_points_z -
    //          labeled_ROIs    -
    //      output:
    //          all_points_in_ROIs  -
    ///////////////////////////////////////////////////////////////////////
    void ExtractPointsInROIs(Vector< Vector< Vector<double> > >& all_points_in_ROIs,
                             const Matrix<int>& mat_2D_pos_x, const Matrix<int>& mat_2D_pos_y,
                             const PointCloud& point_cloud,
                             const Matrix<int>& labeled_ROIs);

};

#endif // DETECTOR_SEG_H
