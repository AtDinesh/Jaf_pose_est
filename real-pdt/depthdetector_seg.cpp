#include "depthdetector_seg.h"
#include "AncillaryMethods.h"


DepthDetector_Seg::DepthDetector_Seg()
{
}

Vector<Vector<double> > DepthDetector_Seg::EvaluateTemplate(const Matrix<double> &upper_body_template,
                                                            const Matrix<double> &depth_map,
                                                            Vector<Vector<double> > &close_range_BBoxes, Vector<Vector<double> > distances)
{
    int stride = Globals::evaluation_stride;
    int nr_scales = Globals::evaluation_nr_scales;
    int inc_cropped_height = Globals::evaluation_inc_cropped_height;

    // performance helper variables: just for avoiding recalculation
    int int_half_template_size = Globals::template_size / 2;
    double double_half_template_size = Globals::template_size / 2.0;

    Vector<Vector<double> > final_result;

    // generate the scales
    Vector<double> all_scales(nr_scales, 1.0);
    all_scales(0) = 1;
    for(int sc = 1; sc < nr_scales; ++sc)
    {
        all_scales(sc) = pow(Globals::evaluation_scale_stride,sc);
    }

    //////////////////////////////////////////////
    //    Matrix<double> roi_image(Globals::dImWidth, Globals::dImHeight, 0.0);
    if(visualize_roi)
        roi_image = Matrix<int>(Globals::dImWidth, Globals::dImHeight, 0);
    //////////////////////////////////////////////
    // CHECK EVERY CLOSE_RANGE_BBOXES
    for (int i = 0; i < close_range_BBoxes.getSize(); ++i)
    {
        Vector<Vector<double> > result;

        int cropped_height = (int)(close_range_BBoxes(i)(3)/2.0);
        cropped_height += (close_range_BBoxes(i)(3) * Globals::evaluation_inc_height_ratio)/2.0;
        close_range_BBoxes(i)(1) -= (close_range_BBoxes(i)(3) * Globals::evaluation_inc_height_ratio)/2.0;

        if( close_range_BBoxes(i)(1)+cropped_height >= Globals::dImHeight)
            cropped_height = Globals::dImHeight - (int)close_range_BBoxes(i)(1) - 1;

        if(Globals::verbose)
            cout << "(distances(i) " << distances(i)(0) << " radius " << distances(i)(1)/2.0 << endl;

        // Cropped and Filter depth_map with respect to distance from camera
        int start_column = (int)close_range_BBoxes(i)(0);
        int end_column = (int)(close_range_BBoxes(i)(0) + close_range_BBoxes(i)(2));
        int start_row = (int)max(0.0, close_range_BBoxes(i)(1));
        int end_row = (int)close_range_BBoxes(i)(1) + cropped_height;

        Matrix<double> cropped(end_column-start_column+1, end_row-start_row+1);
        // Set thrtesholds for depth_map
        double min_distance_threshold = distances(i)(0)- (distances(i)(1)+0.2)/2.0;
        double max_distance_threshold = distances(i)(0)+ (distances(i)(1)+0.2)/2.0;
        double d_val;
        for(int ii = 0, ii_depth = start_column; ii < cropped.x_size(); ++ii, ++ii_depth) //Extract points satisfying distance conditions
        {
            for(int jj = 0, jj_depth = start_row; jj < cropped.y_size(); ++jj, ++jj_depth)
            {
                d_val = depth_map(ii_depth,jj_depth);
                if(d_val <= min_distance_threshold || d_val >= max_distance_threshold)
                {
                    cropped(ii, jj) = 0;
                }
                else
                {
                    cropped(ii, jj) = d_val;
                }
            }
        }


        //////////////// just for test (must be removed)
//        if(visualize_roi)
//            for(int tmpx=start_column, tmpxx=0; tmpxx<cropped.x_size(); ++tmpx,++tmpxx)
//            {
//                for(int tmpy=start_row, tmpyy=0; tmpyy<cropped.y_size(); ++tmpy,++tmpyy)
//                {
//                    if(tmpyy==0 || tmpyy==cropped.y_size()-1 || tmpxx==0 || tmpxx==cropped.x_size()-1) //if on the border of the image
//                        roi_image(tmpx,tmpy)=i+1;
//
//                    if(cropped(tmpxx,tmpyy)!=0)
//                        roi_image(tmpx,tmpy)=i+1;
//                }
//            }
        ////////////////////////////////////////////////// //Visualizing all the ROIs in the image (before Template evaluation


        // Resize Cropped - with respect to template --> needed before evaluation (See. Boosting theory)
        double ratio = close_range_BBoxes(i)(3) / (Globals::template_size * 3.0);
        int new_height = (int)(cropped.y_size() * all_scales(nr_scales-1) / ratio);
        int new_width = (int)(cropped.x_size() * all_scales(nr_scales-1) / ratio);
        if(ratio > 1)
        {
            cropped.DownSample(new_width, new_height);
        }
        else if(ratio < 1)
        {
            cropped.UpSample(new_width, new_height);
        }


        Matrix<double> copy_cropped = cropped;

        for(int scale_index = 0; scale_index < all_scales.getSize(); ++scale_index)
        {
            cropped = copy_cropped; //just to scale from the initial image

            // Resize Cropped in loop with different scales
            int xSizeCropped = (int)(cropped.x_size() / all_scales(scale_index));
            int ySizeCropped = (int)(cropped.y_size() / all_scales(scale_index));

            if(all_scales(scale_index) != 1)
                cropped.DownSample(xSizeCropped , ySizeCropped);

            Matrix<double> extended_cropped(xSizeCropped + Globals::template_size, ySizeCropped+inc_cropped_height, 0.0);
            extended_cropped.insert(cropped, (int)(double_half_template_size)-1, 0);

            int distance_x_size = ceil((extended_cropped.x_size()-Globals::template_size)/(double)stride);
            int distance_y_size = ceil((ySizeCropped + inc_cropped_height - Globals::template_size - 1)/(double)stride);
            Matrix<double> resulted_distances(distance_x_size, distance_y_size);
            Matrix<double> resulted_medians(distance_x_size, distance_y_size);


            for(int y = 0, yyy = 0; yyy<distance_y_size; y=y+stride, ++yyy)
            {
                for(int x = 0, xxx = 0; xxx<distance_x_size; x=x+stride, ++xxx)
                {
                    double sum = 0;
                    int x_size = min(extended_cropped.x_size()-1, x+Globals::template_size) - x;
                    int y_size = min(extended_cropped.y_size()-1, y+Globals::template_size) - y;

                    int start_row = (int)max(0.0, y + y_size/2.0-5);
                    int end_row = (int)min((double)Globals::dImHeight-1, y + y_size/2.0+5);

                    int start_column = (int)max(0.0, x + x_size/2.0-5);
                    int end_column = (int)min((double)Globals::dImWidth-1, x + x_size/2.0+5);

                    // Normalize the cropped part of the image. regarding the current position of the template;
                    // Crop only some pixels in the middle
                    double median = AncillaryMethods::MedianOfMatrixRejectZero(extended_cropped, start_row, end_row, start_column, end_column);

                    if(median == 0)
                    {
                        resulted_distances(xxx,yyy) = 1000; //no need of mormalization
                        continue;
                    }

                    int x_start_of_temp = max(0, int_half_template_size - x);
                    double x_end_of_temp = min(Globals::template_size, extended_cropped.x_size() - int_half_template_size -x);
                    int evaluating_area = (x_end_of_temp - x_start_of_temp)*Globals::template_size+1;

                    if(evaluating_area > Globals::template_size * double_half_template_size)
                    {
                        for(int x_of_temp = x_start_of_temp; x_of_temp < x_end_of_temp; ++x_of_temp)
                        {
                            int x_of_extended_cropp = x + x_of_temp;

                            for(int y_of_temp = 0; y_of_temp < Globals::template_size; ++y_of_temp)
                            {
                                double difference = upper_body_template(x_of_temp, y_of_temp)-extended_cropped(x_of_extended_cropp, y_of_temp+y)/median;

                                sum += difference*difference;
                            }
                        }

                        resulted_distances(xxx,yyy) = sum/(double)evaluating_area;
                        resulted_medians(xxx, yyy) = median;
                    }
                    else
                    {
                        resulted_distances(xxx,yyy) = 1000;
                    }
                }
            } //Templates evaluated !!!

//            std::cout << "cropped.x_size() : " << cropped.x_size() << std::endl;
//            std::cout << "cropped.y_size() : " << cropped.y_size() << std::endl;
//            std::cout << "roi_image.x_size() : " << roi_image.x_size() << std::endl;
//            std::cout << "roi_image.y_size() : " << roi_image.y_size() << std::endl;

            //////////////// just for test (must be removed)
//            if(visualize_roi)
//            for(int tmpx=start_column, tmpxx=0; tmpxx<cropped.x_size(); ++tmpx,++tmpxx)
//            {
//                for(int tmpy=start_row, tmpyy=0; tmpyy<cropped.y_size(); ++tmpy,++tmpyy)
//                {
//                    if(tmpyy==0 || tmpyy==cropped.y_size()-1 || tmpxx==0 || tmpxx==cropped.x_size()-1) //if on the border of the image
//                        roi_image(tmpx,tmpy)=i+1;
//
//                    if(cropped(tmpxx,tmpyy)!=0)
//                        roi_image(tmpx,tmpy)=i+1;
//                }
//            }
            //////////////////

            Vector<Vector<double> > max_pos; //search for local maxima
            AncillaryMethods::NonMinSuppression2d(resulted_distances, max_pos, Globals::evaluation_NMS_threshold);

            int n_xSizeTemp = (int)(Globals::template_size*ratio/all_scales(scale_index));

            for(int j = 0; j < max_pos.getSize(); ++j)
            {
                Vector<double> bbox(6);
                bbox(0) = (max_pos(j)(0)*stride-double_half_template_size)*ratio/all_scales(scale_index) + close_range_BBoxes(i)(0);
                bbox(1) = (max_pos(j)(1)*stride)*ratio/all_scales(scale_index) +close_range_BBoxes(i)(1);
                bbox(2) = n_xSizeTemp;
                bbox(3) = n_xSizeTemp;
                bbox(4) = resulted_distances((int)max_pos(j)(0),(int)max_pos(j)(1));
                bbox(5) = resulted_medians((int)max_pos(j)(0),(int)max_pos(j)(1));

                result.pushBack(bbox);
            }
        }

        AncillaryMethods::GreedyNonMaxSuppression(result, Globals::evaluation_greedy_NMS_overlap_threshold, Globals::evaluation_greedy_NMS_threshold, upper_body_template, final_result);
    }


    /// To just vidualize final detections - comment if not needed
    // CHECK EVERY DETECTED_BBOXES

    //Copy final_result to keep information about the full detection
    Vector<Vector<double> > final_result_full = final_result;

    //First, let's get the information about the full detection (used to get PCD files)
    if (final_result_full.getSize()!=0)
    {
        // initialize size of variables
        x_distribution_full.clearContent();
        x_distribution_full.setSize(final_result.getSize());
        y_distribution_full.clearContent();
        y_distribution_full.setSize(final_result.getSize());

        // working on every bbox of final_result
        for (int i = 0; i < final_result_full.getSize(); ++i)
        {

            ///to get the head + body:
            int cropped_height = (int)(final_result_full(i)(3)/2.0);
            cropped_height += (final_result_full(i)(3));//* Globals::evaluation_inc_height_ratio;
            //final_result_full(i)(1) -= (final_result_full(i)(3) * Globals::evaluation_inc_height_ratio)/2.0;

            if( final_result_full(i)(1)+cropped_height >= Globals::dImHeight)
                cropped_height = Globals::dImHeight - (int)final_result_full(i)(1) - 1;

            if(Globals::verbose)
                cout << "(distances(i) " << distances(i)(0) << " radius " << distances(i)(1)/2.0 << endl;

            // Cropped and Filter depth_map with respect to distance from camera
            int start_column = (int)final_result_full(i)(0); // can be negativ !!!!!!!!!!!!!!!!!
            int end_column = (int)(final_result_full(i)(0) + final_result_full(i)(2));
            int start_row = (int)max(0.0, final_result_full(i)(1));
            int end_row = (int)final_result_full(i)(1) + cropped_height;

            Matrix<double> cropped1(end_column-start_column+1, end_row-start_row+1);
            // Set thrtesholds for depth_map
            double min_distance_threshold = distances(i)(0)- (distances(i)(1)+0.2)/3.0; //was 0.2, changed to 0.005
            double max_distance_threshold = distances(i)(0)+ (distances(i)(1)+0.2)/2.0; //was 0.2, changed to 0.5
            double d_val;


            for(int ii = 0, ii_depth = start_column; ii < cropped1.x_size(); ++ii, ++ii_depth) //Extract points satisfying distance conditions
            {
                for(int jj = 0, jj_depth = start_row; jj < cropped1.y_size(); ++jj, ++jj_depth)
                {
                    if (ii_depth < depth_map.x_size() && jj_depth < depth_map.y_size() && ii_depth > 0 && jj_depth > 0)
                    {
                        d_val = depth_map(ii_depth,jj_depth); // Problème ici !!
                    }
                    else d_val = distances(i)(0);

                    if(d_val <= min_distance_threshold || d_val >= max_distance_threshold)
                    {
                        cropped1(ii, jj) = 0;
                    }
                    else
                    {
                        cropped1(ii, jj) = d_val;   //for cropped image -> vizualisation
                        // pushBack data in variables for PCA
                        x_distribution_full(i).pushBack(ii_depth);
                        y_distribution_full(i).pushBack(jj_depth);
                    }
                }
            }

//        ////////////// just for test (must be removed)
            if(visualize_roi)
                for(int tmpx=start_column, tmpxx=0; tmpxx<cropped1.x_size(); ++tmpx,++tmpxx)
                {
                    for(int tmpy=start_row, tmpyy=0; tmpyy<cropped1.y_size(); ++tmpy,++tmpyy)
                    {
                        if(tmpyy==0 || tmpyy==cropped1.y_size()-1 || tmpxx==0 || tmpxx==cropped1.x_size()-1) //if on the border of the image
                            if (tmpx<640 && tmpy<480 && tmpx > 0 && tmpy > 0)
                                roi_image(tmpx,tmpy)=i+1;

                        if(cropped1(tmpxx,tmpyy)!=0 && tmpx<640 && tmpy<480 && tmpx > 0 && tmpy > 0)
                            roi_image(tmpx,tmpy)=i+1;
                    }
                }
        }
    }

    //Filter the point cloud for projection on the ground plane

    if (final_result.getSize()!=0)
    {
        // initialize size of variables
        x_distribution.clearContent();
        x_distribution.setSize(final_result.getSize());
        y_distribution.clearContent();
        y_distribution.setSize(final_result.getSize());

        // working on every bbox of final_result
        for (int i = 0; i < final_result.getSize(); ++i)
        {

            ///to get the head + body:
//             int cropped_height = (int)(final_result(i)(3)/2.0);
//            cropped_height += (final_result(i)(3));//* Globals::evaluation_inc_height_ratio;
//            //final_result(i)(1) -= (final_result(i)(3) * Globals::evaluation_inc_height_ratio)/2.0;
//
//            if( final_result(i)(1)+cropped_height >= Globals::dImHeight)
//                cropped_height = Globals::dImHeight - (int)final_result(i)(1) - 1;

            ///Suppress the head
            int cropped_height = (int)(final_result(i)(3)/2);
            //cropped_height += (final_result(i)(3));//* Globals::evaluation_inc_height_ratio;
            //final_result(i)(1) -= (final_result(i)(3) * Globals::evaluation_inc_height_ratio)/2.0;

            //Change start row not to have the head !
            int remove_height = (int)(final_result(i)(3)/2);
            final_result(i)(1) += remove_height;

            if( final_result(i)(1)+cropped_height >= Globals::dImHeight)
                cropped_height = Globals::dImHeight - (int)final_result(i)(1) - 1;

            if(Globals::verbose)
                cout << "(distances(i) " << distances(i)(0) << " radius " << distances(i)(1)/2.0 << endl;

            // Cropped and Filter depth_map with respect to distance from camera
            int start_column = (int)final_result(i)(0); // can be negativ !!!!!!!!!!!!!!!!!
            int end_column = (int)(final_result(i)(0) + final_result(i)(2));
            int start_row = (int)max(0.0, final_result(i)(1));
            int end_row = (int)final_result(i)(1) + cropped_height;

            Matrix<double> cropped1(end_column-start_column+1, end_row-start_row+1);
            // Set thrtesholds for depth_map
            double min_distance_threshold = distances(i)(0)- (distances(i)(1)+0.2)/3.0; //was 0.2, changed to 0.005
            double max_distance_threshold = distances(i)(0)+ (distances(i)(1)+0.2)/2.0; //was 0.2, changed to 0.5
            double d_val;


            for(int ii = 0, ii_depth = start_column; ii < cropped1.x_size(); ++ii, ++ii_depth) //Extract points satisfying distance conditions
            {
                for(int jj = 0, jj_depth = start_row; jj < cropped1.y_size(); ++jj, ++jj_depth)
                {
                    if (ii_depth < depth_map.x_size() && jj_depth < depth_map.y_size() && ii_depth > 0 && jj_depth > 0)
                    {
                        d_val = depth_map(ii_depth,jj_depth); // Problème ici !!
                    }
                    else d_val = distances(i)(0);

                    if(d_val <= min_distance_threshold || d_val >= max_distance_threshold)
                    {
                        cropped1(ii, jj) = 0;
                    }
                    else
                    {
                        cropped1(ii, jj) = d_val;   //for cropped image -> vizualisation
                        // pushBack data in variables for PCA
                        x_distribution(i).pushBack(ii_depth);
                        y_distribution(i).pushBack(jj_depth);
                    }
                }
            }

//        ////////////// just for test (must be removed)
//            if(visualize_roi)
//                for(int tmpx=start_column, tmpxx=0; tmpxx<cropped1.x_size(); ++tmpx,++tmpxx)
//                {
//                    for(int tmpy=start_row, tmpyy=0; tmpyy<cropped1.y_size(); ++tmpy,++tmpyy)
//                    {
//                        if(tmpyy==0 || tmpyy==cropped1.y_size()-1 || tmpxx==0 || tmpxx==cropped1.x_size()-1) //if on the border of the image
//                            if (tmpx<640 && tmpy<480 && tmpx > 0 && tmpy > 0)
//                                roi_image(tmpx,tmpy)=i+1;
//
//                        if(cropped1(tmpxx,tmpyy)!=0 && tmpx<640 && tmpy<480 && tmpx > 0 && tmpy > 0)
//                            roi_image(tmpx,tmpy)=i+1;
//                    }
//                }
        }
    }
    /// end of vizualisation - WORKING !!

    return final_result;
}
