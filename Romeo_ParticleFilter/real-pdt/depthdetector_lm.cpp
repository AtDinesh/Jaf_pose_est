#include "depthdetector_lm.h"
#include "KConnectedComponentLabeler.h"

DepthDetector_LM::DepthDetector_LM()
{
}

Vector<Vector<double> > DepthDetector_LM::EvaluateTemplate(const Matrix<double> &upper_body_template, const Matrix<double> &depth_map, Vector<Vector<double> > &close_range_BBoxes, Vector<Vector<double> > distances)
{
    int head_pose = 4*Globals::template_size/30;
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

    if(visualize_roi)
        roi_image = Matrix<int>(Globals::dImWidth, Globals::dImHeight, 0);

    for (int i = 0; i < close_range_BBoxes.getSize(); i++)
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

        double min_distance_threshold = distances(i)(0)- (distances(i)(1)+0.2)/2.0;
        double max_distance_threshold = distances(i)(0)+ (distances(i)(1)+0.2)/2.0;
        for(int ii = 0, ii_depth = start_column; ii < cropped.x_size(); ii++, ii_depth++)
        {
            for(int jj = 0, jj_depth = start_row; jj < cropped.y_size(); jj++, jj_depth++)
            {
                if(depth_map(ii_depth,jj_depth) <= min_distance_threshold || depth_map(ii_depth,jj_depth) >= max_distance_threshold)
                {
                    cropped(ii, jj) = 0;
                }
                else
                {
                    cropped(ii, jj) = depth_map(ii_depth, jj_depth);
                }
            }
        }

        ////////////////
        if(visualize_roi)
        {
            for(int tmpx=start_column, tmpxx=0; tmpxx<cropped.x_size(); tmpx++,tmpxx++)
            {
                for(int tmpy=start_row, tmpyy=0; tmpyy<cropped.y_size(); tmpy++,tmpyy++)
                {
                    if(tmpyy==0 || tmpyy==cropped.y_size()-1 || tmpxx==0 || tmpxx==cropped.x_size()-1)
                        roi_image(tmpx,tmpy)=i+1;

                    if(cropped(tmpxx,tmpyy)!=0)
                        roi_image(tmpx,tmpy)=i+1;
                }
            }
        }
        /////////////////////////////////

        // Resize Cropped - with respect to template
        double ratio = close_range_BBoxes(i)(3) / (Globals::template_size * 3.0);
        int new_height = (int)(cropped.y_size() * all_scales(nr_scales-1) / ratio);
        int new_width = (int)(cropped.x_size() * all_scales(nr_scales-1) / ratio);
        if(new_height<=0 || new_width<=0)
            continue;
        if(cropped.y_size() > new_height)
        {
            cropped.DownSample(new_width, new_height);
        }
        else if(cropped.y_size() < new_height)
        {
            cropped.UpSample(new_width, new_height);
        }


        //*******************************************************************************************************
        Matrix<int> b_cropped(cropped.x_size(), cropped.y_size());
        for(int ic = 0; ic<cropped.x_size(); ++ic)
        {
            for(int j=0; j<cropped.y_size(); ++j)
            {
                if(cropped(ic,j)>0)
                    b_cropped(ic,j)=1;
                else
                    b_cropped(ic,j)=0;
            }
        }

        KConnectedComponentLabeler ccl(Globals::region_size_threshold, b_cropped.data(), b_cropped.x_size(), b_cropped.y_size());

        ccl.Process();

        Matrix<double> components(cropped.x_size(),cropped.y_size());
        //*******************************************************************************************************
        for(int cr_com = 0; cr_com < ccl.m_ObjectNumber; ++cr_com)
        {
            for(int x=0; x<cropped.x_size(); ++x)
            {
                for(int y=0; y<cropped.y_size(); ++y)
                {
                    if(b_cropped(x,y)==cr_com+1)
                        components(x,y) = cropped(x,y);
                    else
                        components(x,y) = 0;
                }
            }

            Matrix<double> copy_component = components;

            for(int scale_index = 0; scale_index < all_scales.getSize(); scale_index++)
            {
                copy_component = components;

                // Resize Cropped in loop with different scales
                int xSizeCropped = (int)(copy_component.x_size() / all_scales(scale_index));
                int ySizeCropped = (int)(copy_component.y_size() / all_scales(scale_index));

                if(all_scales(scale_index) != 1)
                    copy_component.DownSample(xSizeCropped , ySizeCropped);

                Matrix<double> extended_cropped(xSizeCropped + Globals::template_size, ySizeCropped+inc_cropped_height, 0.0);
                extended_cropped.insert(copy_component, (int)(double_half_template_size)-1, 0);

                //*Local Max *******************************
                Matrix<double> extended_cropped_open = extended_cropped;
                AncillaryMethods::MorphologyOpen(extended_cropped_open);
                Vector<double> ys ,slopes;
                AncillaryMethods::ExtractSlopsOnBorder(extended_cropped_open, ys, slopes);
                Vector<int> max_xs = AncillaryMethods::FindLocalMax(slopes);
                //******************************************

                int distances_matrix_x_size = max_xs.getSize();

                Vector<double> resulted_distances(distances_matrix_x_size);
                Vector<double> resulted_medians(distances_matrix_x_size);

                Vector<int> rxs(max_xs.getSize()),rys(max_xs.getSize());
                for(int ii = 0; ii<max_xs.getSize(); ++ii)
                {
                    int cx = max(max_xs(ii)-int_half_template_size,0);
                    int cy = max(extended_cropped.y_size()-(int)ys(max_xs(ii))-head_pose, 0);
                    rxs(ii) = cx;
                    rys(ii) = cy;
                    double local_result, local_best=1000;

                    for(int y=cy; y<=cy+1*stride; y+=stride)
                    {
                        if(y>=extended_cropped.y_size() || y>=extended_cropped.y_size()) continue;
                        int y_size = min(extended_cropped.y_size()-1, y+Globals::template_size) - y;

                        int start_row = (int)max(0.0, y + y_size/2.0-5);
                        int end_row = (int)min((double)Globals::dImHeight-1, y + y_size/2.0+5);
                        if(end_row >=extended_cropped.y_size()) continue;

                        for(int x=max(0,cx-5*stride); x<=cx+5*stride; x+=stride)
                        {
                            if(x>=extended_cropped.x_size()) continue;
                            int x_size = min(extended_cropped.x_size()-1, x+Globals::template_size) - x;

                            int start_column = (int)max(0.0, x + x_size/2.0-5);
                            int end_column = (int)min((double)Globals::dImWidth-1, x + x_size/2.0+5);

                            // Normalize the cropped part of the image. regarding the current position of the template;
                            // Crop only some pixels in the middle
                            double median = AncillaryMethods::MedianOfMatrixRejectZero(extended_cropped, start_row, end_row, start_column, end_column);
                            if(median == 0)
                            {
                                resulted_distances(ii) = 1000;
                                continue;
                            }

                            int x_start_of_temp = max(0, int_half_template_size - x);
                            double x_end_of_temp = Globals::template_size;
                            int evaluating_area = (x_end_of_temp - x_start_of_temp)*Globals::template_size+1;

                            double sum = 0;

                            if(evaluating_area > Globals::template_size * double_half_template_size)
                            {
                                for(int x_of_temp = 0; x_of_temp < x_end_of_temp; x_of_temp++)
                                {
                                    int x_of_extended_cropp = x + x_of_temp;

                                    for(int y_of_temp = 0; y_of_temp < Globals::template_size; y_of_temp++)
                                    {
                                        if(y_of_temp+y>=extended_cropped.y_size()) break;
                                        double difference = upper_body_template(x_of_temp, y_of_temp)-extended_cropped(x_of_extended_cropp, y_of_temp+y)/median;

                                        sum += difference*difference;
                                    }
                                }

                                local_result = sum/(double)evaluating_area;
                                if(local_best>local_result)
                                {
                                    local_best = local_result;
                                    resulted_medians(ii) = median;
                                    rxs(ii)=x;
                                    rys(ii)=y;
                                }
                            }
                        }
                    }
                    resulted_distances(ii) = local_best;
                }

                int n_xSizeTemp = (int)(Globals::template_size*ratio/all_scales(scale_index));

                for(int ii =0; ii<resulted_distances.getSize(); ++ii)
                {
                    if(resulted_distances(ii)<Globals::evaluation_NMS_threshold_LM)
                    {
                        int x = rxs(ii);
                        int y = rys(ii);

                        Vector<double> bbox(6);
                        bbox(0) = (x-double_half_template_size)*ratio/all_scales(scale_index) + close_range_BBoxes(i)(0);
                        bbox(1) = y*ratio/all_scales(scale_index) +close_range_BBoxes(i)(1);
                        bbox(2) = n_xSizeTemp;
                        bbox(3) = n_xSizeTemp;
                        bbox(4) = resulted_distances(ii);
                        bbox(5) = resulted_medians(ii);

                        result.pushBack(bbox);
                    }
                }
            }
        }
        AncillaryMethods::GreedyNonMaxSuppression(result, Globals::evaluation_greedy_NMS_overlap_threshold, Globals::evaluation_greedy_NMS_threshold, upper_body_template, final_result);
    }
    return final_result;
}
