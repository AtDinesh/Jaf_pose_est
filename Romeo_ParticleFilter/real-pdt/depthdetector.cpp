#include "depthdetector.h"

DepthDetector::DepthDetector()
{
}

Vector<Vector<double> > DepthDetector::EvaluateTemplate(const Matrix<double> &upper_body_template, const Matrix<double> &depth_map, Vector<Vector<double> > &close_range_BBoxes, Vector<Vector<double> > distances)
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
    if(visualize_roi)
        roi_image = Matrix<int>(Globals::dImWidth, Globals::dImHeight, 0);
    //////////////////////////////////////////////

    for (int i = 0; i < close_range_BBoxes.getSize(); i++)
    {
        Vector<Vector<double> > result;

        int cropped_height =  (int)(close_range_BBoxes(i)(3)/2.0);
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

        //////////////// just for test (must be removed)
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
        //////////////////////////////////////////////////


        // Resize Cropped - with respect to template
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

        for(int scale_index = 0; scale_index < all_scales.getSize(); scale_index++)
        {
            cropped = copy_cropped;

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

            for(int y = 0, yyy = 0; yyy<distance_y_size; y=y+stride, yyy++)
            {
                for(int x = 0, xxx = 0; xxx<distance_x_size; x=x+stride, xxx++)
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
                        resulted_distances(xxx,yyy) = 1000;
                        continue;
                    }

                    int x_start_of_temp = max(0, int_half_template_size - x);
                    double x_end_of_temp = min(Globals::template_size, extended_cropped.x_size() - int_half_template_size -x);
                    int evaluating_area = (x_end_of_temp - x_start_of_temp)*Globals::template_size+1;

                    if(evaluating_area > Globals::template_size * double_half_template_size)
                    {
                        for(int x_of_temp = x_start_of_temp; x_of_temp < x_end_of_temp; x_of_temp++)
                        {
                            int x_of_extended_cropp = x + x_of_temp;

                            for(int y_of_temp = 0; y_of_temp < Globals::template_size; y_of_temp++)
                            {
                                double difference = upper_body_template(x_of_temp, y_of_temp)-extended_cropped(x_of_extended_cropp, y_of_temp+y)/median;

                                sum += difference*difference;
                            }
                        }

                        resulted_distances(xxx,yyy) = sum/(double)evaluating_area;
                        resulted_medians(xxx,yyy) = median;
                    }
                    else
                    {
                        resulted_distances(xxx,yyy) = 1000;
                    }
                }
            }

            Vector<Vector<double> > max_pos;
            AncillaryMethods::NonMinSuppression2d(resulted_distances, max_pos, Globals::evaluation_NMS_threshold);

            int n_xSizeTemp = (int)(Globals::template_size*ratio/all_scales(scale_index));

            for(int j = 0; j < max_pos.getSize(); j++)
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

        static int oo=0;
        char pp[300];
        sprintf(pp,"/home/hosseini/r_%08d_0.txt",oo++);
        Matrix<double>(result).WriteToTXT(pp);
        AncillaryMethods::GreedyNonMaxSuppression(result, Globals::evaluation_greedy_NMS_overlap_threshold, Globals::evaluation_greedy_NMS_threshold, upper_body_template, final_result);
    }

    return final_result;
}
