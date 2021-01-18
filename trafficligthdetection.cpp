#include "trafficligthdetection.h"

TrafficLigthDetection::TrafficLigthDetection()
{
}

/* EillipseFitting: In this function, an ellipse is fitted to every cluster
 * of pixels in a binary image.
 * input: A binary image
 * Output: a vector of fitted ellipses; an ellipse is represented by a rotated rectangle
 */
std::vector<cv::RotatedRect> EillipseFitting(cv::Mat binary_image)
{
    cv::Mat temp_img = binary_image.clone();
    cv::cvtColor(temp_img,temp_img,cv::COLOR_GRAY2BGR);
    std::vector<cv::RotatedRect> marks;
    std::vector<double> error_vec;
    int num_of_ellipses = 0;
    // find contours of objects
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(binary_image , contours ,
                     cv::RETR_EXTERNAL , cv::CHAIN_APPROX_NONE);
    double out_of_ellipse_thr = 4.;
    for (int i = 0 ; i < contours.size() ; i++)
    {
        std::vector<cv::Point> contour = contours[i];
        // if the circumstance of a circle is less than 8 pixels, it is removed.
        if (contour.size()<8)
            continue;
        // fit an ellipse to the contours
        cv::RotatedRect box = fitEllipse(contour);
        //
        if (box.size.area() < 4) // 2 by 2 pixels
            continue;
        // change rotated box to an ellipse equatin
        double sum_error = 0.;
        double a = 0.5 * box.size.width;
        double b = 0.5 * box.size.height;
        if (a==0||b==0)
            continue;
        double alfa = box.angle * M_PI / 180.;
        double x0 = box.center.x;
        double y0 = box.center.y;
        double c1 = (cos(alfa) * cos(alfa))/(a*a)+(sin(alfa) * sin (alfa)) / (b*b);
        double c2 = -2. * cos(alfa) * sin(alfa) * (1. / (a*a) - 1. / (b*b) );
        double c3 = (sin(alfa) * sin(alfa))/(a*a)+(cos(alfa) * cos (alfa)) / (b*b);
        // estimating out of ellipse geometric error
        for (int j = 0 ; j < contour.size() ; j++)
        {
            cv::Point p = contour[j];
            double pixel_error = c1 * (p.x - x0) * (p.x - x0) + c2 * (p.x - x0) * (p.y - y0) + c3 * (p.y - y0) * (p.y - y0) - 1;
            sum_error = sum_error + pixel_error * pixel_error;
        }
        double error = std::sqrt(sum_error);
        if (error < out_of_ellipse_thr)
        {
            marks.push_back(box);
            num_of_ellipses++;
            std::cout << " object: " << num_of_ellipses << " out_of_ellipse error: " << error
                      << " ;area of ellipse is " << box.size.area() << std::endl;
            cv::ellipse(temp_img,box,cv::Scalar(0,0,255));
            error_vec.push_back(error);
        }
    }
    std::vector<double> sorted_error =  error_vec;
    std::sort(sorted_error.begin(), sorted_error.end());
//    int max_of_ellipses = 20;
//    for (int i = error_vec.size() -1  ; i > -1 ; i--)
//        if ( (error_vec[i] > sorted_error[max_of_ellipses-1]) && (max_of_ellipses < error_vec.size()) )
//            marks.erase(marks.begin()+i);
    return marks;
}

/* ColorMask: This function masks every color except red (with different illuminations)
 * input: A color image
 * Output: A binary image
 */
cv::Mat ColorMask(cv::Mat image)
{
    cv::Mat hls_image = image.clone();
    cv::cvtColor(hls_image, hls_image, cv::COLOR_BGR2HLS);
    // Threshold the HSL image, keep only the red pixels
    cv::Mat lower_red_hue_range; // 0-10
    cv::Mat upper_red_hue_range; // 170-179
    cv::inRange(hls_image, cv::Scalar(0, 10, 50), cv::Scalar(10, 255, 255), lower_red_hue_range);
    cv::inRange(hls_image, cv::Scalar(170, 10, 50), cv::Scalar(180, 255, 255), upper_red_hue_range);
    // Combine the above two images
    cv::Mat red_hue_image;
    cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
    return red_hue_image;
}


void TrafficLigthDetection::Detector(cv::Mat image)
{
        std::vector<int> params;
        params.push_back(CV_IMWRITE_PNG_COMPRESSION);
        params.push_back(9);

        cv::Mat masked_image = ColorMask(image);
        cv::imwrite("2_masked.png",masked_image,params);
        cv::medianBlur(masked_image , masked_image , 3);
        cv::imwrite("2_1_median.png",masked_image,params);
        std::vector<cv::RotatedRect> marks = EillipseFitting(masked_image);
        cv::cvtColor(masked_image,masked_image,cv::COLOR_GRAY2BGR);
        for (int i = 0 ; i < marks.size(); i++)
            cv::ellipse(masked_image , marks[i] ,cv::Scalar(0,0,255),cv::FILLED);
        cv::imwrite("3_ellipse_fitting.png",masked_image,params);

}
