#include "detection.h"
#define lower_threshold 10
#define upper_threshold 170
#define out_of_ellipse_thr 10 // 1 pixel
#define max_of_ellipses 16
#define out_of_homography_thr 100 // 10 pixels
#define n_marks 4

// Constructor
Detection::Detection()
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
    for (int i = error_vec.size() -1  ; i > -1 ; i--)
        if ( (error_vec[i] > sorted_error[max_of_ellipses-1]) && (max_of_ellipses < error_vec.size()) )
            marks.erase(marks.begin()+i);
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
    cv::inRange(hls_image, cv::Scalar(0, 10, 50), cv::Scalar(lower_threshold, 255, 255), lower_red_hue_range);
    cv::inRange(hls_image, cv::Scalar(upper_threshold, 10, 50), cv::Scalar(180, 255, 255), upper_red_hue_range);
    // Combine the above two images
    cv::Mat red_hue_image;
    cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
    return red_hue_image;
}

/* Combinator: This function creates a vecotr of all possible sequences.
 * For instance, lets assumed that 10 ellipses have been found and the marker
 * has four ellipses (circles). This funciton gives a vector of possible answer
 * such as {0,0,0,1,0,0,1,1,0,1}
 * Input: The number of observed ellipses,
 *        The number of ellipses (circles) in the marker.
 * Output: A vector of possible answers
 */
std::vector<std::vector<bool> > Combinator
(unsigned int numOfObserved , unsigned int numOfMarkers)
{
    /*set_size of power set of a set with set_size
      n is (2**n -1)*/
    unsigned int pow_set_size = pow(2, numOfObserved);
    std::vector<std::vector<bool> > solutions;
    /*Run from counter 000..0 to 111..1*/
    for(int counter = 0; counter < pow_set_size; counter++)
    {
        std::vector<bool> isSolution;
        unsigned int num = 0;
        for(int j = 0; j < numOfObserved; j++)
        {
            /* Check if jth bit in the counter is set
             If set then pront jth element from set */
            if(counter & (1<<j))
            {
                isSolution.push_back(true);
                num++;
            }
            else
                isSolution.push_back(false);
        }
        if (num == numOfMarkers)
            solutions.push_back(isSolution);
    }
    return solutions;
}

/* OrderEllipses: This function sorts the ellipses clockwise. This function
 * prevents conic mismatches later on.
 * Input: The (unorganized) observed ellipses,
 * Output: The ordered observed ellipses.
 */
std::vector<cv::RotatedRect> OrderEllipses
(std::vector<cv::RotatedRect> unorderedEllipses)
{
    std::vector<cv::RotatedRect> orderedEllipses;
    double x_origin = 0, y_origin = 0;
    int n = unorderedEllipses.size();
    double coeff = 1. / (double)(n);
    for (int i = 0 ; i < n ; i++)
    {
        x_origin = x_origin + coeff * unorderedEllipses[i].center.x;
        y_origin = y_origin + coeff * unorderedEllipses[i].center.y;
    }
    // finding azimuth zero
    double min_dxy = INFINITY;
    double angle_ref;
    for (int i = 0 ; i < n ; i++)
    {
        double dx = unorderedEllipses[i].center.x - x_origin;
        double dy = unorderedEllipses[i].center.y - y_origin;
        if (min_dxy > dx + dy)
        {
            min_dxy = dx + dy;
            angle_ref  = atan2(dy , dx );
        }
    }
    // calculating the angles of ellipses
    std::vector<double> angle_vector;
    for (int i = 0 ; i < n ; i++)
    {
        double dx = unorderedEllipses[i].center.x - x_origin;
        double dy = unorderedEllipses[i].center.y - y_origin;
        double angle = atan2(dy , dx ) - angle_ref;
        if (angle < 0)
            angle = angle + 2 * M_PI;
        if (angle > 2 * M_PI)
            angle = angle - 2 * M_PI;
        angle_vector.push_back(angle);
    }
    // order the ellipses
    std::vector<double> sorted_angle = angle_vector;
    std::sort( sorted_angle.begin(),sorted_angle.end() );
    for (int i = 0 ; i < sorted_angle.size() ; i++)
        for (int j = 0 ; j < angle_vector.size() ; j++)
            if (angle_vector[j] == sorted_angle[i])
            {
                orderedEllipses.push_back(unorderedEllipses[j]);
                continue;
            }
    return orderedEllipses;
}

/* NormalizeEllipses: This function normalizes the observed ellipses.
 * It means that the origin of the coordinate system is transferred to
 * the center of mass of ellipses and the distance of farthest ellipse
 * from the origin is scaled to square root of 2.
 * Input: The ordered observed ellipses,
 * Output: The normalized observed ellipses.
 */
std::vector<cv::RotatedRect> NormalizeEllipses
(std::vector<cv::RotatedRect> orderedEllipses)
{
    // change the origin
    double x_origin = 0, y_origin = 0;
    int n = orderedEllipses.size();
    double coeff = 1. / (double)(n);
    std::vector<cv::RotatedRect> normalizedEllipses;
    for (int i = 0 ; i < n ; i++)
    {
        x_origin = x_origin + coeff * orderedEllipses[i].center.x;
        y_origin = y_origin + coeff * orderedEllipses[i].center.y;
    }
    double d_max = 0;
    for (int i = 0 ; i < n ; i++)
    {
        double dx = orderedEllipses[i].center.x - x_origin;
        double dy = orderedEllipses[i].center.y - y_origin;
        if (d_max < std::sqrt(dx*dx+dy*dy) )
            d_max = std::sqrt(dx*dx+dy*dy);
    }
    // scaling the observed ellipses
    double scale = std::sqrt(2) / d_max;
    for (int i = 0 ; i < n ; i++)
    {
        cv::RotatedRect r;
        r.size.width = scale * orderedEllipses[i].size.width;
        r.size.height = scale * orderedEllipses[i].size.height;
        r.center.x = scale * (orderedEllipses[i].center.x - x_origin);
        r.center.y = scale * ( orderedEllipses[i].center.y - y_origin);
        r.angle = orderedEllipses[i].angle;
        normalizedEllipses.push_back(r);
    }
    return normalizedEllipses;
}

/* Ellipses2Conics: This function converts the ellipses from rotated rectangle representation
 * to the matrix representation.
 * Input: a vector ellipses represented by rotated rectangle,
 * Output: a vector ellipses represented by 3 by 3 matrix.
 */
std::vector<cv::Mat> Ellipses2Conics
(std::vector<cv::RotatedRect> ellipses)
{
    std::vector<cv::Mat> conics;
    for (int i = 0 ; i < ellipses.size(); i++ )
    {
        cv::RotatedRect box = ellipses[i];
        double a = 0.5 * box.size.width;
        double b = 0.5 * box.size.height;
        if (a==0||b==0)
            continue;
        double alfa = box.angle * M_PI / 180.;
        double x0 = box.center.x;
        double y0 = box.center.y;
        double A = (cos(alfa) * cos(alfa))/(a*a)+(sin(alfa) * sin(alfa)) / (b*b);
        double B = -2 * cos(alfa) * sin(alfa) * (1 / (a*a) - 1 / (b*b) );
        double C = (sin(alfa) * sin(alfa))/(a*a)+(cos(alfa) * cos(alfa)) / (b*b);
        double D = -1 * (2 * A * x0 + y0 * B);
        double E = -1 * (2 * C * y0 + B * x0);
        double F = A * x0 * x0 + C * y0 * y0 + B * x0 * y0 - 1;
        cv::Mat conic = (cv::Mat_<double>(3, 3) <<
                         A , B/2 , D/2,
                         B/2 , C , E/2,
                         D/2 , E/2 , F);
        conics.push_back(conic);
    }
    return conics;
}

/* MarkerConics: This function provides the matrix representation
 * of the marker in frontal view.
 * Output: a vector ellipses represented by 3 by 3 matrix in frontal view.
 */
std::vector<cv::Mat> MarkerConics()
{
    std::vector<cv::Mat> conics_marker;
    // Please adjust radius if you use different marker
    double r = 0.33; //pixels
    double l = 1; //pixels
    cv::Mat a , b , c , d;
    // first circle/conic
    a = (cv::Mat_<double>(3, 3) <<
         1, 0, -l,
         0, 1, -l,
         -l , -l , 2*l*l-r*r);
    // second circle/conic
    b = (cv::Mat_<double>(3, 3) <<
         1, 0, -l,
         0, 1 ,l,
         -l , l , 2*l*l-r*r);
    // third circle/conic
    c = (cv::Mat_<double>(3, 3) <<
         1, 0, l,
         0, 1 ,l,
         l , l , 2*l*l-r*r);
    // fourth circle/conic
    d = (cv::Mat_<double>(3, 3) <<
         1, 0, l,
         0, 1 ,-l,
         l , -l , 2*l*l-r*r);
    conics_marker.push_back(a);
    conics_marker.push_back(b);
    conics_marker.push_back(c);
    conics_marker.push_back(d);
    return conics_marker;
}

/* CalcHomography: This function calcuates the homography matrix between the ellipses
 * frontal view and observed view. The out of homography error is calculated and passed
 * by pointer.
 * Input: The ellipses of the marker in frontal view
 *        The ellipses of the marker in observed view
 *        The out of homography error; This value is actually estimated in the function
 *          and is passed by pointer
 * Output:  3 by 3 homography matrix between ellipses in frontal view and observed view.
 */
cv::Mat CalcHomography(std::vector<cv::Mat> marker_conic ,
                       std::vector<cv::Mat> obs_conic , double& error )
{
    for (int i = 0 ; i < marker_conic.size(); i++ )
    {
        // normalization of scale
        double scale = cbrt(cv::determinant(marker_conic[i]) / cv::determinant(obs_conic[i]));
        obs_conic[i] =  obs_conic[i] * scale;
    }
    // Calculating M (9 by 9) matrix
    cv::Mat M(0,0,CV_64F,cv::Scalar(0));
    for (int i = 0 ; i < marker_conic.size(); i++)
        for (int j = 0 ; j < obs_conic.size(); j++)
        {
            if (i!=j)
            {
                cv::Mat C_marker = marker_conic[i].inv() * marker_conic[j];
                cv::Mat C_image = obs_conic[i].inv() * obs_conic[j];
                cv::Mat m(9,9,CV_64F,cv::Scalar(0));
                cv::Mat m11(3,3,CV_64F,cv::Scalar(0));
                cv::Mat m12(3,3,CV_64F,cv::Scalar(0));
                cv::Mat m13(3,3,CV_64F,cv::Scalar(0));
                cv::Mat m21(3,3,CV_64F,cv::Scalar(0));
                cv::Mat m22(3,3,CV_64F,cv::Scalar(0));
                cv::Mat m23(3,3,CV_64F,cv::Scalar(0));
                cv::Mat m31(3,3,CV_64F,cv::Scalar(0));
                cv::Mat m32(3,3,CV_64F,cv::Scalar(0));
                cv::Mat m33(3,3,CV_64F,cv::Scalar(0));
                m11 = C_image.at<double>(0,0) * cv::Mat::eye(3,3,CV_64F) - C_marker.t();
                m12 = C_image.at<double>(0,1) * cv::Mat::eye(3,3,CV_64F);
                m13 = C_image.at<double>(0,2) * cv::Mat::eye(3,3,CV_64F);
                m21 = C_image.at<double>(1,0) * cv::Mat::eye(3,3,CV_64F);
                m22 = C_image.at<double>(1,1) * cv::Mat::eye(3,3,CV_64F) - C_marker.t();
                m23 = C_image.at<double>(1,2) * cv::Mat::eye(3,3,CV_64F);
                m31 = C_image.at<double>(2,0) * cv::Mat::eye(3,3,CV_64F);
                m32 = C_image.at<double>(2,1) * cv::Mat::eye(3,3,CV_64F);
                m33 = C_image.at<double>(2,2) * cv::Mat::eye(3,3,CV_64F) - C_marker.t();
                cv::Mat m1,m2,m3;
                cv::hconcat(m11,m12,m1);
                cv::hconcat(m1,m13,m1);
                cv::hconcat(m21,m22,m2);
                cv::hconcat(m2,m23,m2);
                cv::hconcat(m31,m32,m3);
                cv::hconcat(m3,m33,m3);
                cv::vconcat(m1,m2,m);
                cv::vconcat(m,m3,m);
                if (M.size().area() == 0)
                    M = m;
                else
                    cv::vconcat(M,m,M);
            }
        }
    // estimate h vector
    cv::Mat w, u, vt;
    cv::SVD::compute(M, w, u, vt);
    cv::Mat h = vt.col(vt.cols - 1);
    // calculating out of homography error
    cv::Mat error_vec = M * h;
    //std::cout << " error = " << error.t() << std::endl;
    cv::Mat sse_mat = error_vec.t() * error_vec;
    double sse = sse_mat.at<double>(0,0);
    error = std::sqrt(sse) / (n_marks * (n_marks-1)); // per pair of ellipses
    std::cout << " error = " << error << std::endl;
    return h;
}

/* PlanarConstraint: This function makes sure that the observed ellipses are on a plane
 *                   In other words, this function detects markers by estimating the coplanrity
 *                   of the ellipses.
 * Input: a vector of detected ellipses
 * Output: a vector of markers; every marker is a vector of four ellipses.
 */
std::vector<std::vector<cv::RotatedRect> > PlanarConstraint
    (std::vector<cv::RotatedRect> marks)
{
    std::vector<cv::RotatedRect> marker;
    std::vector<std::vector<cv::RotatedRect> > markers;
    std::vector<cv::Mat> h_vec_vec;
    bool isMarkerFound;
    int num_of_marker = 0;
    do {
        isMarkerFound = false;
        std::vector<std::vector<bool> > combinations = Combinator
                (marks.size() , n_marks);
        double least_error = INFINITY;
        std::vector<bool> best_combination;
        cv::Mat h_vec;
        for (int i = 0 ; i < combinations.size() ; i++)
        {
            std::vector<bool> sol = combinations[i];
            std::vector<cv::RotatedRect> unorderedEllipses;
            std::vector<cv::Mat> conics_marker = MarkerConics();
            for (int j = 0 ; j < sol.size(); j++ )
            {
                std::cout << sol[j] << ",";
                if (sol[j])
                    unorderedEllipses.push_back(marks[j]);
            }
            std::vector<cv::RotatedRect> orderedEllipses = OrderEllipses
                    (unorderedEllipses);
            std::vector<cv::RotatedRect> normalizedEllipses = NormalizeEllipses
                    (orderedEllipses);
            std::vector<cv::Mat> obs_conics = Ellipses2Conics(normalizedEllipses);
    . M9. L6/ L6/ M7. M7- N6- N6- M7- M9- K7- M4- L6- L7, K8+ I8, K4- L3- J7, I8, J7, J7, J6- J6- J6- J6- J7, J7, J7, I8, I8, I7- K7. K7. K7. K7- M8, R6, R5- N6. L8/ M90 P70 P70 P70 N:0 P>1 U?2 \=5 fC9 pK? |VH ”fT ·yg ßœ ÿæª ÿÿâ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿâß ÿ¡ Õ`o ¢CJ ƒ<6 j94 a92 Z:1 V:0 U81 R:1 Q;1 P:1 N:0 M9/ N8/ Q8. N:. M<- N:. O9. N8/ L6/ L6/ M7. M7. N6. N6. P6- M7- K9. J6/ N80 L;0 L;0 K9/ K9/ K9/ O8/ O80 N:0 L;/ M9/ N:0 P:0 R:/ Q;/ Q;0 Q;0 Q=1 S=1 U;1 U:2 S:2 Q:2 R92 Q:2 Q;1 R:1 Q;1 Q;0 Q=/ Q;/ Q;0 S<2 Q=3 T<3 T<3 S<2 S<2 S<2 S<2 W<2 W=3 X>3 X>3 V?3 W=3 V=2 U?2 U?2 V?3 UA4 Y@6 \?8 bE: iK> uNB VG ŠaN ”lV ¦x` ½‰m ç£ ÿÓ£ ÿûÛ ÿÿù ÿÿı ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿÿ ÿÿú çèñ ¸«Ô —{¤ of‡ KX| :Nl 4<X 1'F -4 *) '% '" )! +" ,& .%) 2$+ 5*- ?0. W,1 cG7 vbA wpP pvV s`S nOM gHB ^F: L;6 6'- * "         !#  $                           ÜÄ¯ ÚÅ± ÙÇ± ØÈ± ÙÇ± ÚÄ± ×Å± ÙÅ¯ àÃ¯ ßÄ± ÙÊ± ÚË¯ ÜË® ŞÈ¯ ÚÈ¯ ÙÉ® ŞÆ° àÂ³ ÛÆ´ ÙÊ³ ÛË² İÉ¯ ßÈ¯ ŞÇ± ŞÇ± àÇ¯ äÇ¯ àÇ² İÉ² ÚÈ± ×Ê± ÙÇ² ÜÃ³ ÛÊ³ ÚÇ³ ÜÈ² ŞÇ± âÃ² áÄ² áÃ³ àÆ³ ßÅ³ İÄ³ ÙÊ² ÛÊ² ÛÉ³ ŞÆ³ İÈ³ İÎ² áÆ± âÃ± àÆ³ ßÉ³ ßÉ± ãÆ± àÇ± ÜÈ± ÛÇ² ÛÅ² ÛÅ± áÁ± âÂ° ÚÆ° ÖÆ¯ ÙÂ¯ ÙÂ® ×Å¯ ÛÂ± ŞÃ³ ßÅ´ İÆµ ÛÊ´ ÚÈ² ÜÄ± ÜÌ° ÙÑ° İÍ° àÃ± âÃ² ÜË³ ÒÎµ ÓÆ¶ ×Æ¶ ÖÉµ ÕÉ´ ÓÊ´ ÔÉµ ÚÅ· ßÅ· ßÆµ ÛÉ´ ÚÊµ İÄ´ ÚÈ² ÙË± ØÈ² ØÇ³ ×Ä³ ÚÄ² ØÈ± ÜÈ± ÛÇ² ÖÄ² ÔÁ± ÕÅ¯ ØÇ­ ßÆ­ ŞÆ¯ ×Ç± ÕÆ³ ×Æ³ ×Ä³ ÙÂ² ÙÂ° ÕÀ¯ ÕÀ° ÖÃ° ×Å¯ ÙÀ° Ú¼± ÜÁ± İÂ± İÅ¯ ÚÇ­ ÖÆ¬ ÔÄ­ ×Ä¬ ÚÄ« ×Åª ×Åª ÕÅª ÔÇ« Üº« Û¹­ Ô¿® Ö¾® Ú¾° Ø¾³ ÖÃ³ ÕÂ± Õ½± ×¹² Ù½± ÙÅ¯ ÛÀ® Ş»® ×À® ÕÂ¯ Ö¾¯ Ú»® Ù¾­ ÛÀ« ÚÁ¬ ÙÂ® ×Å¯ ×Â° Ö¾® ÕÁ« ÒÃ« ÑÃ® Ñ¾° Ò»° Ø»¯ ×¾­ Ò¼­ Ñ¿­ ÕÂ® ×½° Ò»° Ò¼® Øº® Øº® Ò¿¯ ÎÀ± ÌÀ² ÌÂ± ÌÂ± Í¾² Í¾± Ò»® Ø´­ Ø¶¬ Ğº¬ Ğ¿¬ ÔÂ¬ Õ¿­ ÎÀ­ Ğ¼­ Õ¼¬ ÔÀ« ÎÀ« Î¿¬ Õ¼¬ Ø»© Ùº© ×½« ÓÀ¬ Ô¿¬ Õ¿­ Õ¾¯ ÑÁ¯ ÎÅ­ ËÀ¬ É¿­ Ï¾® Ğ¿® ÑÂ­ Ù¼¬ Üº« Û»¬ Û¸¬ Õº­ Ğ¼® Ï¿­ Ñ¿­ ĞÀ« Ï½¬ Ğ¼® Ø¼­ Û½© Õ¾© Óº« Ñ¹¬ Ñº« Ïº« Í½¬ ÎÀ­ Ò½­ Óº­ Ó½« ĞÃ« ĞÂ¬ Ñ¼¬ Óºª Ô¸ª Ô·« Ô·¬ Ğ»« Ì»« Íº« Ë½« Ã¿« ¸¼¢ §¯‘ ”‘x †hV iDF V?G ZSG SZH ONC I=9 ;52 31. 4.* 20' 4.& 5-& 8+& 7-% 6,$ 6,$ 5.$ 7.$ 7.$ 7.$ 7.$ 8,$ 7.$ 6/$ 4/$ 2/$ 4.% 30% 6.% 9.$ 9.$ 8/$ 70% 8.% :.% :.% 6.% 50% 50% 7.$ 8/$ 90% 70% ;-& ;0& ;1& <.& :.& 61' 60' 7.' 80' :.& :.& :.& :.& 8.& 60' 73' :1' 81' 90& 90& 90& 71& 91& ;0& :0' 80' 80' 80' 80' 60' 60' 61' 60' 90& 90& 80' 61' 60( 8/( :1' ;2' ;1& 73' 81' 90& 91& 91& ;1& <2& >2& A0& D/( O/. Y<= nXT ’{h «v ³“ ¸—‚ »“ƒ »“ ¹“ º“} º–} ¸• ·“~ ¯y §ˆr xi ’gd ge ’{d Š~c zm[ mQN P@I 9CJ 2GJ /FJ 0CJ 3@J 2AK 3AM 5@N 4CN 2FN 2CN 0?N 0@M 0@L 1?L 1@K 1@K 1@K 0BK 0BM 0AO /@P /B