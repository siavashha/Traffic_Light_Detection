#ifndef DETECTION_H
#define DETECTION_H
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <algorithm>
#include <math.h>

class Detection
{
public:
    Detection();
    std::vector<std::vector<cv::RotatedRect> >  Detect(cv::Mat image);
};

#endif // DETECTION_H
