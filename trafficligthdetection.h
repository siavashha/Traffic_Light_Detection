#ifndef TRAFFICLIGTHDETECTION_H
#define TRAFFICLIGTHDETECTION_H
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

class TrafficLigthDetection
{
public:
    TrafficLigthDetection();
    void Detector(cv::Mat);

};

#endif // TRAFFICLIGTHDETECTION_H
