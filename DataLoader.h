#ifndef DATALOADER_H
#define DATALOADER_H
#include <boost/algorithm/string.hpp>
//#include <boost/algorithm/string/split.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <queue>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class DataLoader
{
private:
    std::string KITTI_leftCam_DIR ;
    std::string KITTI_rightCam_DIR;
    std::string KITTI_leftCam_Color_DIR;
    std::string KITTI_rightCam_Color_DIR;
    std::string KITTI_IMU_Dir;
    std::string KITTI_Laser_Scanner_Dir;
    std::string KITTI_Cam_Calib , KITTI_IMU_VEL , KITTI_VEL_CAM;
    std::string KITTI_LINK_ROAD;

public:
    DataLoader();
    std::vector<cv::Mat> ReadIMU(std::string imu_filename);
    std::vector<cv::Mat> IMU_data_formatter(std::string line);
    std::vector<cv::Mat> Camera_Calibration(std::string calib_filename,
                                            int row_num , int col_num);
    std::vector<double> ReadTimeStamps(std::string filename);
    cv::Mat Timu2vel();
    cv::Mat Tvel2cam();
    void CameraProjectionMatrix
        (std::vector<cv::Mat>& P , std::vector<cv::Mat>& R) ;
    pcl::PointCloud <pcl::PointXYZ>::Ptr RoadLinkLoader(std::string filename);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ReadPCD(std::string filename );
};

#endif // DATALOADER_H
