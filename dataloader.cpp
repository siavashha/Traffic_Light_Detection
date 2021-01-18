//
//  DataLoader.cpp
//  KITTI loader
//
//  Created by Siavish Hosseiny Alamdary on 3/6/14.
//  Copyright (c) 2014 Siavish Hosseiny Alamdary. All rights reserved.
//

#include "DataLoader.h"

// This function recieves the input directory of the KITTI dataset and
// finds the subdirectories of different sensors and transformations
DataLoader::DataLoader()
{

}

std::vector<cv::Mat> DataLoader::ReadIMU(std::string imu_filename)
{
    std::vector<cv::Mat> imu_data;
    std::string line;
    std::ifstream file;
    file.open(imu_filename.c_str() );
    if (file.is_open())
    {
        while(getline(file, line))
        {
            //std::cout << line << std::endl;
            imu_data = IMU_data_formatter(line);
        }
        file.close();
    }
    return imu_data;
}

void DataLoader::CameraProjectionMatrix
    (std::vector<cv::Mat>& P , std::vector<cv::Mat>& R)
{
    // R0
    cv::Mat R00_rect(4,4,CV_64F,cv::Scalar(0));
    R00_rect.at<double>(0,0) = 9.999239e-01;
    R00_rect.at<double>(0,1) = 9.837760e-03;
    R00_rect.at<double>(0,2) = -7.445048e-03;
    R00_rect.at<double>(0,3) = 0.;
    R00_rect.at<double>(1,0) = -9.869795e-03;
    R00_rect.at<double>(1,1) = 9.999421e-01;
    R00_rect.at<double>(1,2) = -4.278459e-03;
    R00_rect.at<double>(1,3) = 0.;
    R00_rect.at<double>(2,0) = 7.402527e-03;
    R00_rect.at<double>(2,1) = 4.351614e-03;
    R00_rect.at<double>(2,2) = 9.999631e-01;
    R00_rect.at<double>(2,3) = 0.;
    R00_rect.at<double>(3,3) = 1.;
    // P0
    cv::Mat P00_rect(3,4,CV_64F,cv::Scalar(0));
    P00_rect.at<double>(0,0) =7.215377e+02;
    P00_rect.at<double>(0,1) =0.000000e+00;
    P00_rect.at<double>(0,2) =6.095593e+02;
    P00_rect.at<double>(0,3) =0.000000e+00;
    P00_rect.at<double>(1,0) =0.000000e+00;
    P00_rect.at<double>(1,1) =7.215377e+02;
    P00_rect.at<double>(1,2) =1.728540e+02;
    P00_rect.at<double>(1,3) =0.000000e+00;
    P00_rect.at<double>(2,0) =0.000000e+00;
    P00_rect.at<double>(2,1) =0.000000e+00;
    P00_rect.at<double>(2,2) =1.000000e+00;
    P00_rect.at<double>(2,3) =0.000000e+00;
    // R1
    cv::Mat R01_rect(4,4,CV_64F,cv::Scalar(0));
    R01_rect.at<double>(0,0) = 9.996878e-01;
    R01_rect.at<double>(0,1) = -8.976826e-03;
    R01_rect.at<double>(0,2) = 2.331651e-02 ;
    R01_rect.at<double>(0,3) = 0.;
    R01_rect.at<double>(1,0) = 8.876121e-03;
    R01_rect.at<double>(1,1) = 9.999508e-01;
    R01_rect.at<double>(1,2) = 4.418952e-03;
    R01_rect.at<double>(1,3) = 0.;
    R01_rect.at<double>(2,0) = -2.335503e-02;
    R01_rect.at<double>(2,1) =  -4.210612e-03;
    R01_rect.at<double>(2,2) = 9.997184e-01;
    R01_rect.at<double>(2,3) = 0.;
    R01_rect.at<double>(3,3) = 1.;
    // P1
    cv::Mat P01_rect(3,4,CV_64F,cv::Scalar(0));
    P01_rect.at<double>(0,0) = 7.215377e+02;
    P01_rect.at<double>(0,1) = 0.000000e+00;
    P01_rect.at<double>(0,2) = 6.095593e+02;
    P01_rect.at<double>(0,3) = -3.875744e+02;
    P01_rect.at<double>(1,0) = 0.000000e+00;
    P01_rect.at<double>(1,1) = 7.215377e+02;
    P01_rect.at<double>(1,2) = 1.728540e+02;
    P01_rect.at<double>(1,3) = 0.000000e+00;
    P01_rect.at<double>(2,0) = 0.000000e+00;
    P01_rect.at<double>(2,1) = 0.000000e+00;
    P01_rect.at<double>(2,2) = 1.000000e+00;
    P01_rect.at<double>(2,3) = 0.000000e+00;
    // R2
    cv::Mat R02_rect(4,4,CV_64F,cv::Scalar(0));
    R02_rect.at<double>(0,0) = 9.998817e-01;
    R02_rect.at<double>(0,1) = 1.511453e-02;
    R02_rect.at<double>(0,2) =  -2.841595e-03 ;
    R02_rect.at<double>(0,3) = 0.;
    R02_rect.at<double>(1,0) = -1.511724e-02;
    R02_rect.at<double>(1,1) = 9.998853e-01;
    R02_rect.at<double>(1,2) = -9.338510e-04;
    R02_rect.at<double>(1,3) = 0.;
    R02_rect.at<double>(2,0) = 2.827154e-03;
    R02_rect.at<double>(2,1) = 9.766976e-04;
    R02_rect.at<double>(2,2) = 9.999955e-01;
    R02_rect.at<double>(2,3) = 0.;
    R02_rect.at<double>(3,3) = 1.;
    // P2
    cv::Mat P02_rect(3,4,CV_64F,cv::Scalar(0));
    P02_rect.at<double>(0,0) = 7.215377e+02;
    P02_rect.at<double>(0,1) = 0.000000e+00;
    P02_rect.at<double>(0,2) = 6.095593e+02;
    P02_rect.at<double>(0,3) = 4.485728e+01;
    P02_rect.at<double>(1,0) = 0.000000e+00;
    P02_rect.at<double>(1,1) = 7.215377e+02;
    P02_rect.at<double>(1,2) = 1.728540e+02;
    P02_rect.at<double>(1,3) = 2.163791e-01;
    P02_rect.at<double>(2,0) = 0.000000e+00;
    P02_rect.at<double>(2,1) = 0.000000e+00;
    P02_rect.at<double>(2,2) = 1.000000e+00;
    P02_rect.at<double>(2,3) = 2.745884e-03;
    // R3
    cv::Mat R03_rect(4,4,CV_64F,cv::Scalar(0));
    R03_rect.at<double>(0,0) = 9.998321e-01;
    R03_rect.at<double>(0,1) = -7.193136e-03;
    R03_rect.at<double>(0,2) =  1.685599e-02;
    R03_rect.at<double>(0,3) = 0.;
    R03_rect.at<double>(1,0) = 7.232804e-03;
    R03_rect.at<double>(1,1) = 9.999712e-01 ;
    R03_rect.at<double>(1,2) = -2.293585e-03;
    R03_rect.at<double>(1,3) = 0.;
    R03_rect.at<double>(2,0) = -1.683901e-02;
    R03_rect.at<double>(2,1) = 2.415116e-03 ;
    R03_rect.at<double>(2,2) = 9.998553e-01;
    R03_rect.at<double>(2,3) = 0.;
    R03_rect.at<double>(3,3) = 1.;
    // P3
    cv::Mat P03_rect(3,4,CV_64F,cv::Scalar(0));
    P03_rect.at<double>(0,0) = 7.215377e+02;
    P03_rect.at<double>(0,1) = 0.000000e+00;
    P03_rect.at<double>(0,2) = 6.095593e+02;
    P03_rect.at<double>(0,3) = -3.395242e+02;
    P03_rect.at<double>(1,0) = 0.000000e+00;
    P03_rect.at<double>(1,1) = 7.215377e+02;
    P03_rect.at<double>(1,2) = 1.728540e+02;
    P03_rect.at<double>(1,3) = 2.199936e+00;
    P03_rect.at<double>(2,0) = 0.000000e+00;
    P03_rect.at<double>(2,1) = 0.000000e+00;
    P03_rect.at<double>(2,2) = 1.000000e+00;
    P03_rect.at<double>(2,3) = 2.729905e-03;
    // P
    P.push_back(P00_rect);
    P.push_back(P01_rect);
    P.push_back(P02_rect);
    P.push_back(P03_rect);
    // R
    R.push_back(R00_rect);
    R.push_back(R01_rect);
    R.push_back(R02_rect);
    R.push_back(R03_rect);
}

std::vector<cv::Mat> DataLoader::IMU_data_formatter(std::string line)
{
    std::vector<cv::Mat> imu_data;
    std::vector<std::string> strs;
    boost::split(strs, line, boost::is_any_of(" "));

    //for (int i = 0 ; i < 3; i++ )
    //std::cout << strs[i] << std::endl;

    // Position, lattitude in Rad, longitude in Rad and h in meters
    cv::Mat pos(3,1,CV_64F,cv::Scalar(0));
    double latti = atof(strs[0].c_str());
    double longi = atof(strs[1].c_str());
    double h = atof(strs[2].c_str());

    pos.at<double>(0,0) = latti; //meridianCurv * (latti - first_latti); //dN
    pos.at<double>(1,0) = longi; //primeVCosined *  (longi - first_longi); //dE
    pos.at<double>(2,0) = h; //h - first_h; //dH
    // Euler angles, roll in Rad [-pi;pi], pitch in Rad [-pi/2;pi/2] and yaw in Rad [-pi;pi]
    cv::Mat Vne(2,1,CV_64F,cv::Scalar(0));
    double Vn = atof(strs[6].c_str());
    double Ve = atof(strs[7].c_str());
    Vne.at<double>(1,0) = Ve;
    Vne.at<double>(0,0) = Vn;
    cv::Mat theta(3,1,CV_64F,cv::Scalar(0));
    double roll = atof(strs[3].c_str());
    double pitch = atof(strs[4].c_str());
    double yaw = atof(strs[5].c_str());
    theta.at<double>(0,0) = roll;
    theta.at<double>(1,0) = pitch;
    theta.at<double>(2,0) = yaw;//(2*M_PI-yaw) - M_PI / 2.;
    // velocity, forward velocity in m/s, leftward velocity in m/s and upward velocity in m/s
    cv::Mat vFLU(3,1,CV_64F,cv::Scalar(0));
    double vF = atof(strs[8].c_str());
    double vL = atof(strs[9].c_str());
    double vU = atof(strs[10].c_str());
    vFLU.at<double>(0,0) = vF;
    vFLU.at<double>(1,0) = vL;
    vFLU.at<double>(2,0) = vU;
    // Angular velocity, forward [rad/s] left[rad/s] up[rad/s]
    cv::Mat omg(3,1,CV_64F,cv::Scalar(0));
    double omgF = atof(strs[20].c_str());
    double omgL = atof(strs[21].c_str());
    double omgU = atof(strs[22].c_str());
    omg.at<double>(0,0) = omgF;
    omg.at<double>(1,0) = omgL;
    omg.at<double>(2,0) = omgU;
    /*******************/
    imu_data.push_back(pos);
    imu_data.push_back(Vne);
    imu_data.push_back(theta);
    imu_data.push_back(vFLU);
    imu_data.push_back(omg);

    return imu_data;
}

std::vector<double> DataLoader::ReadTimeStamps(std::string filename)
{
    std::vector< double > timeStamp;
    std::string line;
    std::ifstream fileTimeStamps;
    const char * cFilename = filename.c_str();
    fileTimeStamps.open(cFilename);
    if (fileTimeStamps.is_open())
    {
        while(getline(fileTimeStamps, line))
        {
            std::vector<std::string> strs;
            boost::split(strs, line, boost::is_any_of(" "));
            std::vector<std::string> strTime;
            boost::split(strTime, strs[1], boost::is_any_of(":"));
            int hour = atoi(strTime[0].c_str());
            int minute = atoi(strTime[1].c_str());
            double second = boost::lexical_cast<double>(strTime[2]);
            timeStamp.push_back(hour * 3600. + minute * 60. + second);
            //std::cout << "hour = " << hour << " minute = " << minute <<  " second = " << second <<std::endl;
            //printf(" it %f \n" , second);
        }
        fileTimeStamps.close();
    }
    return timeStamp;
}

std::vector<cv::Mat> DataLoader::Camera_Calibration(std::string calib_filename ,
                                                    int row_num , int col_num)
{
    std::vector<cv::Mat> camera_calib;
    const char * cKITTI_Cam_Calib = calib_filename.c_str();
    std::ifstream calib_file;
    std::string calib_line;
    calib_file.open(cKITTI_Cam_Calib );
    std::vector<std::string> strs;
    cv::Mat lCalibBW = cv::Mat::zeros(3,3,CV_64F);
    cv::Mat rCalibBW = cv::Mat::zeros(3,3,CV_64F);
    cv::Mat lCalibColor = cv::Mat::zeros(3,3,CV_64F);
    cv::Mat rCalibColor = cv::Mat::zeros(3,3,CV_64F);

    if (calib_file.is_open())
    {
        while(getline(calib_file, calib_line))
        {
            boost::split(strs, calib_line, boost::is_space());
            if (strs[0]!="P_rect_00:" && strs[0]!="P_rect_01:" &&
                    strs[0]!="P_rect_02:" && strs[0]!="P_rect_03:")
                continue;
            else if (strs[0]=="P_rect_00:")
            {
                // The first element of calib_param is ":" and it is not added to matrix
                double lFocal = boost::lexical_cast<double>(strs[1]);
                lCalibBW.at<double>(0,0) = lFocal;
                lCalibBW.at<double>(0,1) = 0.;
                double px = boost::lexical_cast<double>(strs[3]);
                double py = boost::lexical_cast<double>(strs[7]);
                lCalibBW.at<double>(0,2) = -1 * (px - (double)(col_num)/2.);
                lCalibBW.at<double>(1,0) = 0.;
                lCalibBW.at<double>(1,1) = lFocal;
                lCalibBW.at<double>(1,2) = py - (double)(row_num)/2.;
                lCalibBW.at<double>(2,0) = 0.;
                lCalibBW.at<double>(2,1) = 0.;
                lCalibBW.at<double>(2,2) = 1.0;
                camera_calib.push_back(lCalibBW);
            }
            else if (strs[0]=="P_rect_01:")
            {
                // The first element of calib_param is ":" and it is not added to matrix
                double rFocal = boost::lexical_cast<double>(strs[1]);
                rCalibBW.at<double>(0,0) = rFocal;
                rCalibBW.at<double>(0,1) = 0.;
                double px = boost::lexical_cast<double>(strs[3]);
                double py = boost::lexical_cast<double>(strs[7]);
                rCalibBW.at<double>(0,2) = -1 * (px - (double)(col_num)/2.);
                rCalibBW.at<double>(1,0) = 0.;
                rCalibBW.at<double>(1,1) = rFocal;
                rCalibBW.at<double>(1,2) = py - (double)(row_num)/2.;
                rCalibBW.at<double>(2,0) = 0.;
                rCalibBW.at<double>(2,1) = 0.;
                rCalibBW.at<double>(2,2) = 1.0;
                camera_calib.push_back(rCalibBW);
            }
            else if (strs[0]=="P_rect_02:")
            {
                // The first element of calib_param is ":" and it is not added to matrix
                double lFocal = boost::lexical_cast<double>(strs[1]);
                lCalibColor.at<double>(0,0) = lFocal;
                lCalibColor.at<double>(0,1) = 0.;
                double px = boost::lexical_cast<double>(strs[3]);
                double py = boost::lexical_cast<double>(strs[7]);
                lCalibColor.at<double>(0,2) = -1 * (px - (double)(col_num)/2.);
                lCalibColor.at<double>(1,0) = 0.;
                lCalibColor.at<double>(1,1) = lFocal;
                lCalibColor.at<double>(1,2) = py - (double)(row_num)/2.;
                lCalibColor.at<double>(2,0) = 0.;
                lCalibColor.at<double>(2,1) = 0.;
                lCalibColor.at<double>(2,2) = 1.0;
                camera_calib.push_back(lCalibColor);
            }
            else if (strs[0]=="P_rect_03:")
            {
                // The first element of calib_param is ":" and it is not added to matrix
                double lFocal = boost::lexical_cast<double>(strs[1]);
                rCalibColor.at<double>(0,0) = lFocal;
                rCalibColor.at<double>(0,1) = 0.;
                double px = boost::lexical_cast<double>(strs[3]);
                double py = boost::lexical_cast<double>(strs[7]);
                rCalibColor.at<double>(0,2) = -1 * (px - (double)(col_num)/2.);
                rCalibColor.at<double>(1,0) = 0.;
                rCalibColor.at<double>(1,1) = lFocal;
                rCalibColor.at<double>(1,2) = py - (double)(row_num)/2.;
                rCalibColor.at<double>(2,0) = 0.;
                rCalibColor.at<double>(2,1) = 0.;
                rCalibColor.at<double>(2,2) = 1.0;
                camera_calib.push_back(rCalibColor);
            }
        }
        calib_file.close();
    }
    return camera_calib;
}

cv::Mat DataLoader::Timu2vel()
{
    cv::Mat T(4,4,CV_64F, cv::Scalar(0.));
    T.at<double>(0,0) = 9.999976e-01;
    T.at<double>(0,1) = 7.553071e-04;
    T.at<double>(0,2) = -2.035826e-03;
    T.at<double>(1,0) = -7.854027e-04;
    T.at<double>(1,1) = 9.998898e-01;
    T.at<double>(1,2) = -1.482298e-02;
    T.at<double>(2,0) = 2.024406e-03;
    T.at<double>(2,1) = 1.482454e-02;
    T.at<double>(2,2) = 9.998881e-01;
    T.at<double>(0,3) = -8.086759e-01;
    T.at<double>(1,3) = 3.195559e-01;
    T.at<double>(2,3) = -7.997231e-01;
    T.at<double>(3,3) = 1.;
    return T;
}

cv::Mat DataLoader::Tvel2cam()
{
    cv::Mat T(4,4,CV_64F, cv::Scalar(0.));
    T.at<double>(0,0) = 7.533745e-03;
    T.at<double>(0,1) = -9.999714e-01;
    T.at<double>(0,2) = -6.166020e-04;
    T.at<double>(1,0) = 1.480249e-02;
    T.at<double>(1,1) = 7.280733e-04;
    T.at<double>(1,2) = -9.998902e-01;
    T.at<double>(2,0) = 9.998621e-01;
    T.at<double>(2,1) = 7.523790e-03;
    T.at<double>(2,2) = 1.480755e-02;
    T.at<double>(0,3) = -4.069766e-03;
    T.at<double>(1,3) = -7.631618e-02;
    T.at<double>(2,3) = -2.717806e-01;
    T.at<double>(3,3) = 1.;
    return T;
}

pcl::PointCloud <pcl::PointXYZ>::Ptr DataLoader::RoadLinkLoader(std::string filename)
{
    pcl::PointCloud <pcl::PointXYZ>::Ptr llh_road_links
            (new pcl::PointCloud <pcl::PointXYZ>());
    std::string line;
    std::ifstream fileTimeStamps;
    const char * cFilename = filename.c_str();
    fileTimeStamps.open(cFilename);
    if (fileTimeStamps.is_open())
    {
        while(getline(fileTimeStamps, line))
        {
            std::vector<std::string> strs;
            boost::split(strs, line, boost::is_any_of(" "));
            double lati = atof(strs[0].c_str());
            double longi = atof(strs[1].c_str());
            double height_t = atof(strs[2].c_str());
            pcl::PointXYZ p;
            p.y = lati;
            p.x = longi;
            p.z = height_t;
            llh_road_links->points.push_back(p);
        }
        fileTimeStamps.close();
    }
    return llh_road_links;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DataLoader::ReadPCD(std::string filename )
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ> ());
    int32_t num = 1000000;
    float* data = (float*) malloc (num * sizeof (float));
    //pointers
    float *px = data + 0;
    float *py = data + 1;
    float *pz = data + 2;
    float *pr = data + 3;
    // filename
    FILE* stream;
    stream = fopen(filename.c_str(),"rb");
    num = fread(data , sizeof(float) , num , stream )/ 4;

    for (int32_t i = 0; i < num ; i++)
    {
        //float angle = atan2 (*py,*px) * 180 / 3.14159265359;
        //if (std::abs(angle) < 45)
        //{
            pcl::PointXYZ p;
            p.x = *px; p.y = *py; p.z = *pz;// p.intensity = *pr;
            cloud->points.push_back(p);
        //}
            //std::cout << p.z << std::endl;
        px+= 4 ; py += 4; pz +=4; pr+=4;
    }
    fclose(stream);
    free(data);
    return cloud;
}

