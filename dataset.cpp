#include "dataset.h"

Dataset::Dataset(std::string KITTI_DIR)
{
    isReferenced = false;
    roadLink_ENU = pcl::PointCloud<pcl::PointXYZ>::Ptr ( new pcl::PointCloud <pcl::PointXYZ>());
    // camera data
    std::string KITTI_leftCam_DIR = KITTI_DIR + "image_00";
    std::string KITTI_rightCam_DIR = KITTI_DIR + "image_01";
    std::string KITTI_leftCam_Color_DIR = KITTI_DIR + "image_02";
    std::string KITTI_rightCam_Color_DIR = KITTI_DIR + "image_03";
    // IMU data
    std::string KITTI_IMU_Dir = KITTI_DIR + "oxts/data/";
    // Laser Scanner data
    std::string KITTI_Laser_Scanner_Dir = KITTI_DIR + "velodyne_points";
    // calibration files
    std::string KITTI_Cam_Calib = KITTI_DIR + "calib_cam_to_cam.txt";
    std::string KITTI_IMU_VEL = KITTI_DIR + "calib_imu_to_velo.txt";
    std::string KITTI_VEL_CAM = KITTI_DIR + "calib_velo_to_cam.txt";
    // road link
    std::string KITTI_LINK_ROAD =
            + "RoadLink.txt";

    int i = 0;
    for (boost::filesystem::directory_iterator itr(KITTI_IMU_Dir);
         itr!=boost::filesystem::directory_iterator() ; ++itr )
    {
        // reading the image seqeuces
        std::ostringstream convert;
        convert << i;
        std::string firstDigits = convert.str();
        std::string KITTI_filename;
        if (firstDigits.length() == 1)
            KITTI_filename = "000000000" + firstDigits ;
        if (firstDigits.length() == 2)
            KITTI_filename = "00000000" + firstDigits ;
        if (firstDigits.length() == 3)
            KITTI_filename = "0000000" + firstDigits ;
        if (firstDigits.length()>3 || firstDigits.length()==0)
            std::fprintf(stderr,"error in naming");

        // reading camera files
        std::string KITTI_leftImage_BW = KITTI_leftCam_DIR + "/data/" + KITTI_filename + ".png";
        std::string KITTI_rightImage_BW = KITTI_rightCam_DIR + "/data/" + KITTI_filename + ".png";
        std::string KITTI_leftImage_Color = KITTI_leftCam_Color_DIR + "/data/" + KITTI_filename + ".png";
        std::string KITTI_rightImage_Color = KITTI_rightCam_Color_DIR + "/data/" + KITTI_filename + ".png";
        // reading laser scanner and IMU
        std::string KITTI_laserScanner = KITTI_Laser_Scanner_Dir + "/data/" + KITTI_filename + ".bin";
        std::string KITTI_IMU_filename = KITTI_IMU_Dir + KITTI_filename + ".txt";
        roadLink_filename = KITTI_LINK_ROAD;
        calib_filename = KITTI_Cam_Calib;
        lImages_bw_list.push_back(KITTI_leftImage_BW);
        rImages_bw_list.push_back(KITTI_rightImage_BW);
        lImages_color_list.push_back(KITTI_leftImage_Color);
        rImages_color_list.push_back(KITTI_rightImage_Color);
        laserScanner_list.push_back(KITTI_laserScanner);
        imu_list.push_back(KITTI_IMU_filename);
        i++;
    }
}

sensor_data Dataset::LoadData(int epoch)
{   
    sensor_data data;
    data.cloud = ReadPCD(laserScanner_list[epoch]);
    std::vector<cv::Mat> imu_data = ReadIMU(imu_list[epoch]);
    //std::vector<cv::Mat> calib = Camera_Calibration(std::string calib_filename);
    data.imu_pos_llh = imu_data[0];
    data.imu_vel_NE = imu_data[1];
    data.imu_angle_RPY = imu_data[2];
    data.imu_vel_FLU = imu_data[3];
    data.imu_AngularVel_FLU = imu_data[4];

    cv::Mat mTimu2vel = Timu2vel();
    data.Timu2vel = mTimu2vel;

    cv::Mat mTvel2cam = Tvel2cam();
    data.Tvel2cam = mTvel2cam;

    std::vector<cv::Mat> P;
    std::vector<cv::Mat> R;
    CameraProjectionMatrix(P , R);

    data.lImgBW_projection = P[0];
    data.rImgBW_projection = P[1];
    data.lImgColor_projection = P[2];
    data.rImgColor_projection = P[3];

    data.lImgBW_rectification = R[0];
    data.rImgBW_rectification = R[1];
    data.lImgColor_rectification = R[2];
    data.rImgColor_rectification = R[3];

    data.lImgBW = cv::imread(lImages_bw_list[epoch]);
    data.rImgBW = cv::imread(rImages_bw_list[epoch]);
    data.lImgColor = cv::imread(lImages_color_list[epoch]);
    data.rImgColor = cv::imread(rImages_color_list[epoch]);

    std::vector<cv::Mat> camera_calib = Camera_Calibration(calib_filename,
                                                           data.lImgColor.cols , data.lImgColor.rows );
    data.lImgBW_calib = camera_calib[0];
    data.rImgBW_calib = camera_calib[1];
    data.lImgColor_calib = camera_calib[2];
    data.rImgColor_calib = camera_calib[3];

    if (!isReferenced)
    {
        latti_R = data.imu_pos_llh.at<double>(0,0);
        longi_R = data.imu_pos_llh.at<double>(1,0);
        h_R = data.imu_pos_llh.at<double>(2,0);
        MNcurvature();
        pcl::PointCloud <pcl::PointXYZ>::Ptr roadLink_llh =
                RoadLinkLoader(roadLink_filename);
        for (int i = 0 ; i < roadLink_llh->size() ; i++)
        {
            pcl::PointXYZ road_point_llh = roadLink_llh->points[i];

            double N_link = meridianCurv * DEG2RAD(road_point_llh.y - latti_R);
            double E_link = primeVCosined * DEG2RAD(road_point_llh.x - longi_R);
            double U_link = -1.73;
            pcl::PointXYZ road_point_ENU;
            road_point_ENU.x = E_link;
            road_point_ENU.y = N_link;
            road_point_ENU.z = U_link;
            roadLink_ENU->points.push_back(road_point_ENU);
        }
        data.enu_road_links = roadLink_ENU;
        isReferenced = true;
    }
    cv::Mat T_Nav = T_inNav(data.imu_pos_llh , data.imu_angle_RPY);
    data.Timu2nav = T_Nav;
    return data;
}

//pcl::PointCloud <pcl::PointXYZ>::Ptr Dataset::conver2Origin
//        (multi_line_2D roadLink_llh)
//{
//    pcl::PointCloud <pcl::PointXYZ>::Ptr roadLink_ENU
//            (new pcl::PointCloud <pcl::PointXYZ>());
//    BOOST_FOREACH(mline_2d segment, roadLink_llh)
//    {
//        for (int i = 0; i < 2; i++)
//        {
//            point_2d p = segment[i];
//            std::cout << "P.x = " << p.x() << " lambda = " << longi_R << std::endl;
//            double N_link = meridianCurv * DEG2RAD(p.y() - latti_R);
//            double E_link = primeVCosined * DEG2RAD(p.x() - longi_R);
//            double U_link = -1.73;
//            pcl::PointXYZ road_point_ENU;
//            road_point_ENU.x = E_link;
//            road_point_ENU.y = N_link;
//            road_point_ENU.z = U_link;
//            roadLink_ENU->points.push_back(road_point_ENU);
//        }
//    }
//    return roadLink_ENU;
//}


std::vector<std::vector<pcl::PointXYZ > > Dataset::conver2Origin
(multi_line_2D roadLink_llh)
{
    std::vector< std::vector<pcl::PointXYZ> >points;
    BOOST_FOREACH(mline_2d road, roadLink_llh)
    {
        std::vector<pcl::PointXYZ> point;
        for (int i = 0 ; i < road.size() ; i++)
        {
            //std::cout << "segmentsSize = " << road[i].x() << std::endl;
            //mline_2d roadSeg = road[i];
            //for (int j = 0; j < 2; j++)
            //{
            point_2d p = road[i];
            //std::cout << "P.x = " << p.x() << " lambda = " << longi_R << std::endl;
            double N_link = meridianCurv * DEG2RAD(p.y() - latti_R);
            double E_link = primeVCosined * DEG2RAD(p.x() - longi_R);
            double U_link = -1.73;
            pcl::PointXYZ road_point_ENU;
            road_point_ENU.x = E_link;
            road_point_ENU.y = N_link;
            road_point_ENU.z = U_link;
            //std::cout << E_link << ":" << N_link << std::endl;
            //roadLink_ENU->points.push_back(road_point_ENU);
            //}
            point.push_back(road_point_ENU);
        }
        points.push_back(point);
    }
    return points;
}

//std::vector<std::vector<pcl::PointXYZ > > Dataset::conver2Origin
//(multi_polygon_2D polygon_llh)
//{
//    std::vector< std::vector<pcl::PointXYZ> >points;
//    BOOST_FOREACH(polygon_2d poly, polygon_llh)
//    {
//        std::cout << "poly.outer().size()" << poly.outer().size() << std::endl;
//        std::vector<pcl::PointXYZ> point;
//        for (int i = 0 ; i < poly.outer().size() ; i++)
//        {
//            //std::cout << "segmentsSize = " << road[i].x() << std::endl;
//            //mline_2d roadSeg = road[i];
//            //for (int j = 0; j < 2; j++)
//            //{
//            point_2d p = poly.outer()[i];
//            //std::cout << "P.x = " << p.x() << " lambda = " << longi_R << std::endl;
//            double N_link = meridianCurv * DEG2RAD(p.y() - latti_R);
//            double E_link = primeVCosined * DEG2RAD(p.x() - longi_R);
//            double U_link = -1.73;
//            pcl::PointXYZ road_point_ENU;
//            road_point_ENU.x = E_link + 4.5;
//            road_point_ENU.y = N_link - 0.0;
//            road_point_ENU.z = U_link;
//            //std::cout << E_link << ":" << N_link << std::endl;
//            //roadLink_ENU->points.push_back(road_point_ENU);
//            //}
//            point.push_back(road_point_ENU);
//        }
//        points.push_back(point);
//    }
//    return points;
//}

pcl::PointCloud<pcl::PointXYZ >::Ptr Dataset::conver2Origin
(multi_polygon_2D polygon_llh)
{
    pcl::PointCloud<pcl::PointXYZ >::Ptr roadLinkCloud
            ( new pcl::PointCloud<pcl::PointXYZ >() );
    std::vector< std::vector<pcl::PointXYZ> >points;
    BOOST_FOREACH(polygon_2d poly, polygon_llh)
    {
        for (int i = 0 ; i < poly.outer().size() ; i++)
        {
            point_2d p = poly.outer()[i];
            //std::cout << "P.x = " << p.x() << " lambda = " << longi_R << std::endl;
            double N_link = meridianCurv * DEG2RAD(p.y() - latti_R);
            double E_link = primeVCosined * DEG2RAD(p.x() - longi_R);
            double U_link = -1.73;
            pcl::PointXYZ road_point_ENU;
            road_point_ENU.x = E_link + 4.5;
            road_point_ENU.y = N_link - 0.0;
            road_point_ENU.z = U_link;
            roadLinkCloud->points.push_back(road_point_ENU);
        }
    }
    return roadLinkCloud;
}

std::vector<pcl::PointXYZ> Dataset::conver2Origin
(mPoint_2d pointLink_llh)
{
    std::vector<pcl::PointXYZ> points;
    BOOST_FOREACH(point_2d point, pointLink_llh)
    {
        //std::cout << "P.x = " << p.x() << " lambda = " << longi_R << std::endl;
        double N_link = meridianCurv * DEG2RAD(point.y() - latti_R);
        double E_link = primeVCosined * DEG2RAD(point.x() - longi_R);
        double U_link = -1.73;
        pcl::PointXYZ road_point_ENU;
        road_point_ENU.x = E_link;
        road_point_ENU.y = N_link;
        road_point_ENU.z = U_link;
        points.push_back(road_point_ENU);
    }
    return points;
}

cv::Mat R3(double th)
{
    cv::Mat rot_z(3,3,CV_64F,cv::Scalar(0.));
    rot_z.at<double>(0,0) = cos(th);
    rot_z.at<double>(0,1) = sin(th);
    rot_z.at<double>(0,2) = 0;
    rot_z.at<double>(1,0) = -sin(th);
    rot_z.at<double>(1,1) = cos(th);
    rot_z.at<double>(1,2) = 0;
    rot_z.at<double>(2,0) = 0;
    rot_z.at<double>(2,1) = 0;
    rot_z.at<double>(2,2) = 1.;
    return rot_z;
}

cv::Mat R2(double th)
{
    cv::Mat rot_y(3,3,CV_64F,cv::Scalar(0.));
    rot_y.at<double>(0,0) = cos(th);
    rot_y.at<double>(0,1) = 0;
    rot_y.at<double>(0,2) = -sin(th);
    rot_y.at<double>(1,0) = 0;
    rot_y.at<double>(1,1) = 1.;
    rot_y.at<double>(1,2) = 0;
    rot_y.at<double>(2,0) = sin(th);
    rot_y.at<double>(2,1) = 0;
    rot_y.at<double>(2,2) = cos(th);
    return rot_y;
}

cv::Mat R1(double th)
{
    cv::Mat rot_x(3,3,CV_64F,cv::Scalar(0.));
    rot_x.at<double>(0,0) = 1.;
    rot_x.at<double>(0,1) = 0;
    rot_x.at<double>(0,2) = 0;
    rot_x.at<double>(1,0) = 0;
    rot_x.at<double>(1,1) = cos(th);
    rot_x.at<double>(1,2) = sin(th);
    rot_x.at<double>(2,0) = 0;
    rot_x.at<double>(2,1) = -sin(th);
    rot_x.at<double>(2,2) = cos(th);
    return rot_x;
}


void Dataset::MNcurvature()
{
    // translation reference
    double simeMajorAxisOfEarth = 6378137;
    double flattening = 1 / 298.257222101;
    double e2 = 2 * flattening -flattening * flattening;
    primeVCosined = simeMajorAxisOfEarth / std::sqrt(1- e2 * sin(DEG2RAD(latti_R)) *
                                                     sin(DEG2RAD(latti_R))) ; // N * cos (phi)
    primeVCosined = primeVCosined * cos(DEG2RAD(latti_R));
    meridianCurv = simeMajorAxisOfEarth * (1 - e2) /
            ( pow(std::sqrt(
                      1 - e2 * sin(DEG2RAD(latti_R)) * sin(DEG2RAD(latti_R))
                      ),3.)); // M
}

cv::Mat Dataset::T_inNav(cv::Mat llh , cv::Mat angle_RPY)
{
    double latti = llh.at<double>(0,0);
    double longi = llh.at<double>(1,0);
    double h = llh.at<double>(2,0);

    double N = meridianCurv * DEG2RAD(latti - latti_R);
    double E = primeVCosined * DEG2RAD(longi - longi_R);
    double U = h - h_R;

    // Jekeli 2000 p.26 eq. 1.91 NED
    double roll = angle_RPY.at<double>(0,0);
    double pitch = angle_RPY.at<double>(1,0);
    double yaw = angle_RPY.at<double>(2,0);
    cv::Mat rot =  R3(-yaw) * R2(-pitch) * R1(-roll);
    cv::Mat trans = (cv::Mat_<double>(3,1) << E , N , U );
    cv::Mat aux = (cv::Mat_<double> (1,4) << 0 , 0 , 0 , 1 );
    cv::Mat T_34;
    cv::hconcat(rot , trans , T_34);
    cv::Mat Timu2Nav;
    cv::vconcat(T_34 , aux , Timu2Nav);
    return Timu2Nav;
}
