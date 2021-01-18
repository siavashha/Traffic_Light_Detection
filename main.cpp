#include <QCoreApplication>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include "gisretrieval.h"
#include "dataset.h"
#include "SensorsData.h"
#include "Database.h"
#include "trafficligthdetection.h"
//#include "visualizer.h"
//#include "shplearning.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    //
    std::string roads_filename_OSM =
            "/home/sia/Documents/Data/OpenStreetMap/Karlsruhe-shp/shape/roads.shp";
    std::string roads_filename_Karlsruhe =
            "/home/sia/Documents/Data/Karlsruhe/sidewalk.shp";
    std::string points_filename_OSM =
            "/home/sia/Documents/Data/OpenStreetMap/Karlsruhe-shp/shape/points.shp";
    // KITTI dataset directory
    std::string  KITTI_DIR =  "/home/sia/Documents/Data/2011_09_26/2011_09_26_drive_0011_sync/";
    GISRetrieval* gis = new GISRetrieval();
    //Data is loaded in vectors and matrices
    Dataset* datasetReader = new Dataset(KITTI_DIR);
    // traffic light detection
    TrafficLigthDetection* trafficDetector = new TrafficLigthDetection();
    int i = 90;
    sensor_data data = datasetReader->LoadData(i);
    trafficDetector->Detector(data.lImgColor);
    double GPSx = data.imu_pos_llh.at<double>(0,0);
    double GPSy = data.imu_pos_llh.at<double>(1,0);
    double tetha = data.imu_angle_RPY.at<double>(2,0);

    std::cout << " latitude: " << GPSx << " longitude: " << GPSy << std::endl;
    // roadLink
    std::vector<Road_t> roads;
    roads = gis->Read_Lines(roads_filename_OSM , GPSx , GPSy);
    // traffic lights
    std::vector<Point_t> pts;
    pts = gis->Read_Points(points_filename_OSM , GPSx,GPSy , tetha);
    //



    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);

    cv::imwrite("lcolor.png" , data.lImgColor , compression_params);

    pcl::PointCloud<pcl::PointXYZ>::Ptr platform_cloud
         (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointXYZ p; p.x = GPSy; p.y = GPSx; p.z = 0;
    platform_cloud->push_back(p);
    double dN_meter = data.imu_vel_NE.at<double>(0,0) * 0.1;
    double dE_meter = data.imu_vel_NE.at<double>(1,0) * 0.1;
    double dN = (1./ 6371000.0) * (180 / M_PI) * dN_meter;
    double dE = (1./ 6371000.0) * (180 / M_PI) * dE_meter;
    //pcl::PointXYZ pNext; pNext.x = p.x + dN; pNext.y = p.y + dE; pNext.z = 0;
    pcl::PointXYZ pNext; pNext.x = p.x + (1./ 6371000.0) * (180 / M_PI) * cos(tetha);
    pNext.y = p.y + (1./ 6371000.0) * (180 / M_PI) * sin(tetha); pNext.z = 0;
    // Find traffic light direction
    double angle = FindDirection(std::vector<Road_t> roads , Point_t pnt);

    pcl::PointCloud<pcl::PointXYZ>::Ptr traffic_cloud
         (new pcl::PointCloud<pcl::PointXYZ>());
    for (int i = 0 ; i < pts.size() ;  i++)
    {
        Point_t pnt = pts[i];
        pcl::PointXYZ p;
        if (pnt.pointType == 0)
        {
            p.x = pnt.location.x();
            p.y = pnt.location.y();
            p.z = 0;
            traffic_cloud->push_back(p);
        }
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer
            (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    for (int i = 0; i < roads.size() ; i++)
    {
        Road_t r  = roads[i];
        for (int j = 0 ; j < r.roadSegment.size() - 1; j++)
        {
        pcl::PointXYZ p1; p1.x = r.roadSegment[j].x(); p1.y = r.roadSegment[j].y(); p1.z = 0;
        pcl::PointXYZ p2; p2.x = r.roadSegment[j+1].x(); p2.y = r.roadSegment[j+1].y(); p2.z = 0;
        std::string ss_line;
        ss_line = "roadlink" + boost::lexical_cast<std::string>(i)+"_"+boost::lexical_cast<std::string>(j);
        viewer->addLine<pcl::PointXYZ, pcl::PointXYZ>
                (p1, p2 ,
                 255, 255, 0, ss_line);
        }
    }

    viewer->addLine<pcl::PointXYZ, pcl::PointXYZ>
            (p, pNext ,
             255, 0, 255, "direction");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            handler (traffic_cloud, 0, 0,255);
    viewer->addPointCloud(traffic_cloud, handler , "traffic_cloud");
    viewer->setPointCloudRenderingProperties
            (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "traffic_cloud");
//    viewer->setPointCloudRenderingProperties
//            (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 15, "traffic_cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            handler1 (platform_cloud, 0, 255,255);
    viewer->addPointCloud(platform_cloud, handler1 , "platform_cloud");
    viewer->setPointCloudRenderingProperties
            (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "platform_cloud");

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (1000));
    }
    viewer->close();

    return a.exec();
}
