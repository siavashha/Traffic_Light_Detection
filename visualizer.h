#ifndef VISUALIZER_H
#define VISUALIZER_H
#include <boost/lexical_cast.hpp>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/interactor_style.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "Database.h"
#include "dataset.h"

class Visualizer
{
private:
    int object_counter;

public:
    Visualizer(pcl::visualization::PCLVisualizer& viewer);
    Visualizer(pcl::visualization::PCLVisualizer& viewer , cv::Mat orientation);
    void AddPointCloud(pcl::visualization::PCLVisualizer& viewer,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void AddPointCloud(pcl::visualization::PCLVisualizer& viewer ,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void AddPointCloud(pcl::visualization::PCLVisualizer& viewer ,
                       pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);
    void AddPointCloud(pcl::visualization::PCLVisualizer& viewer ,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ,
                       std::string color);
    void AddPointCloud(pcl::visualization::PCLVisualizer& viewer ,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ,
                       cv::Point3f color );
    void AddPointCloud(pcl::visualization::PCLVisualizer& viewer ,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ,
                       int k);
    void AddPointCloud(pcl::visualization::PCLVisualizer& viewer ,
                       cv::Mat b);
    void ShowCoordinateSystem(pcl::visualization::PCLVisualizer& viewer ,
                       std::vector<cv::Mat> NED);
    void ShowRoadLinks(pcl::visualization::PCLVisualizer& viewer,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr roadlinkcloud);
    void ShowRoadLinks(pcl::visualization::PCLVisualizer& viewer,
                                   multi_line_2D roadlink);
    void ShowRoadLinks(pcl::visualization::PCLVisualizer& viewer,
                      std::vector<std::vector<pcl::PointXYZ> > points);
    void ShowRoadLinks(pcl::visualization::PCLVisualizer& viewer,
                                   std::vector<pcl::PointXYZ> points);
    cv::Point3f GroundColorMix( float x, float min, float max);

    cv::Mat ProjectedPCD_Vis
    (cv::Mat image , const pcl::PointCloud<pcl::PointXYZRGB>::Ptr proj_PCD);

    void FindMinMaxPCD(float& max , float& min, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr proj_PCD);
    void AddPointCloud(pcl::visualization::PCLVisualizer& viewer ,
                                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud ,
                                   pcl::PointCloud<pcl::Normal>::Ptr normals);
    void AddPolygonMesh(pcl::visualization::PCLVisualizer& viewer ,
                                   pcl::PolygonMesh mesh);


    void AddPointCloud(pcl::visualization::PCLVisualizer& viewer ,
                                   std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters);


};

#endif // VISUALIZER_H
