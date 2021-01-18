#ifndef GISRETRIEVAL_H
#define GISRETRIEVAL_H
//#include <iostream>
//#include <stdlib.h>
//#include <stdio.h>
//#include <vector>
#include "shapefil.h"
#include "Database.h"
#include "visualizer.h"
#include <boost/foreach.hpp>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

class GISRetrieval
{
public:
    GISRetrieval();
    std::vector<Road_t> Read_Lines (std::string filename , double GPSx , double GPSy);
    std::vector<Point_t> Read_Points (std::string filename , double GPSx , double GPSy , double azimuth);
    double FindDirection(std::vector<Road_t> roads , Point_t pnt);

    //    multi_line_2D Read_Sidewalk
//        (std::string filename , double GPSx , double GPSy);
//    void RoadLinkSelection
//        (double Px , double Py , Roads_t roads , int &speedLimit ,
//         int &roadType , double &dx , double &dy );
//    multi_polygon_2D RoadSpace(Roads_t road , double GPSx , double GPSy);
//    multi_polygon_2D Convert_mline2polygon(multi_line_2D ml);
//    void SaveRoadLinks(std::string s , multi_line_2D roadlink);

};

#endif // GISRETRIEVAL_H
