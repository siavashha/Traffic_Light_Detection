#ifndef DATABASE_H
#define DATABASE_H

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

typedef boost::geometry::model::d2::point_xy<double> point_2d;
typedef boost::geometry::model::multi_point <point_2d> mPoint_2d;
typedef boost::geometry::model::polygon <point_2d> polygon_2d;
typedef boost::geometry::model::segment <point_2d> segment;
typedef boost::geometry::model::linestring <point_2d> mline_2d;
typedef boost::geometry::model::multi_polygon<polygon_2d> multi_polygon_2D;
typedef boost::geometry::model::multi_linestring<mline_2d> multi_line_2D;

enum RoadType {motorway, trunk, primary, secondary, tertiary , service , residential , footway, cycleway};
enum PointType {traffic_signals, stop, street_lamp, crossing, give_way,motorway_junction};

struct Roads_t
{
    std::vector<int> speedLimit;
    std::vector<int> roadType;
    multi_line_2D roadShape;
};

struct Road_t
{
    int speedLimit;
    int roadType;
    mline_2d roadSegment;
};

struct Points_t
{
    std::vector<int> pointType;
    mPoint_2d pointShape;
};

struct Point_t
{
    int pointType;
    point_2d location;
};

#endif // DATABASE_H
