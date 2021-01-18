#include "gisretrieval.h"

GISRetrieval::GISRetrieval()
{
}

template <typename T, typename F>
void read_shapefile(const std::string& filename, std::vector<T>& shapes, F functor)
{
    try
    {
        SHPHandle handle = SHPOpen(filename.c_str(), "rb");
        if (handle <= 0)
            throw std::string("File " + filename + " not found");
        int nShapeType, nEntities;
        double adfMinBound[4], adfMaxBound[4];
        SHPGetInfo(handle, &nEntities, &nShapeType, adfMinBound, adfMaxBound );
        std::cout << " ShapeFileType = " << nShapeType << std::endl;
        std::cout << " nEntities = " << nEntities << std::endl;
        for (int i = 0; i < nEntities; i++)
        {
            SHPObject* psShape = SHPReadObject(handle, i );
            // Read only polygon and line
            T shape;
            functor(psShape, shape);
            shapes.push_back(shape);
            SHPDestroyObject( psShape );
        }
        SHPClose(handle);
    }
    catch(const std::string& s)
    {
        throw s;
    }
    catch(...)
    {
        throw std::string("Other exception");
    }
}

template <typename T>
void convert(SHPObject* psShape, T& shape)
{
    double* x = psShape->padfX;
    double* y = psShape->padfY;
    //std::cout << "x:" << *x << " y: " << *y << " v: " << psShape->nVertices << std::endl;
    for (int v = 0; v < psShape->nVertices; v++)
    {
        typename boost::geometry::point_type<T>::type point;
        boost::geometry::assign_values(point, x[v], y[v]);
        boost::geometry::append(shape, point);
    }
}

//template <typename P>
//multi_polygon_2D Buffering(P& p , double buffer_distance)
//{
//    // Declare strategies
//    const int points_per_circle = 36;
//    boost::geometry::strategy::buffer::distance_symmetric<double>
//            distance_strategy(buffer_distance);
//    boost::geometry::strategy::buffer::join_round
//            join_strategy(points_per_circle);
//    boost::geometry::strategy::buffer::end_round
//            end_strategy(points_per_circle);
//    boost::geometry::strategy::buffer::point_circle
//            circle_strategy(points_per_circle);
//    boost::geometry::strategy::buffer::side_straight
//            side_strategy;
//    // Declare output
//    multi_polygon_2D regionOfInterest;
//    // Create the buffer of a linestring
//    boost::geometry::buffer(p, regionOfInterest,
//                            distance_strategy, side_strategy,
//                            join_strategy, end_strategy, circle_strategy);
//    return regionOfInterest;
//}

//mline_2d addLines(mline_2d a, mline_2d b)
//{
//    mline_2d ab;
//    for (int i = 0 ; i < a.size() ; i++)
//        ab.push_back(a[i]);
//    for (int j = 0 ; j < b.size() ; j++)
//        ab.push_back(b[j]);
//    return ab;
//}

//multi_polygon_2D GISRetrieval::Convert_mline2polygon(multi_line_2D ml)
//{
//    point_2d firstPoint;
//    multi_polygon_2D mPoly;
//    polygon_2d poly;
//    bool first = true;
//    //std::cout << "ml " << ml.size() << std::endl;
//    while (ml.size() != 1)
//    {
//        int i = 0;
//        std::cout << "ml " << ml.size() << std::endl;
//        double min = 100000;
//        int index;
//        bool rev;
//        mline_2d li = ml[i];
//        //std::vector a;
//        //a.erase()
//        for (int j = i + 1; j < ml.size(); j++)
//        {
//            mline_2d lj = ml[j];
//            double dist1 = boost::geometry::distance(li[li.size()-1],lj[0]);
//            if (dist1 < min)
//            {
//                min = dist1;
//                rev = false;
//                index = j;
//            }
//            double dist2 = boost::geometry::distance(li[li.size()-1],lj[lj.size()-1]);
//            if (dist2 < min)
//            {
//                min = dist2;
//                rev = true;
//                index = j;
//            }
//        }
//        mline_2d lij;
//        mline_2d lj = ml[index];
//        if (rev)
//            boost::geometry::reverse(lj);
//        lij = addLines(li , lj);
//        ml.push_back(lij);
//        ml.erase(ml.begin() + index - i);
//        ml.erase(ml.begin() + i);
//     }
//    mline_2d l = ml[0];
//    for (int i = 0 ; i < l.size(); i++)
//        boost::geometry::append(poly , l[i]);
//    boost::geometry::correct(poly);
//    std::cout << boost::geometry::dsv(poly) << std::endl;
//    mPoly.push_back(poly);
//    return mPoly;
//}

//multi_polygon_2D GISRetrieval::RoadSpace(Roads_t road , double GPSx , double GPSy)
//{
//    point_2d poi;
//    poi.x(GPSy);
//    poi.y(GPSx);
//    const double buffer_distance = 5.5 / 6371000.0 * (180 / M_PI);
//    multi_polygon_2D roi = Buffering(road.roadShape ,  buffer_distance);
//    multi_polygon_2D roadSpace;
//    for (int i = 0 ; i < roi.size() ; i++)
//        if (boost::geometry::within(poi , roi[i]))
//            roadSpace.push_back(roi[i]);
//    return roadSpace;
//}

std::vector<Road_t> GISRetrieval::Read_Lines
    (std::string filename , double GPSx , double GPSy)
{
    std::vector<Road_t> roads;
    point_2d poi;
    poi.x(GPSy);
    poi.y(GPSx);
    // create database
    std::string dbf_filename = filename.substr
            (0 , filename.length() - 3 ) + "dbf";
    DBFHandle dbf = DBFOpen(dbf_filename.c_str(), "rb");
    // read roads
    std::vector<mline_2d> lines;
    read_shapefile(filename, lines, convert<mline_2d>);
    // make the buffer
    const double buffer_distance = 50.0/ 6371000.0 * (180 / M_PI);
    for (int i = 0 ; i < lines.size() ; i++)
    {
        mline_2d line = lines[i];
        if ((boost::geometry::distance( poi , line[0] ) < buffer_distance) ||
                (boost::geometry::distance( poi , line[1] ) < buffer_distance))
        {
            Road_t road;
            const char * temp_str = DBFReadStringAttribute(dbf, i, 3);
            if (strcmp(temp_str,"motorway")==0)
                road.roadType = 0;
            else if (strcmp(temp_str,"trunk")==0)
                road.roadType = 1;
            else if (strcmp(temp_str,"primary")==0)
                road.roadType = 2;
            else if (strcmp(temp_str,"secondary")==0)
                road.roadType = 3;
            else if (strcmp(temp_str,"tertiary")==0)
                road.roadType = 4;
            else if (strcmp(temp_str,"service")==0)
                road.roadType = 5;
            else if (strcmp(temp_str,"residential")==0)
                road.roadType = 6;
            else if (strcmp(temp_str,"footway")==0)
                road.roadType = 7;
            else if (strcmp(temp_str,"cycleway")==0)
                road.roadType = 8;
            else
                continue;
            int temp_int = DBFReadIntegerAttribute(dbf, i, 6);
            road.speedLimit = temp_int;
            road.roadSegment = line;
            roads.push_back(road);
        }
    }
    return roads;
}

std::vector<Point_t> GISRetrieval::Read_Points
    (std::string filename , double GPSx , double GPSy , double azimuth)
{
    std::vector<Point_t> pts;
    point_2d poi;
    poi.x(GPSy);
    poi.y(GPSx);
    // create database
    std::string dbf_filename = filename.substr
            (0 , filename.length() - 3 ) + "dbf";
    DBFHandle dbf = DBFOpen(dbf_filename.c_str(), "rb");
    // read shapefile
    std::vector<mPoint_2d> points;
    read_shapefile(filename, points, convert<mPoint_2d>);
    // make the buffer
    const double buffer_distance = 50.0/ 6371000.0 * (180 / M_PI);
    for (int i = 0 ; i < points.size() ; i++)
    {
        mPoint_2d mpoint = points[i];
        point_2d point = mpoint[0];
        //point_2d point = point_s[0];
//        std::cout << " poi_x " << poi.x() << " poi_y " << poi.y() <<
//                     " point_x " << point.x() << " point_y " << point.y() << std::endl;

//        std::cout << " distance: " << boost::geometry::distance( poi , point ) <<
//                     " buffer: " << buffer_distance << std::endl;
        double angle = atan2(poi.y()-point.y(),poi.x()-point.x()) + M_PI;
        if ((boost::geometry::distance( poi , point ) < buffer_distance) && (std::abs(angle - azimuth) < M_PI / 4) )
        {

//            if (angle < 0)
//                angle = angle + 2 * M_PI;
//            if (angle > 2 * M_PI)
//                angle = angle - 2 * M_PI;
            std::cout << " dAngle " <<  angle << ":" << azimuth << std::endl; ;
            //std::cout << " dAngle " <<  (atan2(poi.y()-point.y(),poi.x()-point.x())- azimuth) % (2*M_PI) << std::endl; ;

            Point_t pnt;
            const char * temp_str = DBFReadStringAttribute(dbf, i, 3);
            if (strcmp(temp_str,"traffic_signals")==0)
                pnt.pointType = 0;
            else if (strcmp(temp_str,"stop")==0)
                pnt.pointType = 1;
            else if (strcmp(temp_str,"street_lamp")==0)
                pnt.pointType = 2;
            else if (strcmp(temp_str,"crossing")==0)
                pnt.pointType = 3;
            else if (strcmp(temp_str,"give_way")==0)
                pnt.pointType = 4;
            else if (strcmp(temp_str,"motorway_junction")==0)
                pnt.pointType = 5;
            pnt.location = point;
            pts.push_back(pnt);
        }
    }
    return pts;
}

double FindDistanceToSegment(double x1, double y1, double x2, double y2, double pointX, double pointY)
{
    double diffX = x2 - x1;
    float diffY = y2 - y1;
    if ((diffX == 0) && (diffY == 0))
    {
        diffX = pointX - x1;
        diffY = pointY - y1;
        return sqrt(diffX * diffX + diffY * diffY);
    }

    float t = ((pointX - x1) * diffX + (pointY - y1) * diffY) / (diffX * diffX + diffY * diffY);

    if (t < 0)
    {
        //point is nearest to the first point i.e x1 and y1
        diffX = pointX - x1;
        diffY = pointY - y1;
    }
    else if (t > 1)
    {
        //point is nearest to the end point i.e x2 and y2
        diffX = pointX - x2;
        diffY = pointY - y2;
    }
    else
    {
        //if perpendicular line intersect the line segment.
        diffX = pointX - (x1 + t * diffX);
        diffY = pointY - (y1 + t * diffY);
    }

    //returning shortest distance
    return sqrt(diffX * diffX + diffY * diffY);
}


double GISRetrieval::FindDirection(std::vector<Road_t> roads , Point_t pnt)
{
    double min_dist = INFINITY;
    for (int i = 0 ; i < roads.size() ; i ++)
    {
        Road_t road = roads[i];
        for (int j = 0 ; j < road.roadSegment.size() - 1; j++)
        {
            segment seg;
            seg.first = road.roadSegment[j];
            seg.second = road.roadSegment[j+1];
            double dist = FindDistanceToSegment(seg.first.x() , seg.first.y() ,
                   seg.second.x() , seg.second.y() , pnt.location.x(), pnt.location.y());
            //double dist = boost::geometry::distance(pnt , seg);
            // closest line to the traffic light
            if (dist < min_dist)
            {
                min_dist = dist;

                double angle = atan2(seg.first.y()-seg.second.y(),seg.first.x()-seg.second.x());
            }
        }
    }
}



//void GISRetrieval::RoadLinkSelection
//    (double Px , double Py , Roads_t roads, int &speedLimit , int &roadType , double &dx , double &dy )
//{
//    point_2d poi;
//    poi.x(Py);
//    poi.y(Px);
//    int i = -1;
//    double min = 100000.;
//    mline_2d closestLine;
//    BOOST_FOREACH(mline_2d segments, roads.roadShape)
//    {
//        i++;
//        double dist = boost::geometry::distance( poi , segments );
//        if (dist < min)
//        {
//            closestLine = segments;
//            speedLimit = roads.speedLimit[i];
//            roadType = roads.roadType[i];
//            min = dist;
//        }
//    }
//    min = 100000.;
//    i = -1;
//    segment seg;
//    BOOST_FOREACH(point_2d point, closestLine)
//    {
//        i++;
//        if (i == 0)
//        {
//            seg.first = point;
//            continue;
//        }
//        seg.second = point;
//        double dist = boost::geometry::distance( poi , seg );
//        if (dist < min)
//        {
//            point_2d p1 = seg.first;
//            point_2d p2 = seg.second;

//            dx = p1.x() - p2.x();
//            dy = p1.y() - p2.y();
//            double length = std::sqrt(dx * dx + dy * dy);
//            dx = dx / length;
//            dy = dy / length;
//            min = dist;
//        }
//        seg.first = point;
//    }
//    //roads.roadShape[0].x();
//}

//void GISRetrieval::SaveRoadLinks(std::string s , multi_line_2D roadlink)
//{
//    int i = 0;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr road_cloud
//         (new pcl::PointCloud<pcl::PointXYZ>());

//    std::vector<std::vector<pcl::PointXYZ> > segs;
//    BOOST_FOREACH(mline_2d segments, roadlink)
//    {
//        std::vector<pcl::PointXYZ> seg;
//        i++;
//        //std::cout << i << ":" << segments[0].x() << std::endl;
//        pcl::PointXYZ p,q;
//        p.x = segments[0].x();
//        p.y = segments[0].y();
//        p.z = 0;
//        q.x = segments[1].x();
//        q.y = segments[1].y();
//        q.z = 0;
//        seg.push_back(p);
//        seg.push_back(q);
//        segs.push_back(seg);
//    }
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer
//            (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//    Visualizer* viz = new Visualizer(*viewer);
//    viz->ShowRoadLinks(*viewer,segs);
//    //pcl::io::savePCDFileASCII ("/home/sia/Documents/code/TrafficLight_v1/road_cloud.pcd", *road_cloud);
//}
