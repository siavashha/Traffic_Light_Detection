#include "visualizer.h"

Visualizer::Visualizer(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.addCoordinateSystem();
    viewer.setSize (640,480);  // Visualiser window size
    //viewer.updateCamera();
    viewer.setBackgroundColor (255, 255, 255);
    object_counter=0;
}

Visualizer::Visualizer(pcl::visualization::PCLVisualizer& viewer , cv::Mat orientation)
{
    //viewer.createViewPort(0.0, 0.0, 0.0, 1.0, v1);
    viewer.setBackgroundColor (255, 255, 255);
    Eigen::Affine3f t;
    t(0,0)=orientation.at<double>(0,0);
    t(1,0)=orientation.at<double>(1,0);
    t(2,0)=orientation.at<double>(2,0);
    t(3,0)=0.;
    t(0,1)=orientation.at<double>(0,1);
    t(1,1)=orientation.at<double>(1,1);
    t(2,1)=orientation.at<double>(2,1);
    t(3,1)=0.;
    t(0,2)=orientation.at<double>(0,2);
    t(1,2)=orientation.at<double>(1,2);
    t(2,2)=orientation.at<double>(2,2);
    t(3,2)=0.;
    t(0,3)=0.;
    t(1,3)=0.;
    t(2,3)=0.;
    t(3,3)=1.;

    viewer.addCoordinateSystem(1.0 , t);
}

void Visualizer::AddPointCloud(pcl::visualization::PCLVisualizer& viewer ,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    object_counter++;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            handler (cloud, 0, 0, 255);
    viewer.addPointCloud(cloud, handler , "cloud"+object_counter);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud"+object_counter);
}

void Visualizer::AddPointCloud(pcl::visualization::PCLVisualizer& viewer ,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ,
                               int epoch)
{
    object_counter++;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            handler (cloud, epoch * 20, epoch * 20, 255);
    viewer.addPointCloud(cloud, handler , "cloud"+object_counter);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud"+object_counter);
    Eigen::Vector4f center;
    //pcl::compute3DCentroid(*cloud  , center );
    center(0)=0;
    center(1) =0;
    center(2)=0;
    std::cout << "center: " << center << std::endl;
    pcl::PointXYZ p;
    p.x = center(0); p.y = center(1); p.z = center(2);
    std::string object;
    object = "id" + boost::lexical_cast<std::string>(epoch) +
            "\n" + "x:" + boost::lexical_cast<std::string>(p.x) +
            "\n" + "y:" + boost::lexical_cast<std::string>(p.y) +
            "\n" + "z:" + boost::lexical_cast<std::string>(p.z);

    viewer.addText3D<pcl::PointXYZ>
            (object, p, 0.1 ,0,0,0);
}

void Visualizer::AddPointCloud(pcl::visualization::PCLVisualizer& viewer ,
                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    object_counter++;
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>
            handler (cloud);
    std::string cluster_id = boost::lexical_cast<std::string>(object_counter);
    viewer.addPointCloud(cloud, handler , "cloudRGB"+cluster_id);
    viewer.setPointCloudRenderingProperties
            (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloudRGB"+cluster_id);

}

void Visualizer::AddPointCloud(pcl::visualization::PCLVisualizer& viewer ,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ,
                               cv::Point3f color )
{
    int r =color.x, g=color.y, b=color.z ;
    object_counter++;
    //        double Pr=0.299;
    //        double Pg=0.587;
    //        double Pb=0.114;
    //        double  P =std::sqrt(r*r*Pr + g*g*Pg + b*b*Pb ) ;
    //        double change = epoch / 10;
    //        r=P+(r-P)*change;
    //        g=P+(g-P)*change;
    //        b=P+(b-P)*change;
    std::string str = "cluster"+boost::lexical_cast<std::string>(object_counter);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            handler(cloud, r, g, b);
    viewer.addPointCloud(cloud, handler , str);
    viewer.setPointCloudRenderingProperties (
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                5, str);
}

void Visualizer::AddPointCloud(pcl::visualization::PCLVisualizer& viewer ,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ,
                               std::string text)
{
    object_counter++;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            handler (cloud, 0, 0, 255);
    viewer.addPointCloud(cloud, handler , text);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, text);
    pcl::PointXYZ p;
    Eigen::Vector4f center;
    pcl::compute3DCentroid(*cloud  , center );
    p.x = center(0) ;
    p.y = center(1) ;
    p.z = center(2) +3.0;
    //                viewer.addText3D<pcl::PointXYZI>
    //                (roadPoint, roadlinkcloud->points[i] , 0.1 ,0,0,0);
    viewer.addText3D<pcl::PointXYZ>
            (text, p , 1 ,0,1,1);

//    int r =0, g=0, b=0 ;
//    if (color.compare("red")== 0)
//    {r=255;g=0;b=0;}
//    else if (color.compare("silver")== 0)
//    {r=192;g=192;b=192;}
//    else if (color.compare("teal")== 0)
//    {r=0;g=128;b=128;}
//    else if (color.compare("lightblue")==0)
//    {r=173;g=216;b=230;}

//    object_counter++;
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
//            handler(cloud, r, g, b);
//    std::string str = "cloud"+boost::lexical_cast<std::string>(object_counter);
//    viewer.addPointCloud(cloud, handler , str);
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, str);

}

void Visualizer::AddPointCloud(pcl::visualization::PCLVisualizer& viewer ,
                               cv::Mat b)
{
    object_counter++;
    pcl::PointXYZI p , q;
    p.x = b.at<double>(0,0);
    p.y = b.at<double>(1,0);
    p.z = b.at<double>(2,0);
    p.intensity = 100;
    q.x = 0;
    q.y = 0;
    q.z = 0;
    q.intensity = 100;
    std::string strSphere = "sphere"+boost::lexical_cast<std::string>(object_counter);
    viewer.addSphere (p, 0.2, 0.5, 0.5, 0.0, strSphere);
    std::string strArrow = "arrow_car"+boost::lexical_cast<std::string>(object_counter);
    //viewer.addArrow<pcl::PointXYZI, pcl::PointXYZI>(p,q,0,1,0,false,strArrow);
}

//void Visualizer::AddPointCloud(pcl::visualization::PCLVisualizer& viewer ,
//                               pcl::PointCloud<pcl::PointXYZI>::Ptr cloud0,
//                               pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1,
//                               boost::shared_ptr<pcl::Correspondences> corr)
//{
//    object_counter++;
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>
//            handler (cloud, 0, 0, 255);
//    viewer.addPointCloud(cloud, handler , "cloud"+object_counter);
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud"+object_counter);
//}


void Visualizer::ShowCoordinateSystem(pcl::visualization::PCLVisualizer& viewer,
                                      std::vector<cv::Mat> ENU)
{
    pcl::PointXYZI pO ,pN , pE , pU;

    pO.x = 0; pO.y = 0; pO.z = 0; pO.intensity = 100;

    cv::Mat East;
    East = ENU[0];
    pE.x = East.at<double>(0,0);
    pE.y = East.at<double>(1,0);
    pE.z = East.at<double>(2,0);
    pE.intensity = 100;

    cv::Mat North;
    North = ENU[1];
    pN.x = North.at<double>(0,0);
    pN.y = North.at<double>(1,0);
    pN.z = North.at<double>(2,0);
    pN.intensity = 100;

    cv::Mat Up;
    Up = ENU[2];
    pU.x = Up.at<double>(0,0);
    pU.y = Up.at<double>(1,0);
    pU.z = Up.at<double>(2,0);
    pU.intensity = 100;

    viewer.addArrow<pcl::PointXYZI, pcl::PointXYZI>
            (pE,pO, 1,1,1,false,"arrow1");
    viewer.addText3D<pcl::PointXYZI>
            ("E", pE , 1 ,1,1,1);

    viewer.addArrow<pcl::PointXYZI, pcl::PointXYZI>
            (pN,pO, 1,1,1,false,"arrow2");
    viewer.addText3D<pcl::PointXYZI>
            ("N", pN, 1 ,1,1,1);

    viewer.addArrow<pcl::PointXYZI, pcl::PointXYZI>
            (pU,pO, 1,1,1,false,"arrow3");
    viewer.addText3D<pcl::PointXYZI>
            ("U", pU , 1 ,1,1,1);

}

void Visualizer::ShowRoadLinks(pcl::visualization::PCLVisualizer& viewer,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr roadlinkcloud)
{
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            RoadLinkhandler (roadlinkcloud, 0, 255, 0);
    viewer.addPointCloud(roadlinkcloud, RoadLinkhandler , "RoadLinks");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "RoadLinks");
    for (int  i = 0 ; i < roadlinkcloud->size() - 1; i=i+1)
    {
        std::string ss_line;
        ss_line = "link" + boost::lexical_cast<std::string>(i) + "_" +
                boost::lexical_cast<std::string>(i+1) ;
        viewer.addLine<pcl::PointXYZ, pcl::PointXYZ>
                (roadlinkcloud->points[i], roadlinkcloud->points[i+1] ,
                 0, 255, 0, ss_line);
        //        std::string roadPoint;
        //        roadPoint = "x:" +  boost::lexical_cast<std::string>( roadlinkcloud->points[i].x) +
        //                "\n" + "y:" + boost::lexical_cast<std::string>(roadlinkcloud->points[i].y) +
        //                "\n" + "z:" + boost::lexical_cast<std::string>(roadlinkcloud->points[i].z);

        //                viewer.addText3D<pcl::PointXYZI>
        //                (roadPoint, roadlinkcloud->points[i] , 0.1 ,0,0,0);
    }
}

/**
    * Computes the color gradiant
    * color: the output vector
    * x: the gradiant (beetween 0 and 360)
    * min and max: variation of the RGB channels (Move3D 0 -> 1)
    */
cv::Point3f Visualizer::GroundColorMix( float x, float min, float max)
{
    /*
   * Red = 0
   * Green = 1
   * Blue = 2
   */
    cv::Point3f color;
    float posSlope = (max-min)/60;
    float negSlope = (min-max)/60;

    if( x < 60 )
    {
        color.x = (int) (max);
        color.y = (int) (posSlope*x+min);
        color.z = (int) (min);
    }
    else if ( x < 120 )
    {
        color.x = (int) (negSlope*x+2*max+min);
        color.y = (int) (max);
        color.z = (int) (min);
    }
    else if ( x < 180  )
    {
        color.x  = (int) (min);
        color.y = (int) (max);
        color.z = (int) (posSlope*x-2*max+min);
    }
    else if ( x < 240  )
    {
        color.x = (int) (min);
        color.y = (int) (negSlope*x+4*max+min);
        color.z = (int) (max);
    }
    else if ( x < 300  )
    {
        color.x = (int) (posSlope*x-4*max+min);
        color.y = (int) (min);
        color.z = (int) (max);
    }
    else
    {
        color.x = (int) (max);
        color.y = (int) (min);
        color.z = (int) (negSlope*x+6*max);
    }
    return color;
}

void Visualizer::FindMinMaxPCD(float& max , float& min, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr proj_PCD)
{
    max = 0.;
    min = 1000.;
    for (int32_t i = 0 ; i < proj_PCD->points.size() ; i++)
    {
        pcl::PointXYZRGB p = proj_PCD->points[i];
        if (p.z > max)
            max = p.z;
        if (p.z < min)
            min = p.z;
    }
}

cv::Mat Visualizer::ProjectedPCD_Vis(cv::Mat image , const pcl::PointCloud<pcl::PointXYZRGB>::Ptr proj_PCD)
{
    cv::Mat img = image.clone();
    float maxDepth , minDepth;
    FindMinMaxPCD(maxDepth , minDepth, proj_PCD);
    std::cout << " max " << maxDepth << " min " << minDepth << std::endl;
    cv::Point3f color;
    for (int i = 0 ; i < proj_PCD->size() ; i++)
    {
        cv::Point2f q;
        pcl::PointXYZRGB p = proj_PCD->points[i];

        q.x = (float) (p.x) ;
        q.y = (float) (p.y) ;
        color = GroundColorMix(((float)(p.z) * 360. / maxDepth), minDepth, maxDepth);
        cv::circle(img ,q ,1,cv::Scalar(color.z, color.y, color.x),2,8);
    }
    return img;
}

void Visualizer::AddPointCloud(pcl::visualization::PCLVisualizer& viewer ,
                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud ,
                               pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    object_counter++;

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer.addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals);//, 10, 0.05, "normals");
    // viewer.addCoordinateSystem (1.0);
    //  viewer.initCameraParameters ();


    //    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>
    //            handler (cloud);
    //    viewer.addPointCloud(cloud, handler , "cloudRGBNormals"+object_counter);
    //    viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "cloudRGBNormals"+object_counter);
    //    viewer.setPointCloudRenderingProperties
    //            (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloudRGBNormals"+object_counter);
}

void Visualizer::AddPointCloud(pcl::visualization::PCLVisualizer& viewer ,
                               pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud)
{
    object_counter++;
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>
            handler (cloud);
    viewer.addPointCloud(cloud, handler , "cloudRGBNormal"+object_counter);
    viewer.setPointCloudRenderingProperties
            (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloudRGBNormal"+object_counter);


    //    object_counter++;
    //    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>
    //            handler (cloud);
    //    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> color2 (cloud, 0, 0, 255);
    //    viewer.addPointCloud (cloud, color2, "cloudRGBNormal"+object_counter);
    //    viewer.addPointCloud(cloud, handler , "cloudRGBNormal"+object_counter);
    ////    viewer.addPointCloud (cloud , "normals" ,);
    ////    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "normals");

    ////    viewer.setPointCloudRenderingProperties
    ////            (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloudRGBNormal"+object_counter);
    //    //pcl::visualization::PCLVisualizerInteractorStyle::OnKeyDown();

    //    //    viewer.setPointCloudRenderingProperties
    ////            (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "cloudRGBNormal"+object_counter);
}

void Visualizer::AddPolygonMesh(pcl::visualization::PCLVisualizer& viewer ,
                                pcl::PolygonMesh mesh)
{
    viewer.addPolygonMesh(mesh);
}

void Visualizer::AddPointCloud(pcl::visualization::PCLVisualizer& viewer ,
                               std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters)
{
    for (int i = 0 ; i < clusters.size() ; i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = clusters[i] ;
        object_counter++;
        std::string cluster_id = boost::lexical_cast<std::string>(object_counter);
        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ>
                handler (cloud);
        viewer.addPointCloud(cloud, handler , "cloud"+cluster_id);
        viewer.setPointCloudRenderingProperties
                (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud"+cluster_id);
    }

}

void Visualizer::ShowRoadLinks(pcl::visualization::PCLVisualizer& viewer,
                               multi_line_2D roadlink)
{
    int i = 0;
    BOOST_FOREACH(mline_2d segments, roadlink)
    {
        i++;
        //std::cout << i << ":" << segments[0].x() << std::endl;

        pcl::PointXYZ p,q;
        p.x = segments[0].x();
        p.y = segments[0].y();
        p.z = 0;
        q.x = segments[1].x();
        q.y = segments[1].y();
        q.z = 0;
        std::string ss_line = "link" + boost::lexical_cast<std::string>(i) ;
        viewer.addLine<pcl::PointXYZ, pcl::PointXYZ>
                (p , q , 0, 255, 0, ss_line);
    }
}

void Visualizer::ShowRoadLinks(pcl::visualization::PCLVisualizer& viewer,
                               std::vector<std::vector<pcl::PointXYZ> > points)
{
    std::cout << "points " << points.size() << std::endl;
    for (int i = 0; i < points.size() ; i++)
    {
        std::vector<pcl::PointXYZ> point = points[i];
        for (int j = 0; j < point.size() - 1; j++)
        {
        pcl::PointXYZ p1 = point[j];
        pcl::PointXYZ p2 = point[j + 1];
        std::string ss_line;
        ss_line = "roadlink" + boost::lexical_cast<std::string>(i) + "_"
                + boost::lexical_cast<std::string>(j);
        viewer.addLine<pcl::PointXYZ, pcl::PointXYZ>
                (p1, p2 ,
                 0, 255, 0, ss_line);
        }

    }
}

void Visualizer::ShowRoadLinks(pcl::visualization::PCLVisualizer& viewer,
                               std::vector<pcl::PointXYZ> points)
{
    for (int i = 0; i < points.size() ; i++)
    {
        pcl::PointXYZ p = points[i];
        std::string ss_line;
        ss_line = "pointlink" + boost::lexical_cast<std::string>(i)
                + boost::lexical_cast<std::string>(object_counter++);
        viewer.addSphere (p, 0.2, 0.5, 0.5, 0.0, ss_line);
    }
}
