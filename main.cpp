//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/io/ply_io.h>
//
//int main(int argc, char** argv)
//{
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//
//    if (pcl::io::loadPLYFile<pcl::PointXYZ>("/home/桌面/qwe.ply", *cloud) == -1) //* load the file
//    {
//        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
//        system("PAUSE");
//        return (-1);
//    }
//    std::cout << "Loaded "
//              << cloud->width * cloud->height
//              << " data points from test_pcd.pcd with the following fields: "
//              << std::endl;
//    for (size_t i = 0; i < cloud->points.size(); ++i)
//        std::cout << "    " <<(cloud->points[i].x)/1000
//                  << " " <<(cloud->points[i].y)/1000
//                  << " " <<(cloud->points[i].z)/1000
//                  << std::endl;
//
//
//    for (size_t i =0; i<cloud->points.size(); ++i)
//    {
//        cloud->points[i].x = (cloud->points[i].x)/100;
//        cloud->points[i].y = (cloud->points[i].y)/100;
//        cloud->points[i].z = (cloud->points[i].z)/100;
//    }
//    pcl::io::savePLYFile<pcl::PointXYZ>("/home/cobot/桌面/asd.ply",*cloud) == -1;
//    //std::string filename("test.pcd");
//    //pcl::PCDWriter writer;
//    //writer.write(filename, *cloud);
//
//    system("PAUSE");
//    return (0);
//}

