//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/surface/gp3.h>      //贪婪投影三角化算法
//#include <pcl/io/vtk_lib_io.h>
//
//
//int main (int argc, char** argv)
//{
//    // 将一个XYZ点类型的PCD文件打开并存储到对象中
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PCLPointCloud2 cloud_blob;
//    pcl::io::loadPCDFile ("/home/桌面/dd/zxc.pcd", cloud_blob);
//    pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
//    //* the data should be available in cloud
//
//    // Normal estimation*
//    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;      //法线估计对象
//    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);   //存储估计的法线
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);  //定义kd树指针
//    tree->setInputCloud (cloud);   ///用cloud构建tree对象
//    n.setInputCloud (cloud);
//    n.setSearchMethod (tree);
//    n.setKSearch (20);
//    n.compute (*normals);       ////估计法线存储到其中
//    //* normals should not contain the point normals + surface curvatures
//
//    // Concatenate the XYZ and normal fields*
//    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
//    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);    //连接字段
//    //* cloud_with_normals = cloud + normals
//
//    //定义搜索树对象
//    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
//    tree2->setInputCloud (cloud_with_normals);   //点云构建搜索树
//
//    // Initialize objects
//    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;   //定义三角化对象
//    pcl::PolygonMesh triangles;                //存储最终三角化的网络模型
//
//    // Set the maximum distance between connected points (maximum edge length)
//    gp3.setSearchRadius (50);  //设置连接点之间的最大距离，（即是三角形最大边长）
//
//    // 设置各参数值
//    gp3.setMu (5);  //设置被样本点搜索其近邻点的最远距离为2.5，为了使用点云密度的变化
//    gp3.setMaximumNearestNeighbors (10000);    //设置样本点可搜索的邻域个数
//    gp3.setMaximumSurfaceAngle(M_PI/4); // 设置某点法线方向偏离样本点法线的最大角度45
//    gp3.setMinimumAngle(M_PI/18); // 设置三角化后得到的三角形内角的最小的角度为10
//    gp3.setMaximumAngle(2*M_PI/3); // 设置三角化后得到的三角形内角的最大角度为120
//    gp3.setNormalConsistency(false);  //设置该参数保证法线朝向一致
//
//    // Get result
//    gp3.setInputCloud (cloud_with_normals);     //设置输入点云为有向点云
//    gp3.setSearchMethod (tree2);   //设置搜索方式
//    gp3.reconstruct (triangles);  //重建提取三角化
//
//    // 附加顶点信息
//    std::vector<int> parts = gp3.getPartIDs();
//    std::vector<int> states = gp3.getPointStates();
//
//
//    pcl::io::savePolygonFile("/home/cobot/桌面/dd/trangle.stl",triangles);
//    // Finish
//    return (0);
//}


//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/voxel_grid.h>
//
//
//int
//main (int argc, char** argv)
//{
//
//    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
//    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
//
//    //点云对象的读取
//    pcl::PCDReader reader;
//
//    reader.read ("/home/桌面/dd/noname_20190518170701.pcd", *cloud);    //读取点云到cloud中
//
//    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
//              << " data points (" << pcl::getFieldsList (*cloud) << ").";
//
//    /******************************************************************************
//    创建一个叶大小为1cm的pcl::VoxelGrid滤波器，
//  **********************************************************************************/
//    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;  //创建滤波对象
//    sor.setInputCloud (cloud);            //设置需要过滤的点云给滤波对象
//    sor.setLeafSize (1, 1, 1);  //设置滤波时创建的体素体积为1cm的立方体
//    sor.filter (*cloud_filtered);           //执行滤波处理，存储输出
//
//    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
//              << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";
//
//    pcl::PCDWriter writer;
//    writer.write ("/home/桌面/dd/zxc.pcd", *cloud_filtered,
//                  Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
//
//    return (0);
//}
//



///////PCL 求质心   点云质心是（x=-41.8134,y=93.9094,z=901.151)
//#include <iostream>
//#include <pcl/io/ply_io.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/common/centroid.h> //中心？
//int main(int argc, char** argv)
//{
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//
//    pcl::io::loadPLYFile<pcl::PointXYZ>("/home/桌面/dd/qqqqq.ply", *cloud) == -1;
//    //创建存储点云质心的对象
//    Eigen::Vector4f centroid;
//    //
//    pcl::compute3DCentroid(*cloud, centroid);
//    std::cout << "点云质心是（"
//              << centroid[0] << ","
//              << centroid[1] << ","
//              << centroid[2] << ")." << std::endl;
//    system("pause");
//    return 0;
//}


//////平面拟合     Model coefficients: a=0.00103819 b=0.00267527 c=0.999996 d=-1129.39
//#include <iostream>
//#include <pcl/ModelCoefficients.h>  //模型系数
//#include <pcl/io/pcd_io.h>                  //输入输出
//#include <pcl/point_types.h>                //点云（类型）
//#include <pcl/sample_consensus/method_types.h>  //随机样本一致性算法 方法类型
//#include <pcl/sample_consensus/model_types.h>       //随机样本一致性算法 模型类型
//#include <pcl/segmentation/sac_segmentation.h>  //随机样本一致性算法 分割方法
//#include <pcl/io/ply_io.h>
//
//int main (int argc, char** argv)
//{
//
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::io::loadPLYFile<pcl::PointXYZ>("/home/桌面/dd/ping_mian_ni_he.ply", *cloud) == -1;
//
//    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients); //存储输出的模型的系数
//    pcl::PointIndices::Ptr inliers (new pcl::PointIndices); //存储内点，使用的点
//
//    //创建分割对象
//    pcl::SACSegmentation<pcl::PointXYZ> seg;
//    //可选设置
//    seg.setOptimizeCoefficients (true);
//    //必须设置
//    seg.setModelType (pcl::SACMODEL_PLANE); //设置模型类型，检测平面
//    seg.setMethodType (pcl::SAC_RANSAC);      //设置方法【聚类或随机样本一致性】
//    seg.setDistanceThreshold (0.01);
//    seg.setInputCloud (cloud->makeShared ());
//    seg.segment (*inliers, *coefficients);    //分割操作
//
//    if (inliers->indices.size () == 0)//根据内点数量，判断是否成功
//    {
//
//        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
//        return (-1);
//    }
//    //显示模型的系数
//    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
//              <<coefficients->values[1] << " "
//              <<coefficients->values[2] << " "
//              <<coefficients->values[3] <<std::endl;
//
//    return (0);
//}


/////顶点最值  min.x = -97.1654   min.y = -84.4536   max.x = 14.7979   max.y = 274.748
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/io/ply_io.h>
using namespace std;
int
main(int argc,char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPLYFile<pcl::PointXYZ>("/home/桌面/dd/qqqqq.ply", *cloud) == -1;


    pcl::PointXYZ min;//用于存放三个轴的最小值
    pcl::PointXYZ max;//用于存放三个轴的最大值
    pcl::getMinMax3D(*cloud,min,max);

    cout<<"min.x = "<<min.x<<"min.y = "<<min.y<<"\n"<<endl;
    cout<<"max.x = "<<max.x<<"max.y = "<<max.y<<"\n"<<endl;
    return 0;

}
