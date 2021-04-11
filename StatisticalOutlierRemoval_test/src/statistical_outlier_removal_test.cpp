/*
* 文件名:   test_statistical_outlier_removal.cpp
* 版权:     Copyright 2019-2022 wong.  All Rights Reserved.
* 描述:     使用RANSAC算法实现点云平面拟合与分割
* 参考链接:  StatisticalOutlierRemoval：离群点移除 https://www.cnblogs.com/penuel/p/13639803.html
*           PCL两个窗口显示点云 https://blog.csdn.net/weixin_45377028/article/details/104564467
* 修改人:   wong
* 修改时间: 2021-04-11
* 修改内容: 实现了使用PCL库中 StatisticalOutlierRemoval库函数 进行点云离群点移除.
* 
*/
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_inliers (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_outliers (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<pcl::PointXYZ> ("../table_scene_lms400.pcd", *cloud); //点云的读取

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);  //设置输入
  sor.setMeanK (50);  //设置用于平均距离估计的 KD-tree最近邻搜索点的个数.
  sor.setStddevMulThresh (1.0); //高斯分布标准差的倍数, 也就是 u+1*sigma,u+2*sigma,u+3*sigma 中的 倍数1、2、3 
  sor.filter (*cloud_filtered_inliers); // 滤波后输出 //具体实现可以见 pcl-pcl-1.8.1/filters/src/filter.cpp中的filter()(内部调用的是statistical_outlier_removal.cpp中的applyFilter()).

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered_inliers << std::endl;

  sor.setNegative (true);
  sor.filter (*cloud_filtered_outliers);
  // 将滤波后的点云、离群点云分别写入文件
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("table_scene_lms400_inliers.pcd", *cloud_filtered_inliers, false);
  writer.write<pcl::PointXYZ> ("table_scene_lms400_outliers.pcd", *cloud_filtered_outliers, false);


  pcl::visualization::PCLVisualizer viewer("双窗口学习");
  //添加坐标系
  //viewer.addCoordinateSystem(0,0);

  int v1(0);   //设置左右窗口
  int v2(1);
  int v3(2);

  viewer.createViewPort(0.0, 0.5, 1, 1, v1); 
  viewer.setBackgroundColor(0, 0, 0, v1);
  
  viewer.createViewPort(0.0, 0.0, 0.5, 0.5, v2);
  viewer.setBackgroundColor(0.5, 0.5, 0.5, v2);

  viewer.createViewPort(0.5, 0.0, 1, 0.5 , v3); 
  viewer.setBackgroundColor(0, 0, 0, v3);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_green(cloud, 0, 255, 0);      // 显示绿色点云
  viewer.addPointCloud(cloud, cloud_in_green, "cloud_in", v1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_out_blue(cloud_filtered_inliers, 0, 0, 255);      // 显示蓝色点云
  viewer.addPointCloud(cloud_filtered_inliers, cloud_out_blue, "cloud_out_inliers", v2);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_out_orage(cloud_filtered_outliers, 250, 128, 10);     //显示橘色点云
  viewer.addPointCloud(cloud_filtered_outliers, cloud_out_orage, "cloud_out_outliers", v3);
  //viewer.setSize(960, 780);
  while (!viewer.wasStopped())
  {
    viewer.spinOnce();
  }

  return (0);
}
