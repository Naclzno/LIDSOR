#ifndef PREPROCESSING_HH
#define PREPROCESSING_HH

#include <pcl/point_cloud.h>
#include <cmath>
#include <methods/point.hh> 

using PointT = PointXYZILT;
using PointCloudT = pcl::PointCloud<PointT>;

void preprocessPointCloud(PointCloudT::Ptr cloud, PointCloudT::Ptr environment_features, float distance_threshold, int intensity_threshold) {
  
  PointCloudT::Ptr temp_cloud_filtered(new PointCloudT); 
  for (const auto& point : cloud->points) {

    float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    if (distance < distance_threshold && point.intensity < intensity_threshold) {
      temp_cloud_filtered->points.push_back(point);
    } else {
      if (point.time == 3) {
      environment_features->points.push_back(point);
      }
    }
  }

  cloud->swap(*temp_cloud_filtered);

}


#endif // PREPROCESSING_HH
