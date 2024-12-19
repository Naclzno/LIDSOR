#ifndef INCLUDE_EVA_HH
#define INCLUDE_EVA_HH

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <methods/preprocessing.hh>

template <typename T>
int intersection(const typename pcl::PointCloud<T>::Ptr &c1, typename pcl::PointCloud<T>::Ptr c2, float distance_threshold) {
  int correct = 0;
  int k = 1; 
  auto kdtree(new pcl::KdTreeFLANN<T>); 
  kdtree->setInputCloud(c2); 
  std::vector<int> pointIdxNKNSearch(k);
  std::vector<float> pointNKNSquareDistance(k);
  for (auto searchPoint = c1->begin(); searchPoint != c1->end(); searchPoint++) 
    if (kdtree->nearestKSearch(*searchPoint, k, pointIdxNKNSearch,
                               pointNKNSquareDistance) > 0) 
      if (pointNKNSquareDistance[0] <= distance_threshold) 
        correct++;
  return correct;
}

template <typename T>
std::vector<double> eva(const typename pcl::PointCloud<T>::Ptr &sp, const typename pcl::PointCloud<T>::Ptr &sn,
                        const typename pcl::PointCloud<T>::Ptr &ep,
                        const typename pcl::PointCloud<T>::Ptr &en, float distance_threshold) {
  int tn = intersection<T>(sn, en, distance_threshold); 
  int fn = intersection<T>(sp, en, distance_threshold); 
  int fp = intersection<T>(sn, ep, distance_threshold); 
  int tp = intersection<T>(sp, ep, distance_threshold);
  double accuracy = static_cast<double>(tp + tn) / static_cast<double>(tp + tn + fp + fn);
  double error = static_cast<double>(fp + fn) / static_cast<double>(tp + tn + fp + fn);
  double precision = static_cast<double>(tp) / static_cast<double>(tp + fp);
  double recall = static_cast<double>(tp) / static_cast<double>(tp + fn);
  double fscore = 2*precision*recall/(precision+recall);

  std::vector<double> result{accuracy, error, precision, recall, fscore};
  return result;
}

template <typename T>
void separateByLabel(const typename pcl::PointCloud<T>::Ptr &input_cloud,
                     typename pcl::PointCloud<T>::Ptr &sn, 
                     typename pcl::PointCloud<T>::Ptr &sp  
                     ) {
  for (const auto &point : input_cloud->points) {
    
    if (point.time != 3) {
      continue;  
    }
    
    if (point.label == 0) {
      sn->points.push_back(point); 
    } else if (point.label == 1) {
      sp->points.push_back(point); 
    }
  }

  sn->width = sn->points.size();
  sn->height = 1;
  sn->is_dense = true;

  sp->width = sp->points.size();
  sp->height = 1;
  sp->is_dense = true;
}

#endif // INCLUDE_EVA_HH