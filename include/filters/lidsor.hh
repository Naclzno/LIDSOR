#ifndef INCLUDE_LIDSOR_HH
#define INCLUDE_LIDSOR_HH

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <methods/preprocessing.hh>

template <typename T>
inline double distance(const T &p) { return sqrt(p.x * p.x + p.y * p.y + p.z * p.z); }

class LIDSOR {
public:
  LIDSOR() = default;
  ~LIDSOR() = default;

  void SetMeanK(const int mean_k) { this->mean_k_ = mean_k; }
  void SetStandardDeviationMultiplier(const double std_mul) {
    this->standard_deviation_multiplier_ = std_mul;
  }
  void SetRangeMultiplier(const double range_multiplier) {
    this->range_multiplier_ = range_multiplier;
  }
  void SetIntensityThreshold(const int32_t intensity_threshold) {
    this->intensity_threshold_ = intensity_threshold;
  }
  void SetDistanceThreshold(const double distance_threshold) {
    this->distance_threshold_ = distance_threshold;
  }

  int GetMeanK() { return this->mean_k_; }
  double GetStandardDeviationMultiplier() {
    return this->standard_deviation_multiplier_;
  }
  double GetRangeMultiplier() { return this->range_multiplier_; }
  int8_t GetIntensityThreshold() { return this->intensity_threshold_; }
  
  template <typename T>
  void Filter(typename pcl::PointCloud<T>::Ptr &input_cloud,
              typename pcl::PointCloud<T> &filtered_cloud,
              typename pcl::PointCloud<T> &noise_cloud) {

    typename pcl::KdTreeFLANN<T> kd_tree;
    kd_tree.setInputCloud(input_cloud);

    std::vector<int> pointIdxNKNSearch(this->mean_k_);
    std::vector<float> pointNKNSquaredDistance(this->mean_k_);
    std::vector<float> mean_distances;

    for (typename pcl::PointCloud<T>::iterator it = input_cloud->begin();
         it != input_cloud->end(); ++it) {
      kd_tree.nearestKSearch(*it, this->mean_k_, pointIdxNKNSearch,
                             pointNKNSquaredDistance);

      double dist_sum = 0;
      for (int j = 1; j < this->mean_k_; ++j) {
        dist_sum += sqrt(pointNKNSquaredDistance[j]);
      }
      mean_distances.push_back(
          static_cast<float>(dist_sum / (this->mean_k_ - 1)));
    }

    double sum = 0, sq_sum = 0;
    for (size_t i = 0; i < mean_distances.size(); ++i) {
      sum += mean_distances[i];
      sq_sum += mean_distances[i] * mean_distances[i];
    }
    double mean = sum / static_cast<double>(mean_distances.size());
    double variance =
        (sq_sum - sum * sum / static_cast<double>(mean_distances.size())) /
        (static_cast<double>(mean_distances.size()) - 1);
    double stddev = sqrt(variance);

    double distance_threshold =
        (mean + this->standard_deviation_multiplier_ * stddev);
    int i = 0;
    for (typename pcl::PointCloud<T>::iterator it = input_cloud->begin();
         it != input_cloud->end(); ++it) {
      double range = distance(*it);
      double dynamic_threshold =
          distance_threshold * this->range_multiplier_ * range;

      if (mean_distances[i] > dynamic_threshold and it->r < this->intensity_threshold_ and distance(*it) < this->distance_threshold_) {
        noise_cloud.push_back(*it);
      } else {
        filtered_cloud.push_back(*it);
      }
      i++;
    }
  }

private:
  int mean_k_; 
  double standard_deviation_multiplier_; 
  double range_multiplier_; 
  uint8_t intensity_threshold_; 
  double distance_threshold_; 
};

#endif // INCLUDE_LIDSOR_HH