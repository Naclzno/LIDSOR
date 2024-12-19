#include <methods/eva.hh> 
#include <gflags/gflags.h>
#include <cstdio> 

DEFINE_string(f, "", "origin_cloud");
DEFINE_string(ep, "", "exp_positive"); // noise
DEFINE_string(en, "", "exp_negative"); // non-noise

int main(int argc, char *argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  PointCloudT::Ptr origin(new PointCloudT);
  PointCloudT::Ptr sp_converted(new PointCloudT);
  PointCloudT::Ptr sn_converted(new PointCloudT);
  PointCloudT::Ptr ep_converted(new PointCloudT);
  PointCloudT::Ptr en_converted(new PointCloudT);

  pcl::PointCloud<pcl::PointXYZI>::Ptr sp(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr sn(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr ep(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr en(new pcl::PointCloud<pcl::PointXYZI>);

  if (!loadPointCloudFromTxt(FLAGS_f, origin)) {
    return -1;
  }

  separateByLabel<PointT>(origin, sn_converted, sp_converted);
  
  // separateByLabel<PointXYZIL>(origin, sn_converted, sp_converted);
 
  if (!loadPointCloudFromTxt(FLAGS_ep, ep_converted)) {
    return -1;
  }
  if (!loadPointCloudFromTxt(FLAGS_en, en_converted)) {
    return -1;
  }

  for (const auto& point : sp_converted->points) {
    pcl::PointXYZI new_point;
    convertToPclPointXYZI(point, new_point);
    sp->points.push_back(new_point);
  }

  for (const auto& point : sn_converted->points) {
    pcl::PointXYZI new_point;
    convertToPclPointXYZI(point, new_point);
    sn->points.push_back(new_point);
  }

  for (const auto& point : ep_converted->points) {
    pcl::PointXYZI new_point;
    convertToPclPointXYZI(point, new_point);
    ep->points.push_back(new_point);
  }

  for (const auto& point : en_converted->points) {
    pcl::PointXYZI new_point;
    convertToPclPointXYZI(point, new_point);
    en->points.push_back(new_point);
  }

  enum {accuracy, error, precision, recall, fscore};
  std::vector<double> result = eva<pcl::PointXYZI>(sp, sn, ep, en, 0.001);
  printf("%.2f %.2f %.2f %.2f %.2f\n", result[accuracy], result[error], result[precision], result[recall], result[fscore]);

  return 0;
}