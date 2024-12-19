#include <gflags/gflags.h>
#include <pcl/io/pcd_io.h>
#include <filters/lidsor.hh>
#include <methods/timer.hh>
#include <cstdio>

DEFINE_string(f, "", "origin_cloud");
DEFINE_string(ep, "./ep.pcd", "ep path");
DEFINE_string(en, "./en.pcd", "en path");
DEFINE_int32(k, 50, "min_k_neighbours"); 
DEFINE_double(m, 0.15, "standard_deviation_multiplier");
DEFINE_double(r, 0.05, "range_multiplier");
DEFINE_double(d, 20.0, "distance_threshold");
DEFINE_int32(i, 5, "intensity_threshold");

int main(int argc, char *argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  PointCloudT::Ptr origin(new PointCloudT);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr en(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ep(new pcl::PointCloud<pcl::PointXYZRGBA>);

  if (!loadPointCloudFromTxt(FLAGS_f, origin)) {
    return -1;
  }

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr converted(new pcl::PointCloud<pcl::PointXYZRGBA>);
  for (int i = 0; i < origin->points.size(); ++i) {
    pcl::PointXYZRGBA pcl_point;
    convertToPclPointXYZRGBA(origin->points[i], pcl_point, i);
    converted->points.push_back(pcl_point);
  }

  Timer timer;
  LIDSOR outrem;
  outrem.SetMeanK(FLAGS_k);
  outrem.SetStandardDeviationMultiplier(FLAGS_m);
  outrem.SetRangeMultiplier(FLAGS_r);
  outrem.SetIntensityThreshold(FLAGS_i);
  outrem.SetDistanceThreshold(FLAGS_d);
  outrem.Filter(converted, *en, *ep);
  printf("%.2f ms\n", timer.Get() * 1e-6);

  PointCloudT::Ptr restored_en(new PointCloudT);
  PointCloudT::Ptr restored_ep(new PointCloudT);

  for (const auto& point : en->points) {
    PointT restored_point;
    restoreAdditionalFields(point, restored_point, *origin);
    restored_en->points.push_back(restored_point);
  }

  for (const auto& point : ep->points) {
    PointT restored_point;
    restoreAdditionalFields(point, restored_point, *origin);
    restored_ep->points.push_back(restored_point);
  }

  savePointCloudToTxt(restored_ep, FLAGS_ep);
  savePointCloudToTxt(restored_en, FLAGS_en);

  return 0;
}