#ifndef MY_PCL_EXTENSIONS_HH
#define MY_PCL_EXTENSIONS_HH

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <fstream>
#include <string>
#include <sstream>

// x, y, z, intensity, label and time
struct PointXYZILT
{
  PCL_ADD_POINT4D;
  int intensity;
  int label;
  int time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZILT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (int, intensity, intensity)
                                  (int, label, label)
                                  (int, time, time))

bool loadPointCloudFromTxt(const std::string &filename, pcl::PointCloud<PointXYZILT>::Ptr cloud)
{
  std::ifstream fs(filename);
  if (!fs.is_open())
  {
    return false;
  }

  std::string line;
  PointXYZILT point;
  while (std::getline(fs, line))
  {
    std::istringstream iss(line);
    iss >> point.x >> point.y >> point.z >> point.intensity >> point.label >> point.time;
    cloud->points.push_back(point);
  }

  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;

  return true;
}

void savePointCloudToTxt(const pcl::PointCloud<PointXYZILT>::Ptr cloud, const std::string &filename)
{
  std::ofstream fs(filename);
  if (!fs.is_open())
  {
    return;
  }

  for (const auto &point : cloud->points)
  {
    fs << point.x << " " << point.y << " " << point.z << " " << point.intensity << " " << point.label << " " << point.time << std::endl;
  }
}


void savePointCloudToTxt(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const std::string &filename)
{
  std::ofstream fs(filename);
  if (!fs.is_open())
  {
    return;
  }

  for (const auto &point : cloud->points)
  {
    fs << point.x << " " << point.y << " " << point.z << " " << static_cast<int>(point.r) << " " << static_cast<int>(point.g) << " " << static_cast<int>(point.b) << std::endl;
  }
}

void savePointCloudToTxt(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB, 
    const pcl::PointCloud<PointXYZILT>::Ptr cloudCustom, 
    const std::string &filename)
{

    std::ofstream fs(filename, std::ios::out);  
    if (!fs.is_open())
    {
        return;
    }

    for (const auto &point : cloudRGB->points)
    {
        fs << point.x << " " << point.y << " " << point.z 
           << " " << static_cast<int>(point.r) 
           << " " << static_cast<int>(point.g) 
           << " " << static_cast<int>(point.b) << std::endl;
    }

    fs.close();
    fs.open(filename, std::ios::app);  

    for (const auto &point : cloudCustom->points)
    {
        fs << point.x << " " << point.y << " " << point.z 
           << " " << point.intensity 
           << " " << point.label 
           << " " << point.time << std::endl;
    }

    fs.close();
}

void convertToPclPointXYZI(const PointXYZILT& src, pcl::PointXYZI& dest, int index) {
  dest.x = src.x;
  dest.y = src.y;
  dest.z = src.z;
  dest.intensity = static_cast<int>(index);  
}

void restoreAdditionalFields(const pcl::PointXYZI& src, PointXYZILT& dest, const pcl::PointCloud<PointXYZILT>& originalCloud) {
  int index = static_cast<int>(src.intensity);  
  dest.x = originalCloud.points[index].x;
  dest.y = originalCloud.points[index].y;
  dest.z = originalCloud.points[index].z;
  dest.intensity = originalCloud.points[index].intensity;
  dest.label = originalCloud.points[index].label;
  dest.time = originalCloud.points[index].time;
}

void convertToPclPointXYZRGBA(const PointXYZILT& src, pcl::PointXYZRGBA& dest, int index) {
  dest.x = src.x;
  dest.y = src.y;
  dest.z = src.z;
  dest.r = static_cast<uint8_t>(src.intensity); 

  dest.g = static_cast<uint8_t>(index % 256);  
  dest.b = static_cast<uint8_t>((index / 256) % 256);  
  dest.a = static_cast<uint8_t>(index / (256 * 256));  

}

void restoreAdditionalFields(const pcl::PointXYZRGBA& src, PointXYZILT& dest, const pcl::PointCloud<PointXYZILT>& originalCloud) {
  int index = src.g + src.b * 256 + src.a * 256 * 256;  

  dest.x = originalCloud.points[index].x;
  dest.y = originalCloud.points[index].y;
  dest.z = originalCloud.points[index].z;
  dest.intensity = originalCloud.points[index].intensity;
  dest.label = originalCloud.points[index].label;
  dest.time = originalCloud.points[index].time;
}

void convertToPclPointXYZI(const PointXYZILT& src, pcl::PointXYZI& dest) {
  dest.x = src.x;
  dest.y = src.y;
  dest.z = src.z;
  dest.intensity = src.intensity;
}

void convertToPclPointXYZRGB(const PointXYZILT& src, pcl::PointXYZRGB& dest) {
  dest.x = src.x;
  dest.y = src.y;
  dest.z = src.z;
  dest.r = static_cast<uint8_t>(src.intensity); 
  dest.g = static_cast<uint8_t>(src.label);     
  dest.b = static_cast<uint8_t>(src.time);      
}



#endif // MY_PCL_EXTENSIONS_HH
