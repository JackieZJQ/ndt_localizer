#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Geometry>

#include <vector>

namespace ndt {

class MapLoader : public rclcpp::Node
{
public:
  MapLoader(const char* name);

  void InitIO();

  void InitTfParams();

private:
  bool LoadPcd(sensor_msgs::msg::PointCloud2& msgMap);

  bool LoadPcd(pcl::PointCloud<pcl::PointXYZI>& cloudMap);

  void SavePcd(const pcl::PointCloud<pcl::PointXYZI>& cloudMap);

  void TransformMap(pcl::PointCloud<pcl::PointXYZI>& cloudInMap, sensor_msgs::msg::PointCloud2& msgOutMap);

private:
  std::string pcd_file_path_ = "";
  
  std::string points_map_topic_ = "";

  Eigen::Affine3d map_transform_;

  std::vector<std::string> map_path_list_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
};

} // namespace ndt