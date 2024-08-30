#include "map_loader.h"

namespace ndt {

MapLoader::MapLoader(const char* name) : Node(name)
{
  RCLCPP_INFO(this->get_logger(), "\033[1;32m---->\033[0m Map Loader Started.");

  InitIO();

  InitTfParams();

  map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(points_map_topic_, 10);

  sensor_msgs::msg::PointCloud2 msgMap;
  pcl::PointCloud<pcl::PointXYZI> cloudMap;
  
  if (!LoadPcd(cloudMap))
  {
    RCLCPP_INFO(this->get_logger(), "load map failed, exit.");
    exit(-1);
  }

  TransformMap(cloudMap, msgMap);

  // if (!LoadPcd(msgMap))
  // {
  //   RCLCPP_INFO(this->get_logger(), "load map failed, exit.");
  //   exit(-1);
  // }

  if (msgMap.width != 0)
  {
    msgMap.header.frame_id = "map";
    map_pub_->publish(msgMap);
    
    RCLCPP_INFO_STREAM(this->get_logger(), "publish map points, map points size: " << msgMap.width);
  }
}

void MapLoader::InitIO()
{
  pcd_file_path_ = "/home/jackie/workspace/colcon_ws/src/ndt_localizer/map/kaist02_binary.pcd";
  points_map_topic_ = "points_map";

  map_path_list_.push_back(pcd_file_path_);
  
  RCLCPP_INFO_STREAM(this->get_logger(), "pcd file path: " << pcd_file_path_ << ", " << "ndt_map_topic: " << points_map_topic_);
}

void MapLoader::InitTfParams()
{
  map_transform_ = Eigen::Affine3d::Identity();
  Eigen::Vector3d tf_trans = map_transform_.translation();
  Eigen::Vector3d tf_euler = map_transform_.linear().eulerAngles(0, 1, 2);
  RCLCPP_INFO_STREAM(this->get_logger(),  "map transform x: " << tf_trans(0) << " y: " << tf_trans(1) << " z: " << tf_trans(2)
                                       << " roll: " << tf_euler(0) << " pitch: " << tf_euler(1) << " yaw: " << tf_euler(2));
}

bool MapLoader::LoadPcd(sensor_msgs::msg::PointCloud2& msgMap)
{
  sensor_msgs::msg::PointCloud2 part;
  for (const std::string &path : map_path_list_)
  {
    if (msgMap.width == 0) 
    {
      if (pcl::io::loadPCDFile(path, msgMap) == -1)
      {
        RCLCPP_INFO_STREAM(this->get_logger(), "load map failed, path: " << path);
        return false;
      }
    }
    else 
    {
      if (pcl::io::loadPCDFile(path, part) == -1)
      {
        RCLCPP_INFO_STREAM(this->get_logger(), "load map failed, path: " << path);
        return false;
      }

      msgMap.data.insert(msgMap.data.end(), part.data.begin(), part.data.end());
      msgMap.width += part.width;
      msgMap.row_step += part.row_step;
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "load map success, path: " << path);
    if (!rclcpp::ok()) break;
  }

  return true;
}

bool MapLoader::LoadPcd(pcl::PointCloud<pcl::PointXYZI>& cloudMap)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPart(new pcl::PointCloud<pcl::PointXYZI>);
  for (const std::string& path : map_path_list_)
  {
    cloudPart->clear();
    if (pcl::io::loadPCDFile(path, *cloudPart) == -1)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "load map failed, path: " << path);
      return false;
    }

    cloudMap += *cloudPart;
    
    RCLCPP_INFO_STREAM(this->get_logger(), "load map success, path: " << path);
    if (!rclcpp::ok()) break;
  }

  return true;
}

void MapLoader::SavePcd(const pcl::PointCloud<pcl::PointXYZI>& cloudMap)
{
  pcl::io::savePCDFileBinary("/home/jackie/workspace/colcon_ws/src/ndt_localizer/map/kaist02_binary.pcd", cloudMap);
}

void MapLoader::TransformMap(pcl::PointCloud<pcl::PointXYZI>& cloudInMap, sensor_msgs::msg::PointCloud2& msgOutMap)
{
  pcl::transformPointCloud(cloudInMap, cloudInMap, map_transform_.matrix());
  pcl::toROSMsg(cloudInMap, msgOutMap);

  SavePcd(cloudInMap);
}

} // namesapce ndt

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto map_loader_node = std::make_shared<ndt::MapLoader>("map_loader");
  rclcpp::spin(map_loader_node);
  rclcpp::shutdown();
}