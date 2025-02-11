#include "pcd2octomap/pcd2octomap.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

Pcd2Octomap::Pcd2Octomap(const rclcpp::NodeOptions & options)
  : Node("pcd2octomap", options)
{
  // Declare parameters
  this->declare_parameter<std::string>("pcd_path", "");
  this->declare_parameter<std::string>("output_octmap_name", "");
  this->declare_parameter<double>("resolution", 0.05);

  // Get parameters
  this->get_parameter("pcd_path", pcd_path_);
  this->get_parameter("resolution", resolution_);
  this->get_parameter("output_octmap_name", output_octmap_name_);

  // Log retrieved parameters in English
  RCLCPP_INFO(this->get_logger(), "pcd_path: %s", pcd_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "output_octmap_name: %s", output_octmap_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "resolution: %f", resolution_);

  // Execute conversion from PCD to OctoMap
  convert();

  // Shutdown node after processing is complete
  rclcpp::shutdown();
}

void Pcd2Octomap::convert()
{
  // Load the PCD file
  pcl::PointCloud<pcl::PointXYZRGBA> cloud;
  if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(pcd_path_, cloud) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file: %s", pcd_path_.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Successfully loaded PCD file. Number of points: %zu", cloud.points.size());

  // Create an OctoMap tree with the specified resolution
  octomap::OcTree tree(resolution_);

  // Insert each point from the point cloud into the OctoMap (mark as occupied)
  for (const auto & point : cloud.points) {
    tree.updateNode(octomap::point3d(point.x, point.y, point.z), true);
  }

  // Update inner nodes occupancy
  tree.updateInnerOccupancy();

  // Save the OctoMap in binary format
  if (tree.writeBinary(output_octmap_name_)) {
    RCLCPP_INFO(this->get_logger(), "OctoMap has been saved to %s", output_octmap_name_.c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to save OctoMap to %s", output_octmap_name_.c_str());
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(Pcd2Octomap)
