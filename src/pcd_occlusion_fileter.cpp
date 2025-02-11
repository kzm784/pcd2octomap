#include "pcd2octomap/pcd_occlusion_fileter.hpp"

PcdOcclusionFilter::PcdOcclusionFilter(const rclcpp::NodeOptions & options)
  : Node("pcd_occlusion_filter", options)
{
  // Declare parameters
  this->declare_parameter<std::string>("pcd_path", "");
  this->declare_parameter<double>("resolution", 0.05);
  this->declare_parameter<float>("position_x", 0.0);
  this->declare_parameter<float>("position_y", 0.0);
  this->declare_parameter<float>("position_z", 0.0);

  // Retrieve parameters
  this->get_parameter("pcd_path", pcd_path_);
  this->get_parameter("resolution", resolution_);
  this->get_parameter("position_x", position_x_);
  this->get_parameter("position_y", position_y_);
  this->get_parameter("position_z", position_z_);

  // Log retrieved parameters
  RCLCPP_INFO(this->get_logger(), "pcd_path: %s", pcd_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "resolution: %f", resolution_);
  RCLCPP_INFO(this->get_logger(), "position_x: %f", position_x_);
  RCLCPP_INFO(this->get_logger(), "position_y: %f", position_y_);
  RCLCPP_INFO(this->get_logger(), "position_z: %f", position_z_);

  // Execute conversion from PCD to OctoMap with occlusion filtering and dilation
  convert();

  // Shutdown the node after processing is complete
  rclcpp::shutdown();
}

void PcdOcclusionFilter::convert()
{
  // 1. Load the point cloud from the original PCD file
  pcl::PointCloud<pcl::PointXYZRGBA> cloud;
  if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(pcd_path_, cloud) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file: %s", pcd_path_.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Successfully loaded PCD file. Number of points: %zu", cloud.points.size());

  // 2. Create an OctoMap OcTree with the specified resolution and insert all points from the cloud
  octomap::OcTree tree(resolution_);
  for (const auto & point : cloud.points) {
    tree.updateNode(octomap::point3d(point.x, point.y, point.z), true);
  }
  tree.updateInnerOccupancy();

  // 3. Perform occlusion filtering using ray-casting from the specified observation point
  pcl::PointCloud<pcl::PointXYZRGBA> visible_cloud;
  // Set the observation point using the provided parameters
  octomap::point3d observation_point(position_x_, position_y_, position_z_);
  const double epsilon = 0.01;
  for (const auto & point : cloud.points) {
    octomap::point3d target(point.x, point.y, point.z);
    octomap::point3d vec = target - observation_point;
    double distance = vec.norm();

    // If the point is nearly identical to the observation point, consider it visible
    if (distance < std::numeric_limits<double>::epsilon()) {
      visible_cloud.points.push_back(point);
      continue;
    }

    // Normalize the direction vector (operator/ is not defined for octomap::point3d)
    octomap::point3d direction = vec * (1.0 / distance);

    octomap::point3d hit;
    // Cast a ray from the observation point to the target point
    // The maximum range is set to the distance between the observation point and the target
    bool hit_found = tree.castRay(observation_point, direction, hit, distance);
    if (hit_found) {
      double hit_distance = (hit - observation_point).norm();
      // If an obstacle is encountered before reaching the target, skip this point (occluded)
      if (hit_distance < distance - epsilon) {
        continue;
      }
    }
    visible_cloud.points.push_back(point);
  }
  RCLCPP_INFO(this->get_logger(), "Visible points after occlusion filtering: %zu", visible_cloud.points.size());

  // 4. Include nearby points (dilation process)
  // Here, we add points from the original cloud that are within a radius of 0.5 m from any visible point
  const double dilation_radius = 0.5;       // Radius for neighborhood search
  const double duplicate_threshold = 1e-6;    // Threshold for duplicate detection

  // Build a KD-Tree for fast neighborhood search in the visible cloud
  pcl::KdTreeFLANN<pcl::PointXYZRGBA> kd_tree;
  kd_tree.setInputCloud(visible_cloud.makeShared());

  // Initialize the extended visible cloud with the current visible cloud
  pcl::PointCloud<pcl::PointXYZRGBA> extended_visible_cloud = visible_cloud;

  for (const auto & point : cloud.points) {
    // Search for neighbors within the dilation radius using the KD-Tree
    std::vector<int> indices;
    std::vector<float> sqr_distances;
    if (kd_tree.radiusSearch(point, dilation_radius, indices, sqr_distances) > 0) {
      // Check if the point already exists in the visible cloud (if nearly identical, it's a duplicate)
      bool already_exists = false;
      for (size_t i = 0; i < indices.size(); ++i) {
        if (sqr_distances[i] < duplicate_threshold * duplicate_threshold) {
          already_exists = true;
          break;
        }
      }
      if (!already_exists) {
        extended_visible_cloud.points.push_back(point);
      }
    }
  }
  RCLCPP_INFO(this->get_logger(), "Extended visible points (after dilation): %zu", extended_visible_cloud.points.size());

  // 5. Create a merged point cloud for visualization:
  //    - Original cloud points are colored blue
  //    - Extended visible points are colored red
  pcl::PointCloud<pcl::PointXYZRGBA> merged_cloud;
  
  // Color original cloud points blue (R=0, G=0, B=255, A=255)
  for (auto point : cloud.points) {
    point.r = 0;
    point.g = 0;
    point.b = 255;
    point.a = 255;
    merged_cloud.points.push_back(point);
  }
  
  // Color extended visible points red (R=255, G=0, B=0, A=255)
  for (auto point : extended_visible_cloud.points) {
    point.r = 255;
    point.g = 0;
    point.b = 0;
    point.a = 255;
    merged_cloud.points.push_back(point);
  }
  
  RCLCPP_INFO(this->get_logger(), "Merged point cloud contains %zu points.", merged_cloud.points.size());

  // 6. Save the merged point cloud to a file named "occlusion_filtered_result.pcd"
  if (pcl::io::savePCDFileBinary("occlusion_filtered_result.pcd", merged_cloud) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to save occlusion filtered point cloud.");
  } else {
    RCLCPP_INFO(this->get_logger(), "Occlusion filtered point cloud saved as occlusion_filtered_result.pcd");
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(PcdOcclusionFilter)
