#ifndef PCD2OCTOMAP__PCD_OCCLUSION_FILTER_HPP_
#define PCD2OCTOMAP__PCD_OCCLUSION_FILTER_HPP_

#include <string>
#include <limits> 

#include <octomap/octomap.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <rclcpp/rclcpp.hpp>

class PcdOcclusionFilter : public rclcpp::Node
{
public:
  explicit PcdOcclusionFilter(const rclcpp::NodeOptions & options);
  void convert();

private:
  // Parameters
  std::string pcd_path_;
  double resolution_;
  float position_x_;
  float position_y_;
  float position_z_;
  double epsilon_;
  double dilation_radius_;
  double duplicate_threshold_;
};

#endif  // PCD2OCTOMAP__PCD_OCCLUSION_FILTER_HPP_
