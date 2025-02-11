#ifndef PCD2OCTOMAP__PCD2OCTOMAP_HPP_
#define PCD2OCTOMAP__PCD2OCTOMAP_HPP_

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <octomap/octomap.h>

class Pcd2Octomap : public rclcpp::Node
{
public:
  explicit Pcd2Octomap(const rclcpp::NodeOptions & options);
  void convert();

private:
  // Parameters
  std::string pcd_path_;
  std::string output_octmap_name_;
  double resolution_;
};

#endif  // PCD2OCTOMAP__PCD2OCTOMAP_HPP_
