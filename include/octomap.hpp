#pragma once
#ifndef CUSTOM_GEOMETRY_OCTOMAP_HPP
#define CUSTOM_GEOMETRY_OCTOMAP_HPP

#include <Eigen/Core>

#include <memory>
#include <iostream>
#include <vector>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_ros/publisher.h>
#include <pcl/io/ply_io.h>

#include "box_merge.hpp"
#include "color_info.hpp"
#include "generate_point_cloud.hpp"

namespace custom_geometry
{
class Item
{
public:
    Item();
    Item(double, double, double);
    ~Item() = default;

    Item& operator =(const Item&);
    bool operator ==(Item&) const;

    void print() const;

    double x;
    double y;
    double z;
};

class Octomap
{
public:
  Octomap(double, Eigen::Vector3d, bool, std::string, bool, std::string, bool);
  Octomap(double, pcl::PointCloud<pcl::PointXYZ>, Eigen::Vector3d);
  ~Octomap() = default;
  Octomap(const Octomap&) = default;
  Octomap& operator=(const Octomap&) = default;
  Octomap(Octomap&&) = default;
  Octomap& operator=(Octomap&&) = default;

  std::vector<BoxMerge> createOctomap(std::string);
  ColorInfo generateColor();
  pcl::PointCloud<pcl::PointXYZ> getFullCloud();
  int getIndexItem(std::vector<Item>, Item);
  Item getMinValue(std::vector<Item>);

private:
    double resolution;
    double delta_resolution;
    Eigen::Vector3d position = Eigen::Vector3d(0.0, 0.0, 0.0);
    pcl::PointCloud<pcl::PointXYZ> full_cloud;
    bool generate = false;
    bool scene_save_ply = false;
    std::string scene_save_path = "";
    bool octomap_merge_box = true;
};

}  // namespace custom_geometry

#endif