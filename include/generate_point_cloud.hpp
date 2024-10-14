#pragma once
#ifndef GENERATE_POINT_CLOUD_HPP
#define GENERATE_POINT_CLOUD_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen3/Eigen/Eigen>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_ros/publisher.h>
#include <pcl/io/ply_io.h>

#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>

#include <memory>
#include <iostream>
#include <vector>
#include <string>


namespace custom_geometry
{
class GeneratePointCloud {
  public:
    GeneratePointCloud(std::string, double, bool, std::string);

    ~GeneratePointCloud() = default;
    GeneratePointCloud(const GeneratePointCloud&) = default;
    GeneratePointCloud& operator=(const GeneratePointCloud&) = default;
    GeneratePointCloud(GeneratePointCloud&&) = default;
    GeneratePointCloud& operator=(GeneratePointCloud&&) = default;

    // GeneratePointCloud(double);

    // Generate a table
    void generateTable();

    // Generate a trash can
    void generateBox();

    // Generate a chair
    void generateChair();

    // Generate a wall
    void generateWall();

    // Generate a cabinet
    void generateCabinet();

    pcl::PointCloud<pcl::PointXYZ> point_cloud;

    private:
      double resolution = 0.1;
      double delta_resolution = 0.05;

      // Save to a file .ply
      bool save_file = true;

      // Path to the folder to save the files in folder "/home"
      // Default folder name: "merge_obs_algorithm_code"
      std::string path = "merge_obs_algorithm_code";

};
}

#endif