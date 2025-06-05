#include "generate_point_cloud.hpp"

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

#include <octomap/octomap.h>

#include <memory>
#include <iostream>
#include <vector>
#include <string>

using namespace custom_geometry;
    
GeneratePointCloud::GeneratePointCloud(std::string obstacles_name, double delta_resolution_, bool save_file_, std::string path_){
    delta_resolution = delta_resolution_;
    resolution = delta_resolution_ * 2;
    save_file = save_file_;

    char* var_value = std::getenv("HOME_USER");
    path = std::string("/home/") + std::string(var_value) + path_;

    if(obstacles_name == "table") generateTable();
    if(obstacles_name == "box") generateBox();
    if(obstacles_name == "chair") generateChair();
    if(obstacles_name == "wall") generateWall();
    if(obstacles_name == "cabinet") generateCabinet();
    if(obstacles_name == "floor") generateFloor();

    if (save_file) {
        std::string file_path = path + "/" + obstacles_name + ".ply";
        std::cout << "PLY FILE SAVE path = " << file_path << std::endl;
        pcl::io::savePLYFile(file_path, point_cloud);
    }
}

// Generate a table
void GeneratePointCloud::generateTable() {
    double delta = delta_resolution; // side of the cube =* 2

    auto length_ = 1.2; // table length
    auto height_ = 0.7; // table height
    auto width_ = 0.8; // table width

    auto length = static_cast<int>(length_ / delta);
    auto height = static_cast<int>(height_ / delta);
    auto width = static_cast<int>(width_ / delta);

    for (int x = 0; x < length; ++x) {
        for (int y = 0; y < width; ++y) {
            for (int z = 0; z < height; ++z) {
                // Table top
                point_cloud.push_back(pcl::PointXYZ(static_cast<float>(x * delta),
                static_cast<float>(y * delta),
                static_cast<float>(height_)));

                // Table legs
                if (x == 0 && y == 0 || x == 0 && y == width-1 || x == length-1 && y == 0 || x == length-1 && y == width-1) {
                    point_cloud.push_back(pcl::PointXYZ(static_cast<float>(x * delta),
                    static_cast<float>(y * delta),
                    static_cast<float>(z * delta)));
                }
            }
        }
    } 
}


// Generate a trash can
void GeneratePointCloud::generateBox() {
    double delta = delta_resolution; // side of the cube =* 2

    auto length_ = 0.5; // length
    auto height_ = 0.5; // height
    auto width_ = 0.5; // width

    auto length = static_cast<int>(length_ / delta);
    auto height = static_cast<int>(height_ / delta);
    auto width = static_cast<int>(width_ / delta);

    for (int x = 0; x < length; ++x) {
        for (int y = 0; y < width; ++y) {
            for (int z = 0; z < height; ++z) {
                // bottom
                point_cloud.push_back(pcl::PointXYZ(static_cast<float>(x * delta),
                static_cast<float>(y * delta),
                static_cast<float>(0)));
                
                // walls
                if (x == 0 || y == 0 || x == length-1 || y == width-1) { //|| x == length-1 || y == width-1) {
                    point_cloud.push_back(pcl::PointXYZ(static_cast<float>(x * delta),
                    static_cast<float>(y * delta),
                    static_cast<float>(z * delta)));
                }
            }
        }
    }
}


// Generate a chair
void GeneratePointCloud::generateChair() {
    double delta = delta_resolution; // side of the cube =* 2

    auto length_ = 0.4; // seat length
    auto height_ = 0.4; // seat height
    auto height_seat_ = 1.2; // backrest height
    auto width_ = 0.4; // seat width

    auto length = static_cast<int>(length_ / delta);
    auto height = static_cast<int>(height_ / delta);
    auto width = static_cast<int>(width_ / delta);
    auto height_seat = static_cast<int>(height_seat_ / delta);

    for (int x = 0; x < length; ++x) {
        for (int y = 0; y < width; ++y) {
            for (int z = 0; z < height; ++z) {
                // the seat
                point_cloud.push_back(pcl::PointXYZ(static_cast<float>(x * delta),
                static_cast<float>(y * delta),
                static_cast<float>(height_)));

                // Legs
                if (x == 0 && y == 0 || x == 0 && y == width-1 || x == length-1 && y == 0 || x == length-1 && y == width-1) {
                    point_cloud.push_back(pcl::PointXYZ(static_cast<float>(x * delta),
                    static_cast<float>(y * delta),
                    static_cast<float>(z * delta)));
                }
            }
        }
    }

    // The back
    for (int x = 0; x < length; ++x) {
        for (int y = 0; y < width; ++y) {
            for (int z = height; z < height_seat; ++z) {
                point_cloud.push_back(pcl::PointXYZ(static_cast<float>(length_-delta),
                static_cast<float>(y * delta),
                static_cast<float>(z * delta)));
            }
        }
    }
}

// Generate a wall
void GeneratePointCloud::generateWall() {
    double delta = delta_resolution; // side of the cube =* 2

    auto length_ = resolution; // distance of the wall
    auto height_ = 2.0; // wall height
    auto width_ = 2.0; // wall width

    auto length = static_cast<int>(length_ / delta);
    auto height = static_cast<int>(height_ / delta);
    auto width = static_cast<int>(width_ / delta);

    for (int x = 0; x < length; ++x) {
        for (int y = 0; y < width; ++y) {
            for (int z = 0; z < height; ++z) {
                point_cloud.push_back(pcl::PointXYZ(static_cast<float>(length_),
                static_cast<float>(y * delta),
                static_cast<float>(z * delta)));
            }
        }
    }
}

// Generate a cabinet
void GeneratePointCloud::generateCabinet() {
    double delta = delta_resolution; // side of the cube =* 2

    auto length_ = 0.4; // cabinet depth
    auto height_ = 2.0; // cabinet height
    auto width_ = 1.0; // cabinet width
    auto count_shelf = 4;

    auto length = static_cast<int>(length_ / delta);
    auto height = static_cast<int>(height_ / delta);
    auto width = static_cast<int>(width_ / delta);


    int heigth_shelf = static_cast<int>(height / (count_shelf+1));
    std::vector<int> z_tmp;

    for (int i = heigth_shelf; i <= height; i++) {
        z_tmp.push_back(i);
        i += heigth_shelf;
    }

    for (int x = 0; x < length; ++x) {
        for (int y = 0; y < width; ++y) {
            for (int z = 0; z < height; ++z) {
                // cabinet body
                if (z == height -1 || z == 0 || y == width - 1 || y == 0 || x == length - 1) {
                    point_cloud.push_back(pcl::PointXYZ(static_cast<float>(x * delta),
                    static_cast<float>(y * delta),
                    static_cast<float>(z * delta)));
                }

                // shelves in the closet
                if (std::find(z_tmp.begin(), z_tmp.end(), z) != z_tmp.end()) {
                    point_cloud.push_back(pcl::PointXYZ(static_cast<float>(x * delta),
                    static_cast<float>(y * delta),
                    static_cast<float>(z * delta)));

                }
            }
        }
    }
}



void GeneratePointCloud::generateFloor() {
    double delta = delta_resolution; // side of the cube =* 2

    auto length_ = 10.0; // length
    auto height_ = 0.1; // height
    auto width_ = 10.0; // width

    auto length = static_cast<int>(length_ / delta);
    auto height = static_cast<int>(height_ / delta);
    auto width = static_cast<int>(width_ / delta);

    for (int x = 0; x < length; ++x) {
        for (int y = 0; y < width; ++y) {
            for (int z = 0; z < height; ++z) {
                point_cloud.push_back(pcl::PointXYZ(
                    static_cast<float>(x * delta),
                    static_cast<float>(y * delta),
                    static_cast<float>(z * delta))
                );
            }
        }
    }
}

