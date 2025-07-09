#include "octomap.hpp"
#include "box_merge.hpp"
#include "color_info.hpp"
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
#include <cstddef>
#include <cstring>

using namespace custom_geometry;

Item::Item() {
    x = 0;
    y = 0;
    z = 0;
}


Item::Item(double x_, double y_, double z_) {
    x = x_;
    y = y_;
    z = z_;
}


Item& Item::operator =(const Item& item) {
    x = item.x;
    y = item.y;
    z = item.z;

    return *this;
}

void Item::print() const {
    std::cout << "Item x = " << x << ", y = " << y << ", z = " << z << std::endl;
}

bool Item::operator ==(Item& item) const {
    return round(x*1000) == round(item.x*1000) && round(y*1000) == round(item.y*1000) && round(z*1000) == round(item.z*1000);
}


// TODO: do the sorting
static bool compare(Item &a, Item &b, std::string axis) {

    /// x, y, z
    if (a.x != b.x) {
        return a.x < b.x;
    }

    if (a.x == b.x && a.y != b.y) {
        return a.y < b.y;
    }

    if (a.x == b.x && a.y == b.y && a.z != b.z) {
        return a.z < b.z;
    }

    return a.x == b.x && a.y == b.y && a.z == b.z;
}


Octomap::Octomap(double delta_resolution_, pcl::PointCloud<pcl::PointXYZ> full_cloud_, Eigen::Vector3d position_) {
    resolution = delta_resolution_ * 2;
    delta_resolution = delta_resolution_;
    full_cloud = full_cloud_;
    position = position_;
}


Octomap::Octomap(double delta_resolution_, 
                 Eigen::Vector3d position_,
                 bool generate_,
                 std::string obstacles_name,
                 bool scene_save_ply_, 
                 std::string scene_save_path_,
                 bool octomap_merge_box_ = true) {
    
    resolution = delta_resolution_ * 2;
    delta_resolution = delta_resolution_;
    position = position_;
    generate = generate_;
    scene_save_ply = scene_save_ply_;
    scene_save_path = scene_save_path_;
    octomap_merge_box = octomap_merge_box_;

    if (generate) {
        GeneratePointCloud cloud(obstacles_name, delta_resolution, scene_save_ply, scene_save_path);
        full_cloud = cloud.point_cloud;
    }
}


int Octomap::getIndexItem(std::vector<Item> coords, Item item) {
    int index = -1;

    for (int i = 0; i < coords.size(); i++) {
        // std::cout << "try find - " << std::endl;
        // item.print();
        // coords[i].print();
        if (coords[i] == item) {
            index = i;
            break;
        }
    }

    return index;
}


Item Octomap::getMinValue(std::vector<Item> coords) {
    Item item(10000, 10000, 10000);

    for (int i = 0; i < coords.size(); i++) {
        if ((coords[i].x + coords[i].y + coords[i].z) <= (item.x + item.y + item.z)) {
            item = coords[i];
        }
    }

    return item;
}


ColorInfo Octomap::generateColor() {
    double r = ((double)rand()) / RAND_MAX;
    double g = ((double)rand()) / RAND_MAX;
    double b = ((double)rand()) / RAND_MAX;

    ColorInfo color("", r, g, b, 1);

    return color;
}


pcl::PointCloud<pcl::PointXYZ> Octomap::getFullCloud() {
    return full_cloud;
}


std::vector<BoxMerge> Octomap::createOctomap(std::string axis) {
    
    std::vector<BoxMerge> boxes;

    auto ot = std::make_shared<octomap::OcTree>(resolution);

    for (auto& point : full_cloud.points) {
        ot->updateNode(point.x, point.y, point.z, true, true);
    }

    ot->updateInnerOccupancy();
    ot->toMaxLikelihood();

    std::cout << "Octomap size = " << ot->size() << std::endl;
    std::cout << "Octomap Nodes = " << ot->calcNumNodes() << std::endl;
    bool res = ot->inBBX(octomap::point3d(0, 0, 0));
    std::cout << "Point (0, 0, 0) in octomap = " << (res ? "Yes" : "No") << std::endl;

    std::vector<Item> coords;

    for (octomap::OcTree::iterator it = ot->begin(); it != ot->end(); ++it) {
        double x = it.getX();
        double y = it.getY();
        double z = it.getZ();

        Item item(x, y, z);
        coords.push_back(item);
    }

    // TODO: Sorting by coordinates to merge
    // std::sort(coords.begin(), coords.end(), boost::bind(compare, _1, _2, axis));

    int step = 0;

    if (coords.size() == 0) {
        return boxes;
    }

    // return as octomap
    if (!octomap_merge_box) {
        for (const auto& point : coords) {
            ot->updateNode(point.x, point.y, point.z, true, true);

            double length = resolution;
            double width = resolution;
            double height = resolution;
            
            BoxMerge box(length, width, height, point.x, point.y, point.z);
            boxes.push_back(box);
        }

        ot->updateInnerOccupancy();
        ot->toMaxLikelihood();

        return boxes;
    }

    Item tmp = getMinValue(coords);
    Item a = tmp;
    Item item_tmp = tmp;

    double x_tmp = a.x, y_tmp = a.y, z_tmp = a.z;

    const char *axises = axis.data();
    size_t axises_size = strlen(axises);

    while (coords.size() > 0) {

        bool found_box = false;

        step++;

        int x_count = 0;
        int y_count = 0;
        int z_count = 0;


        // merging by 1 axis
        if (axises_size >= 1) {

            // by X
            if (axises[0] == 'x') {

                while (true) {
                    
                    Item item(x_tmp, y_tmp, z_tmp);
                    // item.print();
                    int index = getIndexItem(coords, item);

                    if (index > -1) {
                        item_tmp = item;
                        x_count++;
                        found_box = true;

                    } else {
                        x_tmp = a.x;
                        y_tmp += resolution;
                        break;
                    }

                    x_tmp += resolution;
                }

            // by Y
            } else if (axises[0] == 'y') {

                int y_count = 0;
                int z_count = 0;

                while (true) {
                    
                    Item item(x_tmp, y_tmp, z_tmp);
                    int index = getIndexItem(coords, item);

                    if (index > -1) {
                        item_tmp = item;
                        y_count++;
                        found_box = true;

                    } else {
                        y_tmp = a.y;
                        z_tmp += resolution;
                        break;
                    }

                    y_tmp += resolution;
                }

            // by Z
            } else if (axises[0] == 'z') {
                int y_count = 0;
                int z_count = 0;

                while (true) {
                    
                    Item item(x_tmp, y_tmp, z_tmp);
                    int index = getIndexItem(coords, item);

                    if (index > -1) {
                        item_tmp = item;
                        z_count++;
                        found_box = true;

                    } else {
                        z_tmp = a.z;
                        y_tmp += resolution;
                        break;
                    }

                    z_tmp += resolution;
                }
            }
        }



        // merging by y
        // TODO add merge by X, Z
        if (found_box && axises_size >= 2) {
            y_count = 1; // y min 1

            while (true) {
                Item item(x_tmp, y_tmp, z_tmp);

                // check that there is a neighbor along the entire length of x

                std::vector<int> ids;

                Item item_tmp_(0, 0, 0);

                for (int i = 0; i < x_count; i++) {
                    Item item(x_tmp, y_tmp, z_tmp);
                    int index = getIndexItem(coords, item);

                    if (index > -1) {
                        item_tmp_ = item;
                        ids.push_back(index);
                        x_tmp += resolution;

                        y_count++;

                    } else {
                        ids.clear();
                        break;
                    }
                }

                if (x_count != 0 && ids.size() == x_count) {
                    item_tmp = item_tmp_;

                    x_tmp = a.x;
                    y_tmp += resolution;
                } else {
                    y_tmp = a.y;
                    break;
                }
            }
        }


        // merging by z
        // TODO add merge by X, Y
        if (found_box  && axises_size >= 3) {
            while (true) {
                std::vector<int> ids;
                Item item_tmp_(0, 0, 0);
                bool next_layer = true;

                for (int xt = 0; xt < x_count; xt++) {
                    for (int yt = 0; yt < y_count; yt++) {

                        Item item(x_tmp + xt * resolution, y_tmp + yt * resolution, z_tmp);
                        int index = getIndexItem(coords, item);

                        if (index > -1) {
                            ids.push_back(index);
                        } else {
                            next_layer = false;
                            break;
                        }
                    }

                    if (!next_layer) break;
                }

                if (ids.size() == (x_count*y_count)) {
                    z_tmp += resolution;
                    item_tmp.z = z_tmp;
                } else {
                    ids.clear();
                    break;
                }
            }
        }


        for (auto it = coords.begin(); it != coords.end(); /* empty */) {
            if (it->x >= (a.x - (resolution / 10)) && it->x <= (item_tmp.x + (resolution / 10)) &&
                it->y >= (a.y - (resolution / 10)) && it->y <= (item_tmp.y + (resolution / 10)) &&
                it->z >= (a.z - (resolution / 10)) && it->z <= (item_tmp.z + (resolution / 10))) {
                
                it = coords.erase(it); // Deleting an element and moving the iterator to the next one
            } else {
                ++it; // Moving the iterator to the next element
            }
        }

        double length = (item_tmp.y - a.y) + resolution;
        double width = (item_tmp.x - a.x) + resolution;
        double height = (item_tmp.z - a.z) + resolution;

        double x = (item_tmp.x + a.x) / 2.0 + position(0);
        double y = (item_tmp.y + a.y) / 2.0 + position(1);
        double z = (item_tmp.z + a.z) / 2.0 + position(2);

        if (height > resolution) {
            height -= resolution;
            z -= delta_resolution;
        }

        BoxMerge box(length, width, height, x, y, z);
        boxes.push_back(box);

        tmp = getMinValue(coords);
        a = tmp;

        x_tmp = a.x, y_tmp = a.y, z_tmp = a.z;

        item_tmp.x = x_tmp;
        item_tmp.y = y_tmp;
        item_tmp.z = z_tmp;

        if (step > 1000) {
            break;
        }
    }

    return boxes;
}

