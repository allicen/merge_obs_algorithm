#include <vector>
#include <string>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "octomap.hpp"
#include "box_merge.hpp"


// Half of the side of the octomap cube
double delta_resolution = 0.1;

// Position of the octomap relative to base_link of robot
Eigen::Vector3d octomap_position = Eigen::Vector3d(0.9, -0.5, 0.0);

// Name of the obstacle
std::string name_obs = "table";

// Order of combining cubes along the axes
std::string axies = "xyz";

// If you need to generate a scene, set it to true
bool scene_obstacle_generate = false;

// If you need to save the obstacle in the format .ply, set true
bool scene_save_ply = false;

// Specify the path to save .ply file
std::string scene_save_path = "";

// Name of the manipulator joint group from the Moveit settings
std::string group = "arm";

// Name of the link that the obstacles will be added to
std::string base_link = "world";

// If you need to generate octomap, set true
bool generate = true;



void pointCloudFilter(pcl::PointCloud<pcl::PointXYZ> &full_cloud) {
  ROS_INFO("Pointcloud size points=%d", full_cloud.points.size());

  float depthThreshold = 2.0; // Filtering points within 2.0 from the origin
  float threshold2 = depthThreshold * depthThreshold;
  

  for (int p=0; p < full_cloud.points.size(); ++p) {
    // find the squared distance from the origin.
    float pointDepth2 = (full_cloud.points[p].x * full_cloud.points[p].x) +
                        (full_cloud.points[p].y * full_cloud.points[p].y) + 
                        (full_cloud.points[p].z * full_cloud.points[p].z);

    // remove point if it's within the threshold range
    if (pointDepth2 > threshold2) {
        full_cloud.points[p] = full_cloud.points[full_cloud.points.size()-1];
        full_cloud.points.resize(full_cloud.points.size()-1);
        --p;
    }
  }
  ROS_INFO("Pointcloud after filter size points=%d", full_cloud.points.size());
}



custom_geometry::Octomap getOctomap() {
    if (generate) {
        custom_geometry::Octomap octomap(delta_resolution, octomap_position, scene_obstacle_generate, name_obs, scene_save_ply, scene_save_path);
        return octomap;
    }

    // Create octomap and add it to the local environment
    // Sensor data (PointCloud) to this variable from sensor_msgs
    pcl::PointCloud<pcl::PointXYZ> full_cloud;
    pointCloudFilter(full_cloud);

    // Do not to set the offset
    Eigen::Vector3d position = Eigen::Vector3d(0.0, 0.0, 0.0);

    custom_geometry::Octomap octomap(delta_resolution, full_cloud, position);

    return octomap;
}


int main(int argc, char** argv) {

    custom_geometry::Octomap octomap = getOctomap();

    std::vector<BoxMerge> boxes = octomap.createOctomap(axies);

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    std::vector<moveit_msgs::ObjectColor> object_colors;

    moveit::planning_interface::MoveGroupInterface move_group(group);
    moveit_visual_tools::MoveItVisualTools visual_tools(base_link);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    for(int i = 0; i < boxes.size(); i++) {

        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = move_group.getPlanningFrame();

        // The id of the object is used to identify it.
        std::string link_name = "box_merge_" + std::to_string(i);
        collision_object.id = link_name;

        // Define a box to add to the world.
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = boxes[i].width;
        primitive.dimensions[primitive.BOX_Y] = boxes[i].length;
        primitive.dimensions[primitive.BOX_Z] = boxes[i].height;

        // Define a pose for the box (specified relative to frame_id)
        geometry_msgs::Pose box_pose;
        // box_pose.orientation.w = 1.0;
        box_pose.position.x = boxes[i].x;
        box_pose.position.y = boxes[i].y;
        box_pose.position.z = boxes[i].z;       

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);

        ColorInfo color_ = octomap.generateColor();
        moveit_msgs::ObjectColor object_color;
        object_color.id = link_name;
        object_color.color.r = color_.getR(); 
        object_color.color.g = color_.getG();
        object_color.color.b = color_.getB();
        object_color.color.a = color_.getA();
        object_colors.push_back(object_color);

        collision_object.operation = collision_object.ADD;
        collision_objects.push_back(collision_object);
    }

    planning_scene_interface.addCollisionObjects(collision_objects, object_colors);

    return 0;
}