// RRT assignment

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf

#include <iostream>
#include <string>
#include <algorithm>
#include <sstream>
#include <cmath>
#include <vector>
#include <random>
// #include <sstream>
// #include <string>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
// #include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

/// CHECK: include needed ROS msg type headers and libraries

using namespace std;

// Struct defining the RRT_Node object in the RRT tree.
// More fields could be added to thiis struct if more info needed.
// You can choose to use this or not
typedef struct RRT_Node {
    double x, y;
    double cost; // only used for RRT*
    int parent; // index of parent node in the tree vector
    bool is_root = false;
} RRT_Node;


class RRT : public rclcpp::Node {
public:
    RRT();
    virtual ~RRT();
private:

    // TODO: add the publishers and subscribers you need

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tree_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr random_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr next_wp_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr next_wp_marker_pub_;

    // random generator, use this
    std::mt19937 gen;
    std::uniform_real_distribution<> x_dist;
    std::uniform_real_distribution<> y_dist;
    

    // callbacks
    // where rrt actually happens
    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg);
    // updates occupancy grid
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

    // RRT methods
    std::vector<double> sample();
    int nearest(std::vector<RRT_Node> &tree, std::vector<double> &sampled_point);
    RRT_Node steer(RRT_Node &nearest_node, std::vector<double> &sampled_point);
    bool check_collision(RRT_Node &nearest_node, RRT_Node &new_node);
    bool is_goal(RRT_Node &latest_added_node, double goal_x, double goal_y);
    std::vector<RRT_Node> find_path(std::vector<RRT_Node> &tree, RRT_Node &latest_added_node);
    void get_goal(std::vector<std::vector<double>> &reference_wps, std::vector<double> curr_pos, vector<double> &goal);
    void sending_wp(std::vector<RRT_Node> &path, double rel_carx, double rel_cary, double curr_theta, double carx, double cary);

    // RRT* methods
    double cost(std::vector<RRT_Node> &tree, RRT_Node &node);
    double line_cost(RRT_Node &n1, RRT_Node &n2);
    std::vector<int> near(std::vector<RRT_Node> &tree, RRT_Node &node);

    // Visualization methods
    void visualize_tree(std::vector<RRT_Node> &tree);
    void visualize_goal(std::vector<double> goal_pt);
    void visualize_point_on_grid(std::vector<double> rand_pt);
    void visualize_path(std::vector<RRT_Node> path);
    void visualize_next_wp(std::vector<double> next_wp);

    // Reference path waypoints
    vector<vector<double>> reference_wps;

    // Function to parse csv file
    void csv_to_wps(string file_path, vector<vector<double>> &wps);

    // occupancy grid
    nav_msgs::msg::OccupancyGrid occupancy_grid_; // we will make in car frame
    int gridsizeX = 20; //20m x 20m grid
    int gridsizeY = 20;
    double grid_resolution = 0.1; // 10 cm resolution

    int grid_width = static_cast<int>(gridsizeX / grid_resolution);
    int grid_height = static_cast<int>(gridsizeY / grid_resolution);

    int buffer_size = 1;

    int iteration_limit = 500;

    double lookahead_for_goal = 2.0; // meters
};

