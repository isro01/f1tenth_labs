// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well

#include "rrt/rrt.h"

using namespace std;

// Destructor of the RRT class
RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(): rclcpp::Node("rrt_node"), gen((std::random_device())()) {

    // ROS publishers
    // TODO: create publishers for the the drive topic, and other topics you might need

    // ROS subscribers
    // TODO: create subscribers as you need
    string pose_topic = "ego_racecar/odom";
    string scan_topic = "/scan";
    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      pose_topic, 1, std::bind(&RRT::pose_callback, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, 1, std::bind(&RRT::scan_callback, this, std::placeholders::_1));
    occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("f1_occupancy_grid", 1);
    tree_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("viz_rrt_tree", 1);
    goal_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("viz_goal", 1);
    random_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("viz_random", 1);
    path_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("viz_path", 1);
    next_wp_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("next_wp_tofollow", 1);
    next_wp_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("next_wp_marker", 1);

    // Get the reference path from csv file
    string csv_path = "/home/vulcan/f1tenth/labs/waypoints/logs/blocked_wps.csv";
    csv_to_wps(csv_path, RRT::reference_wps);
      
    // TODO: create a occupancy grid -> lets use flattened array for now, occupancy msh from nav_msg
    RRT::occupancy_grid_.header.frame_id = "ego_racecar/base_link";
    RRT::occupancy_grid_.info.resolution = RRT::grid_resolution;
    RRT::occupancy_grid_.info.width = RRT::grid_width;
    RRT::occupancy_grid_.info.height = RRT::grid_height;
    RRT::occupancy_grid_.info.origin.position.x = -RRT::gridsizeX / 2.0;
    RRT::occupancy_grid_.info.origin.position.y = -RRT::gridsizeY / 2.0;

    // Initialize the occupancy grid with -1 (unknown)
    RRT::occupancy_grid_.data.resize(RRT::grid_width * RRT::grid_height, -1);
    


    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "Created new RRT Object.");
}

void RRT::csv_to_wps(string file_path, vector<vector<double>> &wps) {

    ifstream file(file_path);

    if (!file.is_open()) {
        cout << "Check file path for waypoints" << endl;
        return;
    }

    string line;
    while (getline(file, line)) {
        stringstream ss(line);
        string data;
        vector<double> row;
        while (getline(ss, data, ',')) {
            row.push_back(stof(data));
        }
        wps.push_back(row);
    }

}

void RRT::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
    // The scan callback, update your occupancy grid here
    // Args:
    //    scan_msg (*LaserScan): pointer to the incoming scan message
    // Returns:
    //

    

    // TODO: update your occupancy grid
    
    // 1. we reset the entire grid to unknown (-1)
    std::fill(RRT::occupancy_grid_.data.begin(), RRT::occupancy_grid_.data.end(), -1);

    // 2. we convert (range, theta) to (x, y) in car frame
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
        double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
        double range = scan_msg->ranges[i];

        // if (range > scan_msg->range_max || range < scan_msg->range_min) {
        //     continue; // skip invalid ranges
        // }
        // cout << "Max range: " << scan_msg->range_max << endl;
        // cout << "Min range: " << scan_msg->range_min << endl;
        if (range< scan_msg->range_min) {
            continue; // skip invalid ranges
        }
        if (range > 10) {
            range = 10.5;
        }

        double x = range * cos(angle);
        double y = range * sin(angle);

        // Convert (x, y) to grid coordinates
        int grid_x = static_cast<int>((x + RRT::gridsizeX / 2.0) / RRT::grid_resolution);
        int grid_y = static_cast<int>((y + RRT::gridsizeY / 2.0) / RRT::grid_resolution);

        
        // using ray tracing, cells from laser to the range would be free
        // use bresenham's line algorithm to mark free cells
        // https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
        int x0 = static_cast<int>(RRT::gridsizeX / 2.0 / RRT::grid_resolution);
        int y0 = static_cast<int>(RRT::gridsizeY / 2.0 / RRT::grid_resolution);
        int x1 = grid_x;
        int y1 = grid_y;
        int dx = std::abs(x1 - x0);
        int dy = std::abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;
        // while (x0 != x1 || y0 != y1) {
            while (abs(x0 - x1) > 2 || abs(y0 - y1) > 2) {
                int index = y0 * RRT::grid_width + x0;
                if (index >= 0 && index < RRT::occupancy_grid_.data.size() && RRT::occupancy_grid_.data[index] != 1) {
                // Mark the cell as free (0)
                RRT::occupancy_grid_.data[index] = 0;
            }
            int e2 = 2 * err;
            if (e2 > -dy) {
                err = err - dy;
                x0 = x0 + sx;
            }
            if (e2 < dx) {
                err = err + dx;
                y0 = y0 + sy;
            }
        }

        // So all these cells would be edges of occupied regions
        if (grid_x >= 0 && grid_x < RRT::grid_width && grid_y >= 0 && grid_y < RRT::grid_height) {
            // Mark the cell as occupied (1)
            RRT::occupancy_grid_.data[grid_y * RRT::grid_width + grid_x] = 1;
        }

        // Optionally, you can also mark the cells around the occupied cell
        // to create a buffer around the obstacle
        for (int dx = -RRT::buffer_size; dx <= RRT::buffer_size; ++dx) {
            for (int dy = -RRT::buffer_size; dy <= RRT::buffer_size; ++dy) {
                int neighbor_x = grid_x + dx;
                int neighbor_y = grid_y + dy;

                if (neighbor_x >= 0 && neighbor_x < RRT::grid_width &&
                    neighbor_y >= 0 && neighbor_y < RRT::grid_height) {
                    // Mark the neighboring cell as occupied (1)
                    RRT::occupancy_grid_.data[neighbor_y * RRT::grid_width + neighbor_x] = 1;
                }
            }
        }
    }

    // 3. publish the occupancy grid
    occupancy_grid_pub_->publish(RRT::occupancy_grid_);

}

void RRT::get_goal(vector<vector<double>> &ref_wps, vector<double> curr_pos, vector<double> &goal) {

    double pos_x = curr_pos[0];
    double pos_y = curr_pos[1];
    vector<double> goal_temp = {0, 0};
    bool goal_found = false;

    for (int j =0; j < ref_wps.size(); j++) {
        double dist = sqrt(pow(pos_x - ref_wps[j][0], 2) + pow(pos_y - ref_wps[j][1], 2));
        if (abs(dist - RRT::lookahead_for_goal) < 0.1) {
            double angle = atan2(ref_wps[j][1] - pos_y, ref_wps[j][0] - pos_x);
            if (cos(angle - curr_pos[2]) > 0) {
                goal = {ref_wps[j][0], ref_wps[j][1]};
                goal_found = true;
                break;
            }
        }
    }

    if (!goal_found) {
        cout << "No goal found at lookahead distance" << endl;
        goal = {goal_temp[0], goal_temp[1]};
    }


}

double get_heading(const geometry_msgs::msg::Quaternion &q)
{
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(tf_q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

void RRT::pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg) {
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    // Returns:
    //
    int randomx = RRT::grid_width / 2 + 20 ;
    int randomy = RRT::grid_height / 2 ;
    // visualize_point_on_grid({randomx, randomy});
    // cout << "Occ grid value: " << RRT::occupancy_grid_.data[randomy * RRT::grid_width + randomx] << endl;
    // cout << "Occ grid value: " << RRT::occupancy_grid_.data[0] << endl;
    

    // Also set the goal
    double car_x = pose_msg->pose.pose.position.x;
    double car_y = pose_msg->pose.pose.position.y;
    double curr_theta = get_heading(pose_msg->pose.pose.orientation);

    // cout << "Orientation of car: " << curr_theta << endl;

    vector<double> goal = {0, 0};
    get_goal(RRT::reference_wps, {car_x, car_y, curr_theta}, goal);
    double goal_x = goal[0];
    double goal_y = goal[1];

    bool path_found = false;

    // need to also put in car frame
    double relative_goal_x = goal_x - car_x;
    double relative_goal_y = goal_y - car_y;
    double relative_dist = sqrt(pow(relative_goal_x, 2) + pow(relative_goal_y, 2));
    double carframe_goal_x = relative_goal_x * cos(-curr_theta) - relative_goal_y * sin(-curr_theta);
    double carframe_goal_y = relative_goal_x * sin(-curr_theta) + relative_goal_y * cos(-curr_theta);

    visualize_goal({goal_x, goal_y});

    // tree as std::vector
    std::vector<RRT_Node> tree;

    // TODO: fill in the RRT main loop
    // 1. init tree with root node
    RRT_Node root;

    root.is_root = true;
    // root must be the car in the grid which is put at center
    root.x = RRT::grid_width / 2;
    root.y = RRT::grid_height / 2;

    tree.push_back(root);

    // 2. loop until goal found or max iterations
    for (int i = 0; i < RRT::iteration_limit; i++ ) {
        
        // 3. sample a point
        vector<double> sampled_pt = sample();

        // bring sampled point from local to global frame
        // sampled_pt[0] += root.x;
        // sampled_pt[1] += root.y;

        // 4. find nearest node in tree to random point
        int nearest_node_idx = nearest(tree, sampled_pt);

        // 5. Move towards the sampled point from nearest node using step size
        RRT_Node nearest_node = tree[nearest_node_idx];
        RRT_Node new_node = steer(nearest_node, sampled_pt);

        // 6. if path collision free: add new node to tree with edge (update parent)
        if (!check_collision(nearest_node, new_node)) {
            new_node.parent = nearest_node_idx;
            tree.push_back(new_node);

            // 7. if new node is close to goal, end RRT loop and return the path from start to goal
            if (is_goal(new_node, carframe_goal_x, carframe_goal_y)) {
                std::vector<RRT_Node> path = find_path(tree, new_node);
                path_found = true;
                // cout << "Path found!" << endl;
                // publish path
                visualize_path(path);
                sending_wp(path, carframe_goal_x, carframe_goal_y, curr_theta, car_x, car_y);
                // publish path as Path message
                break;
            }
        }
    }

    // visualize the tree
    visualize_tree(tree);

    // path found as Path message
    if (!path_found) {
        cout << "Path not found!" << endl;
    }

}

std::vector<double> RRT::sample() {
    // This method returns a sampled point from the free space
    // You should restrict so that it only samples a small region
    // of interest around the car's current position
    // Args:
    // Returns:
    //     sampled_point (std::vector<double>): the sampled point in free space

    std::vector<double> sampled_point;
    // TODO: fill in this method
    // look up the documentation on how to use std::mt19937 devices with a distribution
    // the generator and the distribution is created for you (check the header file)

    // LATER INFORCE KINODYNAMIC CONSTRAINT TO ONLY SAMPLE FROM FEASIBLE POINTS
    // region of interest around the car's current position : 4m x 4m
    random_device rd;
    mt19937 gen(rd());
    int search_range = 8; // 2m x 2m
    int car_x = RRT::grid_width / 2;
    int car_y = RRT::grid_height / 2;
    int start_x = -search_range / RRT::grid_resolution;
    int start_y = -search_range / RRT::grid_resolution;
    int end_x = search_range / RRT::grid_resolution;
    int end_y = search_range / RRT::grid_resolution;
    start_x += car_x;
    start_y += car_y;
    end_x += car_x;
    end_y += car_y;

    // RRT::x_dist = uniform_real_distribution<>(start_x, end_x);
    // RRT::y_dist = uniform_real_distribution<>(start_y, end_y);

    // int sampleX = RRT::x_dist(gen); //grid format
    // int sampleY = RRT::y_dist(gen); //grid format


    // check if occupied or not
    // while (RRT::occupancy_grid_.data[sampleY * RRT::grid_width + sampleX] != 0) {
    //     sampleX = RRT::x_dist(gen);
    //     sampleY = RRT::y_dist(gen);
    // }


    // sampled_point.push_back(sampleX);
    // sampled_point.push_back(sampleY);

    // get the free space in a vector
    vector<pair<int, int>> free_space;
    for (int i = start_x; i < end_x; i++) {
        for (int j = start_y; j < end_y; j++) {
            int index = j * RRT::grid_width + i;
            if (index >= 0 && index < RRT::occupancy_grid_.data.size() && RRT::occupancy_grid_.data[index] == 0) {
                free_space.push_back({i, j});
            }
        }
    }
    // cout << "Free space size: " << free_space.size() << endl;
    // // sample from the free space
    uniform_int_distribution<int> dist(0, free_space.size() - 1);
    int idx = dist(gen);
    sampled_point.push_back(free_space[idx].first);
    sampled_point.push_back(free_space[idx].second);
    
    return sampled_point;
}

int RRT::nearest(std::vector<RRT_Node> &tree, std::vector<double> &sampled_point) {
    // This method returns the nearest node on the tree to the sampled point
    // Args:
    //     tree (std::vector<RRT_Node>): the current RRT tree
    //     sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //     nearest_node (int): index of nearest node on the tree

    int nearest_node = 0;
    // TODO: fill in this method
    double min_dist = 1000000;
    for (int i = 0; i < tree.size(); i++ ) {
        double dist = sqrt(pow(tree[i].x - sampled_point[0], 2) + pow(tree[i].y - sampled_point[1], 2));
        if (dist < min_dist) {
            min_dist = dist;
            nearest_node = i;
        }
    }

    return nearest_node;
}

RRT_Node RRT::steer(RRT_Node &nearest_node, std::vector<double> &sampled_point) {
    // The function steer:(x,y)->z returns a point such that z is “closer” 
    // to y than x is. The point z returned by the function steer will be 
    // such that z minimizes ||z−y|| while at the same time maintaining 
    //||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

    // basically, expand the tree towards the sample point (within a max dist)

    // Args:
    //    nearest_node (RRT_Node): nearest node on the tree to the sampled point
    //    sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //    new_node (RRT_Node): new node created from steering

    RRT_Node new_node;
    // TODO: fill in this method
    
    //expanding from nearest node to sampled point
    double max_dist = 0.5; // 0.5m -> also means 5 cells worth
    
    int x0 = nearest_node.x;
    int y0 = nearest_node.y;
    int x1 = sampled_point[0];
    int y1 = sampled_point[1];
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    while (x0 != x1 || y0 != y1) {
        int index = y0 * RRT::grid_width + x0;
        if (index >= 0 && index < RRT::occupancy_grid_.data.size()) {
            //calc dist in cartesian coordinates from the grid coordinates
            double dist = sqrt( pow( x0*RRT::grid_resolution - nearest_node.x*RRT::grid_resolution, 2) + pow( y0*RRT::grid_resolution - nearest_node.y*RRT::grid_resolution, 2));
            if (dist > max_dist) {
                break;
            }
        } 
        int e2 = 2 * err;
        if (e2 > -dy) {
            err = err - dy;
            x0 = x0 + sx;
        }
        if (e2 < dx) {
            err = err + dx;
            y0 = y0 + sy;
        }
    }

    // So now x0 and x1 is the nearest node
    new_node.x = x0;
    new_node.y = y0;

    return new_node;
}

bool RRT::check_collision(RRT_Node &nearest_node, RRT_Node &new_node) {
    // This method returns a boolean indicating if the path between the 
    // nearest node and the new node created from steering is collision free
    // Args:
    //    nearest_node (RRT_Node): nearest node on the tree to the sampled point
    //    new_node (RRT_Node): new node created from steering
    // Returns:
    //    collision (bool): true if in collision, false otherwise

    bool collision = false;
    // TODO: fill in this method
    // need to check between the two points and a buffer zone to account for the car size
    int x0 = nearest_node.x;
    int y0 = nearest_node.y;
    int x1 = new_node.x;
    int y1 = new_node.y;
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    while (x0 != x1 || y0 != y1) {
        int index = y0 * RRT::grid_width + x0;
        if (index >= 0 && index < RRT::occupancy_grid_.data.size() && RRT::occupancy_grid_.data[index] == 1) {
            collision = true;
            break;
        }
        int e2 = 2 * err;
        if (e2 > -dy) {
            err = err - dy;
            x0 = x0 + sx;
        }
        if (e2 < dx) {
            err = err + dx;
            y0 = y0 + sy;
        }
    }

    return collision;
}

bool RRT::is_goal(RRT_Node &latest_added_node, double goal_x, double goal_y) {
    // This method checks if the latest node added to the tree is close
    // enough (defined by goal_threshold) to the goal so we can terminate
    // the search and find a path
    // Args:
    //   latest_added_node (RRT_Node): latest addition to the tree
    //   goal_x (double): x coordinate of the current goal
    //   goal_y (double): y coordinate of the current goal
    // Returns:
    //   close_enough (bool): true if node close enough to the goal

    bool close_enough = false;
    // TODO: fill in this method

    // convert the latest added node to cartesian coordinates in car frame
    double x = latest_added_node.x * RRT::grid_resolution - RRT::gridsizeX / 2.0;
    double y = latest_added_node.y * RRT::grid_resolution - RRT::gridsizeY / 2.0;    

    // cout << "Original: " << latest_added_node.x << " " << latest_added_node.y << endl;
    // cout << "Converted: " << x << " " << y << endl;

    double dist = sqrt(pow(x - goal_x, 2) + pow(y - goal_y, 2));
    // cout << dist << endl;

    if (dist < 0.5) {
        close_enough = true;
    }

    return close_enough;
}

void RRT::sending_wp(std::vector<RRT_Node> &path, double rel_goalx, double rel_goaly, double curr_theta, double carx, double cary) {

    geometry_msgs::msg::PoseStamped next_wp;

    // cout << "Path size: " << path.size() << endl;

    for (int j = 0; j < path.size(); j++ ) {
        // cout << path[j].x << " " << path[j].y << endl;
        double x = path[j].x * RRT::grid_resolution - RRT::gridsizeX / 2.0;
        double y = path[j].y * RRT::grid_resolution - RRT::gridsizeY / 2.0;
        double angle = atan2(y , x );
        double dist = sqrt(pow(x, 2) + pow(y, 2));
        double direction = cos(angle);
        // cout << "x: " << x << " y: " << y << " dist: " << dist << " cos: " << direction << endl;
        if (cos(angle) > 0 && dist > 0.5 && dist < 1.5) {
            double x_map_frame = x * cos(curr_theta) - y * sin(curr_theta) + carx;
            double y_map_frame = x * sin(curr_theta) + y * cos(curr_theta) + cary;
            next_wp.pose.position.x = x;
            next_wp.pose.position.y = y ;
            next_wp.pose.position.z = 0;
            // next_wp.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, angle));
            next_wp_pub_->publish(next_wp);
            visualize_next_wp({x_map_frame, y_map_frame});
            break;
        }
    }
    // cout << endl;
}

std::vector<RRT_Node> RRT::find_path(std::vector<RRT_Node> &tree, RRT_Node &latest_added_node) {
    // This method traverses the tree from the node that has been determined
    // as goal
    // Args:
    //   latest_added_node (RRT_Node): latest addition to the tree that has been
    //      determined to be close enough to the goal
    // Returns:
    //   path (std::vector<RRT_Node>): the vector that represents the order of
    //      of the nodes traversed as the found path
    
    std::vector<RRT_Node> found_path;
    // TODO: fill in this method

    // this path is in reverse tho
    while (!latest_added_node.is_root) {
        found_path.push_back(latest_added_node);
        latest_added_node = tree[latest_added_node.parent];
    }
    found_path.push_back(tree[0]);

    std::reverse(found_path.begin(), found_path.end());

    return found_path;
}

void RRT::visualize_path(std::vector<RRT_Node> found_path) {
    visualization_msgs::msg::Marker path_marker;
    path_marker.header.frame_id = "ego_racecar/base_link";
    path_marker.header.stamp = this->now();
    path_marker.ns = "path_marker";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::msg::Marker::ADD;
    path_marker.pose.orientation.w = 1.0;
    path_marker.scale.x = 0.1;
    path_marker.color.a = 1.0;
    path_marker.color.r = 0.0;
    path_marker.color.g = 1.0;
    path_marker.color.b = 0.0;

    for (int i = 0; i < found_path.size() ; i++ ) {
        geometry_msgs::msg::Point p;
        p.x = found_path[i].x * RRT::grid_resolution - RRT::gridsizeX / 2.0;
        p.y = found_path[i].y * RRT::grid_resolution - RRT::gridsizeY / 2.0;
        p.z = 0;
        path_marker.points.push_back(p);
    }

    path_pub_->publish(path_marker);
}

void RRT::visualize_point_on_grid(std::vector<double> rand_pt) {
    visualization_msgs::msg::Marker rand_pt_marker;
    rand_pt_marker.header.frame_id = "ego_racecar/base_link";
    rand_pt_marker.header.stamp = this->now();
    rand_pt_marker.ns = "rand_pt_marker";
    rand_pt_marker.id = 0;
    rand_pt_marker.type = visualization_msgs::msg::Marker::SPHERE;
    rand_pt_marker.action = visualization_msgs::msg::Marker::ADD;
    rand_pt_marker.pose.position.x = rand_pt[0] * RRT::grid_resolution - RRT::gridsizeX / 2.0;
    rand_pt_marker.pose.position.y = rand_pt[1] * RRT::grid_resolution - RRT::gridsizeY / 2.0;
    rand_pt_marker.pose.position.z = 0;
    rand_pt_marker.pose.orientation.x = 0.0;
    rand_pt_marker.pose.orientation.y = 0.0;
    rand_pt_marker.pose.orientation.z = 0.0;
    rand_pt_marker.pose.orientation.w = 1.0;
    rand_pt_marker.scale.x = 0.5;
    rand_pt_marker.scale.y = 0.5;
    // rand_pt.marker.scale.z = 0.5;
    rand_pt_marker.color.a = 1.0;
    rand_pt_marker.color.r = 1.0;
    rand_pt_marker.color.g = 0.0;
    rand_pt_marker.color.b = 1.0;

    random_pub_->publish(rand_pt_marker);
}

void RRT::visualize_goal(std::vector<double> goal_pt) {
    visualization_msgs::msg::Marker goal_marker;
    goal_marker.header.frame_id = "map";
    goal_marker.header.stamp = this->now();
    goal_marker.ns = "goal_marker";
    goal_marker.id = 0;
    goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
    goal_marker.action = visualization_msgs::msg::Marker::ADD;
    goal_marker.pose.position.x = goal_pt[0];
    goal_marker.pose.position.y = goal_pt[1];
    goal_marker.pose.position.z = 0;
    goal_marker.pose.orientation.x = 0.0;
    goal_marker.pose.orientation.y = 0.0;
    goal_marker.pose.orientation.z = 0.0;
    goal_marker.pose.orientation.w = 1.0;
    goal_marker.scale.x = 0.3;
    goal_marker.scale.y = 0.3;
    goal_marker.color.a = 1.0;
    goal_marker.color.r = 1.0;
    goal_marker.color.g = 0.0;
    goal_marker.color.b = 0.0;

    goal_pub_->publish(goal_marker);

}

void RRT::visualize_tree(std::vector<RRT_Node> &tree) {
    visualization_msgs::msg::MarkerArray nodes_and_edges;
    visualization_msgs::msg::Marker tree_node;
    tree_node.header.frame_id = "ego_racecar/base_link";
    tree_node.header.stamp = this->now();
    tree_node.ns = "rrt_tree_node";
    tree_node.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    tree_node.action = visualization_msgs::msg::Marker::ADD;
    tree_node.pose.orientation.w = 1.0;
    tree_node.scale.x = 0.1;
    tree_node.scale.y = 0.1;
    tree_node.scale.z = 0.1;
    tree_node.color.a = 1.0;
    tree_node.color.r = 1.0;
    tree_node.color.g = 0.0;
    tree_node.color.b = 0.0;

    visualization_msgs::msg::Marker tree_edge;
    tree_edge.header.frame_id = "ego_racecar/base_link";
    tree_edge.header.stamp = this->now();
    tree_edge.ns = "rrt_tree_edge";
    tree_edge.type = visualization_msgs::msg::Marker::LINE_LIST;
    tree_edge.action = visualization_msgs::msg::Marker::ADD;
    tree_edge.pose.orientation.w = 1.0;
    tree_edge.scale.x = 0.05;
    tree_edge.color.a = 1.0;
    tree_edge.color.r = 0.0;
    tree_edge.color.g = 0.0;
    tree_edge.color.b = 1.0;


    for (int i = 0; i < tree.size(); i++) {
        geometry_msgs::msg::Point p;
        p.x = tree[i].x * RRT::grid_resolution - RRT::gridsizeX / 2.0;
        p.y = tree[i].y * RRT::grid_resolution - RRT::gridsizeY / 2.0;
        p.z = 0;
        tree_node.points.push_back(p);

        if (!tree[i].is_root) {
            geometry_msgs::msg::Point parent_pt;
            parent_pt.x = tree[tree[i].parent].x * RRT::grid_resolution - RRT::gridsizeX / 2.0;
            parent_pt.y = tree[tree[i].parent].y * RRT::grid_resolution - RRT::gridsizeY / 2.0;
            parent_pt.z = 0;
            tree_edge.points.push_back(parent_pt);
            tree_edge.points.push_back(p);
        }

    }

    nodes_and_edges.markers.push_back(tree_node);
    nodes_and_edges.markers.push_back(tree_edge);

    tree_pub_->publish(nodes_and_edges);

}

void RRT::visualize_next_wp(std::vector<double> wp_to_follow) {
    visualization_msgs::msg::Marker next_wp_marker;
    next_wp_marker.header.frame_id = "map";
    next_wp_marker.header.stamp = this->now();
    next_wp_marker.ns = "next_wp_marker";
    next_wp_marker.id = 0;
    next_wp_marker.type = visualization_msgs::msg::Marker::SPHERE;
    next_wp_marker.action = visualization_msgs::msg::Marker::ADD;
    next_wp_marker.pose.position.x = wp_to_follow[0];
    next_wp_marker.pose.position.y = wp_to_follow[1];
    next_wp_marker.pose.position.z = 0;
    next_wp_marker.pose.orientation.x = 0.0;
    next_wp_marker.pose.orientation.y = 0.0;
    next_wp_marker.pose.orientation.z = 0.0;
    next_wp_marker.pose.orientation.w = 1.0;
    next_wp_marker.scale.x = 0.3;
    next_wp_marker.scale.y = 0.3;
    next_wp_marker.color.a = 1.0;
    next_wp_marker.color.r = 0.0;
    next_wp_marker.color.g = 1.0;
    next_wp_marker.color.b = 0.0;

    next_wp_marker_pub_->publish(next_wp_marker);
}

// RRT* methods
double RRT::cost(std::vector<RRT_Node> &tree, RRT_Node &node) {
    // This method returns the cost associated with a node
    // Args:
    //    tree (std::vector<RRT_Node>): the current tree
    //    node (RRT_Node): the node the cost is calculated for
    // Returns:
    //    cost (double): the cost value associated with the node

    double cost = 0;
    // TODO: fill in this method

    return cost;
}

double RRT::line_cost(RRT_Node &n1, RRT_Node &n2) {
    // This method returns the cost of the straight line path between two nodes
    // Args:
    //    n1 (RRT_Node): the RRT_Node at one end of the path
    //    n2 (RRT_Node): the RRT_Node at the other end of the path
    // Returns:
    //    cost (double): the cost value associated with the path

    double cost = 0;
    // TODO: fill in this method

    return cost;
}

std::vector<int> RRT::near(std::vector<RRT_Node> &tree, RRT_Node &node) {
    // This method returns the set of Nodes in the neighborhood of a 
    // node.
    // Args:
    //   tree (std::vector<RRT_Node>): the current tree
    //   node (RRT_Node): the node to find the neighborhood for
    // Returns:
    //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood

    std::vector<int> neighborhood;
    // TODO:: fill in this method

    return neighborhood;
}