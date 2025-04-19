#include <sstream>
#include <string>
#include <fstream>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

/// CHECK: include needed ROS msg type headers and libraries

using namespace std;

// gloabl variable to hold waypoint data
vector<vector<float>> raw_waypoints;
vector<float> orientation_in_wps;
vector<float> opt_vels;
vector<int> corner_idx; // corner indexes

class PurePursuit : public rclcpp::Node
{

public:
    PurePursuit() : Node("pure_pursuit_node")
    {
        // TODO: create ROS subscribers and publishers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_sim_topic, 10, std::bind(&PurePursuit::pose_callback, this, std::placeholders::_1));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("waypoint_visualizer", 10);
        corner_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("corner_visualizer", 10);

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic, 10);

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "next_wp_tofollow", 10, std::bind(&PurePursuit::wp_callback, this, std::placeholders::_1));

        identify_corners();
        
    }


private:
    string odom_sim_topic = "/ego_racecar/odom";
    string drive_topic = "/drive";
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pf_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr corner_pub_;

    float lookahead_distance = 0.95; // meters
    float Kp = 1.0; // proportional gain
    float closest_wp_idx = 0;
    float car_vel = 0;
    vector<float> next_waypoint;

    vector<pair<int, int>> corners; //identifying corners ranges
    // vector<int> corner_idx; // corner indexes

    vector<float> find_closest_waypoint(float x, float y, float theta) // theta in radians
    {

        // waypoint closest to one lookahead distance away and AHEAD of car

        // Lets find closest point to current position
        float min_dist = 1000000;
        int closest_idx = 0;
        for (int i = 0; i < raw_waypoints.size(); i++)
        {
            float dist = sqrt(pow(x - raw_waypoints[i][0], 2) + pow(y - raw_waypoints[i][1], 2));
            if (dist < min_dist)
            {
                min_dist = dist;
                closest_idx = i;
            }
        }

        // cout << "Closest: " << raw_waypoints[closest_idx][0] << " " << raw_waypoints[closest_idx][1] << endl;
        // cout << "distace: " << min_dist << endl;

        vector<float> next_waypoint;
        bool found_pt = false;
        bool found_notvalid = false;
        float test = 0;
        float testangle = 0;
        float testtheta = 0;
        vector<float> testpoint;
        

        // Move forward from the closest point while checking distance closer to lookahead
        // Edit: look at the entire waypoints list to allow for loops
        for (int j = 0; j < raw_waypoints.size(); j++) {
            float dist = sqrt(pow(x - raw_waypoints[j][0], 2) + pow(y - raw_waypoints[j][1], 2));
            if (abs(dist - lookahead_distance) < 0.1) {
                // We will check heading as well
                float angle = atan2(raw_waypoints[j][1] - y, raw_waypoints[j][0] - x);
                found_notvalid = true;
                test = abs(angle - theta);
                testangle = angle;
                testtheta = theta;
                testpoint = raw_waypoints[j];
                if (cos(angle - theta) > 0){
                    next_waypoint = raw_waypoints[j];
                    closest_wp_idx = j;
                    found_pt = true;
                    break;
                }
                
                // Testing dot product

            }
        }
        // cout << "Closest waypoint: " << closest_wp_idx << endl;
        // cout << "Closest waypoint: " << raw_waypoints[closest_wp_idx][0] << " " << raw_waypoints[closest_wp_idx][1] << endl;
        // cout << "Current pos: " << x << " " << y << endl;

        // cout << "angle: " << testangle << " theta: " << testtheta << endl;
        if (!found_pt) {
            cout << "Angle: " << test << endl;
            cout << "closest pt: " << testpoint[0] << " " << testpoint[1] << endl;
            // cout << "angle: " << testangle << " theta: " << testtheta << endl;
            cout << found_notvalid << endl;
            cout << "Last point: " << x << " " << y << endl;
            cout << "No point found at lookahead distance" << endl;
        }
        return next_waypoint;
    }

    double get_heading(const geometry_msgs::msg::Quaternion &q)
    {
        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 m(tf_q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    void corner_viz(vector<vector<float>> wps, vector<int> corner_indices) {
        visualization_msgs::msg::MarkerArray marker_array;
        for (int i = 0; i < corner_indices.size(); i++) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->now();
            marker.ns = "corners";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = wps[corner_indices[i]][0];
            marker.pose.position.y = wps[corner_indices[i]][1];
            marker.pose.position.z = 0.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker_array.markers.push_back(marker);
        }
        corner_pub_->publish(marker_array);

    }
    void waypoint_viz(vector<float> waypoint)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "waypoints";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = waypoint[0];
        marker.pose.position.y = waypoint[1];
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker_pub_->publish(marker);
    }
    
    void identify_corners() //experimental
    {
        // int first_corner = 2600;
        // int second_corner = 4100;
        // int third_corner = 6200;

        // corners.push_back(make_pair(first_corner, first_corner + 500));
        // corners.push_back(make_pair(second_corner, second_corner + 500));
        // corners.push_back(make_pair(third_corner, third_corner + 500));

        // maybe we compare difference between i+20 and i
        for (int i = 0; i < orientation_in_wps.size() - 50; i++) {
            if (abs(orientation_in_wps[i] - orientation_in_wps[i+50]) > 0.05) {
                corner_idx.push_back(i);
            }
        }

        cout << "Corner indexes: " << corner_idx.size() << endl;

    }

    void prep_lookup_table()
    {
        // purpose is to have a lookup table: {waypoints} : [velocity, lookahead distance]
        // this will be tuned and different for every map
    }
    
    // void pose_callback(const geometry_msgs::msg::PoseStamped::ConstPtr pose_msg)
    void wp_callback(const geometry_msgs::msg::PoseStamped::ConstPtr pose_msg)
    {
        // cout << "Received waypoint" << endl;
        // cout << "x: " << pose_msg->pose.position.x << " y: " << pose_msg->pose.position.y << endl;
        float x = pose_msg->pose.position.x;
        float y = pose_msg->pose.position.y;
        next_waypoint = {x, y};
    }
    
    void pose_callback(const nav_msgs::msg::Odometry::ConstPtr pose_msg)
    {
        
        // TODO: find the current waypoint to track using methods mentioned in lecture
        float curr_x = pose_msg->pose.pose.position.x;
        float curr_y = pose_msg->pose.pose.position.y;
        float curr_theta = get_heading(pose_msg->pose.pose.orientation);
        // vector<float> next_waypoint = find_closest_waypoint(curr_x, curr_y, curr_theta);
        

        // visualize the waypoint
        // waypoint_viz(next_waypoint);  
        // corner_viz(raw_waypoints, corner_idx);  
        
        // for (int i = 0; i < corner_idx.size(); i++) {
        //     if (closest_wp_idx == corner_idx[i]) {
        //         // cout << "Corner detected at: " << corner_idx[i] << endl;
        //         // car_vel = 1.0;
        //         // lookahead_distance = 0.45;
        //     }
        //     else{
        //         // lookahead_distance = 0.55;
        //     }
        // }

        // TODO: transform goal point to vehicle frame of reference
        // float x_rel = next_waypoint[0] - curr_x;
        // float y_rel = next_waypoint[1] - curr_y;
        // this transformation is needed to get y lateral from car for curvature
        // float x_car = cos(curr_theta) * x_rel + sin(curr_theta) * y_rel;
        // float y_car = -sin(curr_theta) * x_rel + cos(curr_theta) * y_rel;
        float x_car = next_waypoint[0];
        float y_car = next_waypoint[1];

        // cout << "Relative x: " << x_car << " Relative y: " << y_car << endl;

        // TODO: calculate curvature/steering angle
        float radius = (lookahead_distance * lookahead_distance) / (2 * y_car);
        float curvature = 1 / radius;
        float steering_angle = atan(lookahead_distance * curvature);

        // cout << "Steering angle: " << steering_angle << endl;
        
        // if (abs(steering_angle) > 0.1) {
        //     cout << "Turning: " <<  steering_angle << endl;
        //     car_vel = 2.2;
        // }
        // else{
        //     car_vel = 4;
        // }
        // car_vel = opt_vels[closest_wp_idx]; // get the velocity from the optimized waypoints
        car_vel = 2.0;
        // TODO: publish drive message, don't forget to limit the steering angle.
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = car_vel;
        drive_pub_->publish(drive_msg);
    }

    
};

int main(int argc, char **argv)
{

    // reading from csv file to store our waypoints
    ifstream file("/home/vulcan/f1tenth/labs/waypoints/logs/manual.csv");

    if (!file.is_open()) {
        cout << "Check file path for waypoints" << endl;
        return 0;
    }

    string line;
    vector<vector<float>> temp_raw_waypoints;
    while (getline(file, line)) {
        stringstream ss(line);
        string data;
        vector<float> row;
        while (getline(ss, data, ',')) {
            row.push_back(stof(data));
        }
        // temp_raw_waypoints.push_back({row[0], row[1]});
        raw_waypoints.push_back({row[0], row[1]});
        orientation_in_wps.push_back(row[2]);
        // opt_vels.push_back(row[2]);
        // raw_waypoints.push_back(row);
    }
    // float temp1 = temp_raw_waypoints[0][0];
    // float temp2 = temp_raw_waypoints[0][1];

    // for (int i = 0; i < raw_waypoints.size(); i++) {
    //     raw_waypoints[i][0] = temp_raw_waypoints[i][1] - temp2 + 0.2;
    //     raw_waypoints[i][1] = temp_raw_waypoints[i][0] - temp1 - 3.1;
    // }
    // cout << "Checking: " << raw_waypoints[0][0] << " " << raw_waypoints[0][1] << endl;
    // cout << "Size: " << raw_waypoints.size() << endl;
    // cout << "Checking: " << raw_waypoints[100][0] << " " << raw_waypoints[100][1] << " " << raw_waypoints[100][2] <<  endl;
    
    // for (int i = 0; i < orientation_in_wps.size() - 50; i++) {
    //     if (abs(orientation_in_wps[i] - orientation_in_wps[i+50]) > 0.14) {
    //         corner_idx.push_back(i);
    //     }
    // }

    

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}