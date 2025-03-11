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
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
/// CHECK: include needed ROS msg type headers and libraries

using namespace std;

// gloabl variable to hold waypoint data
vector<vector<float>> raw_waypoints;

class PurePursuit : public rclcpp::Node
{

public:
    PurePursuit() : Node("pure_pursuit_node")
    {
        // TODO: create ROS subscribers and publishers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_sim_topic, 10, std::bind(&PurePursuit::pose_callback, this, std::placeholders::_1));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("waypoint_visualizer", 10);

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic, 10);
        
    }

    

private:
    string odom_sim_topic = "/ego_racecar/odom";
    string drive_topic = "/drive";
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pf_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    float lookahead_distance = 0.9; // meters
    float Kp = 1.0; // proportional gain

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

        vector<float> next_waypoint;
        bool found_pt = false;
        bool found_notvalid = false;
        float test = 0;
        float testangle = 0;
        float testtheta = 0;
        vector<float> testpoint;

        // Move forward from the closest point while checking distance closer to lookahead
        for (int j = 0; j < raw_waypoints.size(); j++) {
            float dist = sqrt(pow(x - raw_waypoints[j][0], 2) + pow(y - raw_waypoints[j][1], 2));
            if (abs(dist - lookahead_distance) < 0.01) {
                // We will check heading as well
                float angle = atan2(raw_waypoints[j][1] - y, raw_waypoints[j][0] - x);
                found_notvalid = true;
                test = abs(angle - theta);
                testangle = angle;
                testtheta = theta;
                testpoint = raw_waypoints[j];
                if (cos(angle - theta) > 0){
                    next_waypoint = raw_waypoints[j];
                    found_pt = true;
                    break;
                }
                
                // Testing dot product

            }
        }
        cout << "angle: " << testangle << " theta: " << testtheta << endl;
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
    
    // void pose_callback(const geometry_msgs::msg::PoseStamped::ConstPtr pose_msg)
    void pose_callback(const nav_msgs::msg::Odometry::ConstPtr pose_msg)
    {
        
        // TODO: find the current waypoint to track using methods mentioned in lecture
        float curr_x = pose_msg->pose.pose.position.x;
        float curr_y = pose_msg->pose.pose.position.y;
        float curr_theta = get_heading(pose_msg->pose.pose.orientation);
        vector<float> next_waypoint = find_closest_waypoint(curr_x, curr_y, curr_theta);

        // visualize the waypoint
        waypoint_viz(next_waypoint);

        // TODO: transform goal point to vehicle frame of reference
        float x_rel = next_waypoint[0] - curr_x;
        float y_rel = next_waypoint[1] - curr_y;
        // this transformation is needed to get y lateral from car for curvature
        float x_car = cos(curr_theta) * x_rel + sin(curr_theta) * y_rel;
        float y_car = -sin(curr_theta) * x_rel + cos(curr_theta) * y_rel;

        // cout << "Relative x: " << x_car << " Relative y: " << y_car << endl;

        // TODO: calculate curvature/steering angle
        float radius = (lookahead_distance * lookahead_distance) / (2 * y_car);
        float curvature = 1 / radius;
        float steering_angle = atan(lookahead_distance * curvature);

        // cout << "Steering angle: " << steering_angle << endl;

        // TODO: publish drive message, don't forget to limit the steering angle.
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = 1.0;
        drive_pub_->publish(drive_msg);
    }

    
};

int main(int argc, char **argv)
{

    // reading from csv file to store our waypoints
    ifstream file("/home/vulcan/f1tenth/labs/waypoints/logs/smoothed_mimic_locker.csv");

    if (!file.is_open()) {
        cout << "Check file path" << endl;
        return 0;
    }

    string line;
    while (getline(file, line)) {
        stringstream ss(line);
        string data;
        vector<float> row;
        while (getline(ss, data, ',')) {
            row.push_back(stof(data));
        }
        raw_waypoints.push_back(row);
    }

    // cout << "Checking: " << raw_waypoints[100][0] << " " << raw_waypoints[100][1] << " " << raw_waypoints[100][2] <<  endl;
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}