#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
/// CHECK: include needed ROS msg type headers and libraries

class ReactiveFollowGap : public rclcpp::Node {


public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 10, std::bind(&ReactiveFollowGap::lidar_callback, this, std::placeholders::_1));

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic, 10);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    // storing laser scan data
    std::vector<float> laser_readings; //direct from sub
    std::vector<float> corresp_angles_radians; // corresponding angles
    std::vector<float> disparity_ranges; //post disparity extender
    std::vector<float> processed_ranges; //post processing
    std::vector<std::pair<int, int>> gap_indices; // start and end indices of gaps
    std::vector<std::tuple<int, int, float>> disparity_indices; // <disparity index, direction, length> // direction to extend: 0 - right, 1 - left

    // constants
    int fortyfive_idx = 180; // calc by printing once - quadrants
    int N = 1040; // size of laser scan array
    float step_angle=0; // angle increment value in laser scan
    int min_idx = 0; // index of the closest point to lidar
    int max_gap_begin_idx = 0; // index of the start of the max gap
    int max_gap_end_idx = 0; // index of the end of the max gap
    float piecewise_speed = 0; // speed of the car

    // tunable parameters
    int window_size = 1; // size of the moving average window
    float robs = 0.2; // radius of robot
    // float robs = 0.3; // map2
    int remove_thresh = 20; // not used right now
    int disparity_thresh = 1.50; // threshold to identify disparity point
    float disparity_extender_length = 0.4; // length extended from disparity
    // float disparity_extender_length = 0.3; // map2

    
    void preprocess_lidar(float* ranges)
    {   
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)
        processed_ranges.clear();

        // for (int i=0;i<N;i++){
        //     if (ranges[i] > 3.0){
        //         ranges[i] = 3.0;
        //     }
        // }

        for (int i = (fortyfive_idx ); i < ( N - fortyfive_idx ); i++)
        {
            float sum = 0.0;
            for (int j = -(window_size / 2); j < (window_size / 2) + 1; j++)
            {
                sum += ranges[i + j];
            }
            processed_ranges.push_back(sum / window_size);
        }
        return;
    }

    void disparity_extender(float* ranges)
    {
        // Extend disparities in the processed_ranges and then continue the algo by finding the closest point. Also might have to redice robs because of this.
        disparity_indices.clear();
        disparity_ranges.clear();

        for (int i = 0; i < N-1; i++)
        {
            disparity_ranges.push_back(ranges[i]);
            if (std::abs(ranges[i] - ranges[i+1]) > disparity_thresh)
            {
                if (ranges[i] < ranges[i+1])
                {
                    disparity_indices.push_back(std::make_tuple(i, 0, ranges[i]));
                }
                else
                {
                    disparity_indices.push_back(std::make_tuple(i+1, 1, ranges[i+1]));
                }
            }
        }
        // if no gaps found, return
        if (disparity_indices.size() == 0){
            return;
        }

        // extend disparities by choosing directions
        for (int j=0;j<disparity_indices.size();j++) {
            
            // using r*theta = d
            // where r = dist to diisparity pt, theta = arc angle, d = length of extension
            float disp_angle = disparity_extender_length / std::get<2>(disparity_indices[j]);
            int extending_idxs = disp_angle / step_angle;

            if (std::get<1>(disparity_indices[j]) == 0) // extend towards right
            {
                for (int k = 0; k < extending_idxs; k++)
                {
                    disparity_ranges[std::get<0>(disparity_indices[j]) + k] = std::get<2>(disparity_indices[j]);
                }
            }
            else // extend towards left
            {
                for (int k = 0; k < extending_idxs; k++)
                {
                    disparity_ranges[std::get<0>(disparity_indices[j]) - k] = std::get<2>(disparity_indices[j]);
                }
            }
        }
        return;
    }

    void find_max_gap(float* ranges, int* indice)
    {   
        // Return the start index & end index of the max gap in free_space_ranges
        // Implemented inside callback itself
        return;
    }

    void find_best_point(float* ranges, int* indice)
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there
        // Implemented inside callback itself
        return;
    }

    void piecewise_speed_calc(float* ranges, int best_idx, float steer_angle)
    {
        // Return speed of car based on the farthest distance we're aiming for
        // can be deepet point in the max length gap or the deepest gap's center
        // steer angle not used right now, might be useful so kept as input
        // if (ranges[best_idx] >= 5.0){
        //     piecewise_speed = 3.5;
        // } 
        // else if (ranges[best_idx] < 5.0 && ranges[best_idx] > 1.5){
        //     piecewise_speed = 2.0;
        // }
        // else{
        //     piecewise_speed = 1.0;
        // }
        if (ranges[best_idx] >= 7.0) {
            piecewise_speed = 8.0;
        }
        else if (ranges[best_idx] < 7.0 && ranges[best_idx] > 1.5){
            piecewise_speed = (5.0/7.0) * ranges[best_idx];
        }
        else{
            piecewise_speed = 1.0;
        }
        std::cout << "Piecewise speed is: " << piecewise_speed << std::endl;

    }

    
    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        ///////////////////////////////////////////////////////////////////////////////////////// [STEP] Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        ///////////////////////////////////////////////////////////////////////////////////////

        laser_readings = scan_msg->ranges;
        step_angle = scan_msg->angle_increment;

        // Getting all the corresponding angles for easy usage later - filled only once
        if (corresp_angles_radians.size() == 0){
            for (int i = 0; i < scan_msg->ranges.size(); i++)
            {
                corresp_angles_radians.push_back(scan_msg->angle_min + i * scan_msg->angle_increment);
            }
        }

        // First we extend the disparities and then process the LiDAR scan        
        disparity_extender(laser_readings.data());
        preprocess_lidar(disparity_ranges.data());
\
        // [STEP] Find closest point to LiDAR
        for (int i=0;i<processed_ranges.size();i++){
            if (processed_ranges[i] < processed_ranges[min_idx]){
                min_idx = i;
            }
        }
        // float angle = (180/M_PI) * corresp_angles_radians[fortyfive_idx + min_idx];
        // std::cout << "The min distance is at: " << angle << " and the value is: " << processed_ranges[min_idx] << std::endl;

        ///////////////////////////////////////////////////////////////////////////////////////
        // [STEP] Eliminate all points inside 'bubble' (set them to zero) 
        ///////////////////////////////////////////////////////////////////////////////////////
        
        // calculated using geometry
        // where r = shortest distance, theta = arc angle, d = radius of bubble
        float obs_theta = atan(robs / processed_ranges[min_idx]);
        int bubble_radius = obs_theta / scan_msg->angle_increment;
        for (int i = min_idx - bubble_radius; i < min_idx + bubble_radius; i++){
            processed_ranges[i] = 0.0;
        }

        ///////////////////////////////////////////////////////////////////////////////////////
        // [STEP] Find max length gap 
        ///////////////////////////////////////////////////////////////////////////////////////

        // First we get all the gaps from processed_ranges
        int begin_idx = 0;
        int end_idx = 1;
        gap_indices.clear();

        for (int i=1;i<processed_ranges.size();i++) {
            end_idx = i;
            if (processed_ranges[i] < 2.0 && (end_idx - begin_idx) > 1){

                gap_indices.push_back(std::make_pair(begin_idx, end_idx));
                begin_idx = i + 1;
            }
        }
        gap_indices.push_back(std::make_pair(begin_idx, end_idx));
        
        // [OPTION] this is the biggest bap
        int gap_length = 0;
        for(int j=0;j<gap_indices.size();j++){
            // std::cout << "Gap " << j << " is from: " << gap_indices[j].first << " to " << gap_indices[j].second << std::endl;
            if ((gap_indices[j].second - gap_indices[j].first) > gap_length){
                max_gap_begin_idx = gap_indices[j].first;
                max_gap_end_idx = gap_indices[j].second;
                gap_length = max_gap_end_idx - max_gap_begin_idx;
            }
        }

        // [OPTION] this is the deepest gap
        // int deepest_gap_start_idx = 0;
        // int deepest_gap_end_idx = 0;
        // float max_depth = 0.0;
        // for (int k = 0;k<gap_indices.size();k++){
        //     float depth = *std::max_element(processed_ranges.begin() + gap_indices[k].first, processed_ranges.begin() + gap_indices[k].second);

        //     if (gap_indices[k].second - gap_indices[k].first < 10) {
        //         continue;
        //     }
            
        //     std::cout << "Depth of gap " << k << " is: " << depth << std::endl;

        //     if (depth > max_depth){
        //         deepest_gap_start_idx = gap_indices[k].first;
        //         deepest_gap_end_idx = gap_indices[k].second;
        //         max_depth = depth;
        //     }
        // }
        // std::cout << "The gap is from " << deepest_gap_start_idx << " to " << deepest_gap_end_idx << std::endl;
        
        ///////////////////////////////////////////////////////////////////////////////////////
        // [STEP] Find the best point in the gap 
        ///////////////////////////////////////////////////////////////////////////////////////

        int best_point_idx = 0;
        for (int i = max_gap_begin_idx; i < max_gap_end_idx; i++){
            if (processed_ranges[i] > processed_ranges[best_point_idx]){
                best_point_idx = i;
            }
        }
        best_point_idx = max_gap_begin_idx + ((max_gap_end_idx - max_gap_begin_idx)/2);
        // best_point_idx = deepest_gap_start_idx + (deepest_gap_end_idx - deepest_gap_start_idx)/2;
        
        std::cout << "Best point index is: " << best_point_idx << " and value is: " << processed_ranges[best_point_idx] << std::endl;


        ///////////////////////////////////////////////////////////////////////////////////////
        // [STEP] Publish Drive message
        ///////////////////////////////////////////////////////////////////////////////////////
        
        // best_point_idx += fortyfive_idx; // offset by 45 degrees to get corresp angle
        float steering_angle = corresp_angles_radians[best_point_idx + fortyfive_idx]; // radians
        std::cout << "Steering angle is: " << steering_angle*(180/M_PI) << std::endl;
        piecewise_speed_calc(processed_ranges.data(), best_point_idx, steering_angle);
        
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = piecewise_speed;
        drive_pub_->publish(drive_msg);
    }



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}