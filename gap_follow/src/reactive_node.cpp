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

    std::vector<float> laser_readings;
    std::vector<float> corresp_angles_radians;
    int window_size = 5;
    std::vector<float> processed_ranges;
    std::vector<float> disparity_ranges;
    int fortyfive_idx = 180; // calc by printing once
    int N = 1040;
    int min_idx = 0;
    // float robs = 0.2; // radius of robot
    float robs = 0.1;
    std::vector<std::pair<int, int>> gap_indices;
    int max_gap_begin_idx = 0;
    int max_gap_end_idx = 0;
    int remove_thresh = 20;
    int piecewise_speed = 0;
    int disparity_thresh = 1.0;
    // float disparity_extender_length = 0.5;
    float disparity_extender_length = 0.3;
    float step_angle=0;
    std::vector<std::tuple<int, int, float>> disparity_indices;

    
    void preprocess_lidar(float* ranges)
    {   
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)
        processed_ranges.clear();

        for (int i = (fortyfive_idx ); i < ( N - fortyfive_idx ); i++)
        {
            float sum = 0.0;
            for (int j = -(window_size / 2); j < (window_size / 2) + 1; j++)
            {
                sum += ranges[i + j];
            }
            // if (sum / window_size > 3.0)
            // {
            //     sum = window_size*3.0;
            // }
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
                else{
                    disparity_indices.push_back(std::make_tuple(i+1, 1, ranges[i+1]));
                }
            }
        }
        if (disparity_indices.size() == 0){
            return;
        }
        std::cout << "Found " << disparity_indices.size() << " disparities" << std::endl;

        for (int j=0;j<disparity_indices.size();j++) {
            
            std::cout << "shorter length is: " << std::get<2>(disparity_indices[j]) << std::endl;
            // std::cout << "Angle theta is: " <<  << std::endl;
            float disp_angle = disparity_extender_length / std::get<2>(disparity_indices[j]);

            int extending_idxs = disp_angle / step_angle;

            std::cout << "Extending " << extending_idxs << " indexes" << std::endl;

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
        return;
    }

    void find_best_point(float* ranges, int* indice)
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there
        return;
    }

    

    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        
        laser_readings = scan_msg->ranges;
        step_angle = scan_msg->angle_increment;

        // display_ranges(laser_readings.data());
        
        if (corresp_angles_radians.size() == 0){
            for (int i = 0; i < scan_msg->ranges.size(); i++)
            {
                corresp_angles_radians.push_back(scan_msg->angle_min + i * scan_msg->angle_increment);
            }
        }
        // int idx = M_PI_4 / scan_msg->angle_increment; // 45 degrees
        
        disparity_extender(laser_readings.data());
        preprocess_lidar(disparity_ranges.data());
        // disparity_extender();
\
        /// TODO:
        // Find closest point to LiDAR
        for (int i=0;i<processed_ranges.size();i++){
            if (processed_ranges[i] < processed_ranges[min_idx]){
                min_idx = i;
            }
        }
        float angle = (180/M_PI_2) * corresp_angles_radians[fortyfive_idx + min_idx];
        std::cout << "The min distance is at: " << angle << " and the value is: " << processed_ranges[min_idx] << std::endl;


        // Eliminate all points inside 'bubble' (set them to zero) 
        float obs_theta = atan(robs / processed_ranges[min_idx]);
        int bubble_radius = obs_theta / scan_msg->angle_increment;
        // std::cout << "Bubble radius is: " << bubble_radius << std::endl;
        for (int i = min_idx - bubble_radius; i < min_idx + bubble_radius; i++){
            processed_ranges[i] = 0.0;
        }

        // Find max length gap 
        int begin_idx = 0;
        int end_idx = 1;
        gap_indices.clear();

        for (int i=1;i<processed_ranges.size();i++) {
            end_idx = i;
            if (processed_ranges[i] == 0.0 && (end_idx - begin_idx) > 1){

                gap_indices.push_back(std::make_pair(begin_idx, end_idx));
                begin_idx = i + 1;
            }
        }
        gap_indices.push_back(std::make_pair(begin_idx, end_idx));
        
        int gap_length = 0;
        
        for(int j=0;j<gap_indices.size();j++){
            // std::cout << "Gap " << j << " is from: " << gap_indices[j].first << " to " << gap_indices[j].second << std::endl;
            if ((gap_indices[j].second - gap_indices[j].first) > gap_length){
                max_gap_begin_idx = gap_indices[j].first;
                max_gap_end_idx = gap_indices[j].second;
                gap_length = max_gap_end_idx - max_gap_begin_idx;
            }
        }

        // Find the best point in the gap 
        
        int best_point_idx = 0;
        for (int i = max_gap_begin_idx; i < max_gap_end_idx; i++){
            if (processed_ranges[i] > processed_ranges[best_point_idx]){
                best_point_idx = i;
            }
        }
        std::cout << "Best point index is: " << best_point_idx << " and value is: " << processed_ranges[best_point_idx] << std::endl;
        // int left_sum = 0;
        // int right_sum = 0;
        // for (int i = 1; i < remove_thresh; i++){
        //     left_sum += processed_ranges[best_point_idx + i];
        //     right_sum += processed_ranges[best_point_idx - i];
        // }
        
        // std::cout << "Left sum is: " << left_sum << " , Right sum is: " << right_sum << std::endl;
        // if (left_sum > right_sum){
        //     // std::cout << "Biased left" << std::endl;
        //     best_point_idx += remove_thresh*3;
        // }
        // else{
        //     // std::cout << "Biased right" << std::endl;
        //     best_point_idx -= remove_thresh*3;
        // }
        // std::cout << "Left and right of best point: ";
        // for (int k = -5; k < 6; k++){
        //     std::cout << processed_ranges[best_point_idx + k] << " ";
        // } std::cout << std::endl;

        
        // Publish Drive message
        if (processed_ranges[best_point_idx] > 5.0){
            piecewise_speed = 2.0;
        } 
        else if (processed_ranges[best_point_idx] < 5.0 && processed_ranges[best_point_idx] > 1.5){
            piecewise_speed = 1.0;
        }
        else{
            piecewise_speed = 0.5;
        }
        best_point_idx += fortyfive_idx; // offset by 45 degrees
        float steering_angle = corresp_angles_radians[best_point_idx]; // radians
        // std::cout << "Steering angle is: " << steering_angle*(180/M_PI) << std::endl;
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = steering_angle;
        // drive_msg.drive.speed = 2.0;
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