#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <cmath>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class finalProject : public rclcpp::Node {
public:
    finalProject() : Node("final_project_node"), current_state(State::wallFollow) {
        scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&finalProject::lidar_callback, this, std::placeholders::_1));

        drive_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
    }

//states
private:
    enum class State {
        wallFollow,
        gapFollow,
        emergencyBrake
    };

    State current_state;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub;

    //parameters
    double kp = 1.0;
    double ki = 0.0;
    double kd = 0.001;
    double prev_error = 0.0;
    double integral = 0.0;
    double servo_offset = 0.0;
    double desired_distance = 1.0;
    double gap_integral = 0.0;
    double L = 1.25;
    double ttc = 0.4; //ttc value to initiate braking

    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {

        auto ranges = std::vector<float>(scan_msg->ranges.begin(), scan_msg->ranges.end());
        preprocess_lidar(ranges, scan_msg->range_max);

        current_state = stateSelect(ranges, scan_msg);

        //printing info for the current state
        switch (current_state) {
            case State::wallFollow:
                RCLCPP_INFO(this->get_logger(), "Current state: wall follow");
                wall_follow(scan_msg);
                break;

            case State::gapFollow:
                RCLCPP_INFO(this->get_logger(), "Current state: gap follow");
                gap_follow(ranges, scan_msg);
                break;

            case State::emergencyBrake:
                RCLCPP_INFO(this->get_logger(), "Braking!.");
                emergency_brake();
                break;
        }
    }

    //state machine -- selecting state based on current position
    State stateSelect(const std::vector<float>& ranges, const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
                
        double min_ttc = std::numeric_limits<double>::max();
        
        for (size_t i = 0; i < ranges.size(); ++i) {
            if (ranges[i] > 0.0) {
                double angle = std::cos(scan_msg->angle_min + i * scan_msg->angle_increment);
                if (angle > 0) {
                    double ttc = ranges[i] / angle;
                    min_ttc = std::min(min_ttc, ttc);
                }
            }
        }

        if (min_ttc < ttc) {
            return State::emergencyBrake;
        }

        if(min_ttc < 1.0){
            return State::gapFollow;
        }


        else {
            return State::wallFollow;
        }
    }

    //function from gap follow
    void preprocess_lidar(std::vector<float>& ranges, float range_max){
        for (size_t i = 0; i < ranges.size(); ++i) {
            if (ranges[i] > range_max || std::isinf(ranges[i]) || std::isnan(ranges[i])) {
                ranges[i] = 0.0;
            }
        }
    }

    //getting range of obstacles
    double get_range(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, double angle) {    

        int index = static_cast<int>((angle - scan_msg->angle_min) / scan_msg->angle_increment);

        if (index >= 0 && index < static_cast<int>(scan_msg->ranges.size())) {
            double range = scan_msg->ranges[index];

            if (std::isnan(range) || std::isinf(range)) {
                return scan_msg->range_max;
            }
            return range;
        }
        return scan_msg->range_max;
    }

    //wall follow function -- from wall follow assignment
    void wall_follow(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
        
        //ranges
        double a = get_range(scan_msg, M_PI / 4);
        double b = get_range(scan_msg, M_PI / 2);
        
        double theta = std::abs((M_PI / 4) - (M_PI / 2));
        double alpha = std::atan((a * std::cos(theta) - b) / (a * std::sin(theta)));

        //calculating distances
        double distance = b * std::cos(alpha);
        double f_dist = distance + L * std::sin(alpha);
        double error = desired_distance - f_dist;

        //PID control
        double p = kp * error;
        gap_integral += error;
        double i = ki * gap_integral;
        double d = kd * (error - prev_error);
        prev_error = error;

        //calculating and setting the steering angle (lowered a bit)
        double steering_angle = -(p + i + d + servo_offset);
        steering_angle = std::max(-0.8, std::min(0.8, steering_angle));

        //default speed
        double speed = 2.0;

        //setting speed based on the current steering angle
        if (std::abs(steering_angle) > 0.3) {
            speed = 1.5; 
        }
        if (std::abs(steering_angle) > 0.5) {
            speed = 1.0; 
        }
        publish_msg(steering_angle, speed);
    }


    //gap follow -- from gap follow assignment
    void gap_follow(std::vector<float>& ranges, const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
        preprocess_lidar(ranges, scan_msg->range_max);

        //setting bubble
        int closest_index = 0;
        float closest_distance = scan_msg->range_max * 5;
        for (size_t i = 0; i < ranges.size(); ++i) {
            if (ranges[i] < closest_distance && ranges[i] > 0) {
                closest_distance = ranges[i];
                closest_index = i;
            }
        }

        int bubble = 350;
        int bubble_start = std::max(0, closest_index - bubble);
        int bubble_end = std::min(static_cast<int>(ranges.size() - 1), closest_index + bubble);
    
        for (int i = bubble_start; i <= bubble_end; ++i) {
            ranges[i] = 0.0;
        }

        int start, end;
        find_max_gap(ranges, start, end);

        int best_index = find_best_point(ranges, start, end);

        //finding best angle
        double best_angle = scan_msg->angle_min + best_index * scan_msg->angle_increment;
        double error = best_angle;  

        //implementing PID algorithm from wall follow
        double p = kp * error;
        integral += error;
        double i = ki * integral;
        double d = kd * (error - prev_error);
        prev_error = error;

        //calculating steering angle
        double steering_angle = (p + i + d);
        steering_angle = std::max(-1.0, std::min(1.0, steering_angle));

        //default speed 
        double speed = 2.0;

        //setting speed based on the closest obstacle
        if (closest_distance < 0.5) {
            speed = 1.25;  
        } 
        else if (closest_distance < 1.0) {
            speed = 1.5;
        }

        publish_msg(steering_angle, speed);
    }


    //finding best gap -- from gap follow assignment
    void find_max_gap(const std::vector<float>& ranges, int& start, int& end) {

        int max_start = 0; 
        int max_end = 0; 
        int current_start = -1;
        size_t max_gap = 0;
        size_t current_gap = 0;

        for (size_t i = 0; i < ranges.size(); i++) {
            if (ranges[i] > 0.0) {
                if (current_start == -1) current_start = i;
                current_gap++;
            } 
            else {
                if (current_gap > max_gap) {
                    max_gap = current_gap;
                    max_start = current_start;
                    max_end = i - 1;
                }

                current_start = -1;
                current_gap = 0;
            }
        }

        if (current_gap > max_gap) {
            max_gap = current_gap;
            max_start = current_start;
            max_end = ranges.size() - 1;
        }

        start = max_start;
        end = max_end;
    }

    //selecting the best point
    int find_best_point(const std::vector<float>& ranges, int start, int end) {
        int best_index = start;
        float max_distance = 0.0;

        for (int i = start; i <= end; i++) {
            if (ranges[i] > max_distance) {
                max_distance = ranges[i];
                best_index = i;
            }
        }
        return best_index;
    }

    //emergency braking -- set speed to zero
    void emergency_brake() {
        publish_msg(0.0, 0.0);
    }

    //publising drive messages
    void publish_msg(double steering_angle, double speed) {
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = speed;
        drive_pub->publish(drive_msg);
    }

    std::string state(State state) {
        switch (state) {
            case State::wallFollow:
                return "Wall Follow";

            case State::gapFollow:
                return "Gap Follow";

            case State::emergencyBrake:
                return "Emergency Brake";
            
            //default state needed to build
            default:
                return "no current state";
 
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<finalProject>());
    rclcpp::shutdown();
    return 0;
}
