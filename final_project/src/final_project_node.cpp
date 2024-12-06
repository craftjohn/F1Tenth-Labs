#include <sstream>
#include <string>
#include <cmath>
#include <vector>
#include <fstream>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <Eigen/Eigen>

using namespace std;

class FinalProject : public rclcpp::Node
{
private:
    struct Waypoints
    {
        vector<double> X, Y;
        int index = 0;
    } waypoints;

    int num_waypoints = 0;
    double x_pos = 0.0;
    double y_pos = 0.0;
    double min_lookahead = 0.5;
    double max_lookahead = 1.0;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr waypoint_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr waypoint_path_pub;

    //path to data file
    string data_path = "/home/john/Downloads/data.csv";

public:
    FinalProject() : Node("final_project_node")
    {
        //publishers and subscribers
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 25, std::bind(&FinalProject::odom_callback, this, std::placeholders::_1));
        drive_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        waypoint_pub = this->create_publisher<visualization_msgs::msg::Marker>("/lookahead_waypoint", 10);
        waypoint_path_pub = this->create_publisher<visualization_msgs::msg::Marker>("/waypoints", 10);

        load_waypoints();
    }

    //laoding waypoints in from data.csv
    void load_waypoints()
    {
        std::ifstream csvFile_waypoints(data_path);
        std::string line, word;
        
        while (std::getline(csvFile_waypoints, line))
        {
            std::stringstream s(line);
            int col = 0;
            while (std::getline(s, word, ','))
            {
                if (!word.empty())
                {
                    if (col == 0)
                        waypoints.X.push_back(std::stod(word));
                    else if (col == 1)
                        waypoints.Y.push_back(std::stod(word));
                }
                col++;
            }
        }
        num_waypoints = waypoints.X.size();
        RCLCPP_INFO(this->get_logger(), "Loaded %d waypoints...", num_waypoints);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        //current x and y position
        x_pos = msg->pose.pose.position.x;
        y_pos = msg->pose.pose.position.y;

        //calculating the direction
        double car_dir = atan2(2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y), 
            1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +msg->pose.pose.orientation.z * msg->pose.pose.orientation.z));

        //get the waypoint
        get_waypoint(msg);
        auto best_waypoint = Eigen::Vector3d(waypoints.X[waypoints.index], waypoints.Y[waypoints.index], 0.0);

        //finding position
        double dx = best_waypoint.x() - x_pos;
        double dy = best_waypoint.y() - y_pos;
        double x = dx * cos(-car_dir) - dy * sin(-car_dir);
        double y = dx * sin(-car_dir) + dy * cos(-car_dir);
        double distance = sqrt(x * x + y * y);

        double steering_angle = 2.0 * y / (distance * distance);

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = 2.0;
        drive_pub->publish(drive_msg);

        //display waypoints
        show_best_waypoint(best_waypoint);
        waypoint_path();
    }


    void get_waypoint(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double best_distance = -1.0;

        double car_dir = atan2(2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y),
            1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z));

        for (int i = 0; i < num_waypoints; ++i)
        {
            double dx = waypoints.X[i] - x_pos;
            double dy = waypoints.Y[i] - y_pos;
            double dist = sqrt(dx * dx + dy * dy);
            double waypoint_angle = atan2(dy, dx);
            double angle_diff = atan2(sin(waypoint_angle - car_dir), cos(waypoint_angle - car_dir));

            if (dist >= min_lookahead && dist <= max_lookahead && fabs(angle_diff) < M_PI_2 && dist > best_distance)
            {
                best_distance = dist;
                waypoints.index = i;
            }
        }
    }

    //showing waypoint path in RVIZ
    void waypoint_path()
    {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.05;
        marker.color.a = 1.0;
        marker.color.b = 1.0;

        for (size_t i = 0; i < waypoints.X.size(); ++i)
        {
            geometry_msgs::msg::Point point;
            point.x = waypoints.X[i];
            point.y = waypoints.Y[i];
            marker.points.push_back(point);
        }

        waypoint_path_pub->publish(marker);
    }

    //displaying best waypoint marker
    void show_best_waypoint(Eigen::Vector3d &point)
    {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.color.a = 1.0;
        marker.color.g = 1.0;

        marker.pose.position.x = point.x();
        marker.pose.position.y = point.y();
        marker.pose.position.z = 0;

        waypoint_pub->publish(marker);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}
