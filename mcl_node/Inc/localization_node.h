#ifndef LOCALIZATION_NODE_H
#define LOCALIZATION_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>
#include <sensor_msgs/msg/range.hpp>
#include "map.h"
#include "particle.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <visualization_msgs/msg/marker.hpp> // <-- Added Marker include

class LocalizationNode : public rclcpp::Node {
public:
    LocalizationNode();

private:
    void publishParticles();
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void tofArrayCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    // NEW: The map publishing function declaration
    void publishMap();

    ArenaMap game_field_;
    std::vector<Particle> particle_cloud_;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr tof_array_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr tof_front_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

    // NEW: The map publisher and timer declarations
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr map_timer_;

    bool first_odom_ = true;
    double last_odom_x_ = 0.0;
    double last_odom_y_ = 0.0;
    double last_odom_yaw_ = 0.0;
};

#endif // LOCALIZATION_NODE_H