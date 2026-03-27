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
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
	// Tools for Nav2 Communication
	std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	std::array<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr, 5> tof_publishers_;

	// The calculation function
	void publishMapToOdom(const geometry_msgs::msg::Pose& best_pose,
                      const nav_msgs::msg::Odometry& current_odom);

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
    double dist_since_last_update_ = 0.0;
    double yaw_since_last_update_ = 0.0;

};

#endif // LOCALIZATION_NODE_H