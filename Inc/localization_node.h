//
// Created by nokoru on 3/24/26.
//

#ifndef LOCALIZATION_NODE_H
#define LOCALIZATION_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <vector>

// Include your custom math structures
#include "map.h"
#include "particle.h"

class LocalizationNode : public rclcpp::Node {
public:
    // Constructor declaration
    LocalizationNode();

private:
    // Store the engine's state in memory
    ArenaMap game_field_;
    std::vector<Particle> particle_cloud_;

    // --- FUTURE HOME OF ROS 2 TIMERS, PUBLISHERS & SUBSCRIBERS ---
    // rclcpp::Subscription<...>::SharedPtr odom_sub_;
    // rclcpp::Publisher<...>::SharedPtr pose_pub_;
};

#endif // LOCALIZATION_NODE_H