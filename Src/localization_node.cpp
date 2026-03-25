//
// Created by nokoru on 3/24/26.
//

#include "localization_node.h"
#include "mcl.h"
#include <cmath> // For M_PI

// Constructor implementation
LocalizationNode::LocalizationNode() : Node("particle_filter_node") {
    RCLCPP_INFO(this->get_logger(), "Booting up Monte Carlo Localization Engine...");

    // 1. Load the Field Map
    game_field_ = ArenaMap();

    // 2. Determine initial robot pose (Eventually this will come from a ROS parameter)
    double start_x = game_field_.in2m(31.25, 0).x();
    double start_y = game_field_.in2m(0, 6.5).y();
    double start_yaw = M_PI / 2.0;

    // 3. Spawn the initial cloud
    particle_cloud_ = initializeParticles(start_x, start_y, start_yaw);

    RCLCPP_INFO(this->get_logger(), "Spawned %zu particles successfully.", particle_cloud_.size());
}