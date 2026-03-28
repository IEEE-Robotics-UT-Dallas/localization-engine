#include <rclcpp/rclcpp.hpp>
#include "localization_node.h"
#include <map.h>

int main(int argc, char **argv) {
    // 1. Initialize the ROS 2 communications network
    rclcpp::init(argc, argv);

    ArenaMap arena;
	//arena.generateBoundaryGML("ieee_arena_boundary.gml");
    // 2. Create the node
    auto node = std::make_shared<LocalizationNode>();

    // 3. Keep the node alive ("spinning") indefinitely to listen for sensor data
    rclcpp::spin(node);

    // 4. Cleanly shut down when the program is killed
    rclcpp::shutdown();
    return 0;
}