#include "localization_node.h"
#include "mcl.h"
#include "tof_sensor.h"
#include <cmath>


// ==============================================================================
// 1. THE CONSTRUCTOR (Booting up the node)
// ==============================================================================
LocalizationNode::LocalizationNode() : Node("particle_filter_node") {
	initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    	"/initialpose", 10, std::bind(&LocalizationNode::initialPoseCallback, this, std::placeholders::_1)
	);
    RCLCPP_INFO(this->get_logger(), "Booting up Monte Carlo Localization Engine...");

    game_field_ = ArenaMap();

 //   double start_x = game_field_.in2m(31.25, 0).x();
 //   double start_y = game_field_.in2m(0, 6.5).y();
 //   double start_yaw = M_PI / 2.0;

//    particle_cloud_ = initializeParticles(start_x, start_y, start_yaw);
    RCLCPP_INFO(this->get_logger(), "Spawned %zu particles successfully.", particle_cloud_.size());

	// Initialize Publishers
 //   particle_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/particle_cloud", 10);
    map_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/arena_map", 10);
	// 1. Initialize the Broadcaster
	tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

/*	// 2. Setup the 5 ToF outlets
	for (int i = 0; i < 5; ++i) {
	    std::string topic_name = "tof_" + std::to_string(i);
	    tof_publishers_[i] = this->create_publisher<sensor_msgs::msg::Range>(topic_name, 10);
	}
    // Publish the map every 1 second
    map_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&LocalizationNode::publishMap, this)
    );

*/
    // Initialize Subscribers
	odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    	"/odometry/filtered", 10, std::bind(&LocalizationNode::odomCallback, this, std::placeholders::_1));


    // FIXED: Now listening for the Float32MultiArray on /tof_array
//    tof_array_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
 //       "/tof_array", 10, std::bind(&LocalizationNode::tofArrayCallback, this, std::placeholders::_1)
//    );

    // Initialize the Timer (The heartbeat that publishes the cloud at 10Hz)
//    timer_ = this->create_wall_timer(
//        std::chrono::milliseconds(100),
//        std::bind(&LocalizationNode::publishParticles, this)
//    );


}
/*
// ==============================================================================
// 2. THE TOF ARRAY CALLBACK (The Update Step)
// ==============================================================================
// FIXED: std_msgs instead of std::msgs
void LocalizationNode::tofArrayCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    // Keep your movement check
    if (dist_since_last_update_ < 0.05 && yaw_since_last_update_ < 0.1) {
        return;
    }

    if (msg->data.size() != 5) {
        RCLCPP_WARN(this->get_logger(), "Received malformed ToF array! Expected 5, got %zu", msg->data.size());
        return;
    }

    // --- NEW: PART A (The Nav2 Splitter) ---
    for (size_t i = 0; i < 5; ++i) {
        auto range_msg = sensor_msgs::msg::Range();
        range_msg.header.stamp = this->get_clock()->now();
        range_msg.header.frame_id = "tof_sensor_" + std::to_string(i); // Matches your Launch file
        range_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
        range_msg.field_of_view = 0.436;
        range_msg.min_range = 0.02;
        range_msg.max_range = 2.0;
        range_msg.range = msg->data[i];

        tof_publishers_[i]->publish(range_msg);
    }

    // --- YOUR EXISTING LOGIC ---
    std::vector<float> tof_readings(msg->data.begin(), msg->data.end());
    applyToFMeasurement(particle_cloud_, tof_readings, game_field_);
    particle_cloud_ = resampleParticles(particle_cloud_);

    // --- NEW: PART B (The Nav2 Sync) ---
    // Calculate the "Best" pose (average of all particles) to send to Nav2
    double avg_x = 0, avg_y = 0, sum_sin = 0, sum_cos = 0;
    for (const auto& p : particle_cloud_) {
        avg_x += p.x;
        avg_y += p.y;
        sum_sin += std::sin(p.theta);
        sum_cos += std::cos(p.theta);
    }

    geometry_msgs::msg::Pose best_pose;
    best_pose.position.x = avg_x / particle_cloud_.size();
    best_pose.position.y = avg_y / particle_cloud_.size();
    double avg_theta = std::atan2(sum_sin, sum_cos);
    best_pose.orientation.z = std::sin(avg_theta / 2.0);
    best_pose.orientation.w = std::cos(avg_theta / 2.0);

    // Now tell Nav2 where we are relative to the drifting Odometry
    // Assuming you store the latest odom in a member variable:
if (!first_odom_) {
        publishMapToOdom(best_pose, latest_odom_msg_);
    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Waiting for first Odometry reading before publishing TF...");
    }

    dist_since_last_update_ = 0.0;
    yaw_since_last_update_ = 0.0;
}
*/
// ==============================================================================
// 3. THE ODOMETRY CALLBACK (The Prediction Step)
// ==============================================================================
void LocalizationNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // 1. Save this for Nav2's map->odom transform!
    latest_odom_msg_ = *msg;

    double current_x = msg->pose.pose.position.x;
    double current_y = msg->pose.pose.position.y;

    // 2. This quaternion already contains your STM32 IMU heading!
    double q_z = msg->pose.pose.orientation.z;
    double q_w = msg->pose.pose.orientation.w;
    double current_yaw = 2.0 * std::atan2(q_z, q_w);

    if (first_odom_) {
        last_odom_x_ = current_x;
        last_odom_y_ = current_y;
        last_odom_yaw_ = current_yaw;
        first_odom_ = false;
        return;
    }

    double delta_x = current_x - last_odom_x_;
    double delta_y = current_y - last_odom_y_;
    double delta_yaw = current_yaw - last_odom_yaw_;

    while (delta_yaw > M_PI) delta_yaw -= 2.0 * M_PI;
    while (delta_yaw < -M_PI) delta_yaw += 2.0 * M_PI;

    last_odom_x_ = current_x;
    last_odom_y_ = current_y;
    last_odom_yaw_ = current_yaw;

    if (std::abs(delta_x) > 0.001 || std::abs(delta_y) > 0.001 || std::abs(delta_yaw) > 0.001) {
        double distance_m = std::hypot(delta_x, delta_y);

        double direction_check = (delta_x * std::cos(last_odom_yaw_)) + (delta_y * std::sin(last_odom_yaw_));
        if (direction_check < 0.0) {
            distance_m = -distance_m;
        }

        moveParticles(particle_cloud_, distance_m, delta_yaw);

        dist_since_last_update_ += std::abs(distance_m);
        yaw_since_last_update_ += std::abs(delta_yaw);
    }
}

// ==============================================================================
// 4. THE PUBLISHER (Sending data to RViz)
// ==============================================================================
void LocalizationNode::publishParticles() {
    auto msg = geometry_msgs::msg::PoseArray();

    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "map";

    for (const auto& p : particle_cloud_) {
        geometry_msgs::msg::Pose pose;

        pose.position.x = p.x;
        pose.position.y = p.y;
        pose.position.z = 0.0;

        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = std::sin(p.theta / 2.0);
        pose.orientation.w = std::cos(p.theta / 2.0);

        msg.poses.push_back(pose);
    }

    particle_pub_->publish(msg);
}

void LocalizationNode::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    // Extract rotation (yaw) from the quaternion
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    double yaw = 2.0 * atan2(qz, qw);

    RCLCPP_INFO(this->get_logger(), "Teleporting particles to: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw);

    // Re-initialize the cloud at this specific point
    particle_cloud_ = initializeParticles(x, y, yaw);
}

void LocalizationNode::publishMap() {
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "arena_walls";
    marker.id = 0;

	marker.pose.orientation.w = 1.0;

    // We want to draw a series of unconnected lines
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Line thickness
    marker.scale.x = 0.05;

    // Color: Bright Blue (R: 0, G: 0.5, B: 1.0, Alpha: 1.0)
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.5;
    marker.color.b = 1.0;

    // Loop through your math engine's map and add the coordinates
    for (const auto& wall : game_field_.walls) {
        geometry_msgs::msg::Point p_start;
        p_start.x = wall.start_point.x();
        p_start.y = wall.start_point.y();
        p_start.z = 0.0;

        geometry_msgs::msg::Point p_end;
        p_end.x = wall.end_point.x();
        p_end.y = wall.end_point.y();
        p_end.z = 0.0;

        // LINE_LIST requires points in pairs (Start, End, Start, End...)
        marker.points.push_back(p_start);
        marker.points.push_back(p_end);
    }

    map_pub_->publish(marker);
}

void LocalizationNode::publishMapToOdom(const geometry_msgs::msg::Pose& best_pose,
                                        const nav_msgs::msg::Odometry& current_odom) {
    // 1. Map -> Robot (The Truth from your Particles)
    tf2::Transform map_to_base;
    tf2::fromMsg(best_pose, map_to_base);

    // 2. Odom -> Robot (The Drift from your Encoders)
    tf2::Transform odom_to_base;
    tf2::fromMsg(current_odom.pose.pose, odom_to_base);

    // 3. Map -> Odom calculation (The Correction)
    tf2::Transform map_to_odom = map_to_base * odom_to_base.inverse();

    // 4. Send the transform
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "odom";
    t.transform = tf2::toMsg(map_to_odom);

    tf_broadcaster_->sendTransform(t);
}

