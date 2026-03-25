#include <iostream>
#include <cmath>
#include <numeric> // Required for std::accumulate (summing)
#include "map.h"
#include "mapCreator.h"
#include "particle.h"
#include "mcl.h"

// --- Algorithm Tunables ---
// How much we 'trust' the real sensor data (in meters).
// Lower values = stricter weighting.
constexpr double TOF_SENSOR_NOISE_M = 0.02; // +/- 2cm noise expected

int main() {
    ArenaMap game_field;
    PPMWriter writer(1000, 500);

    for (const auto& w : game_field.walls) { writer.drawWall(w); }

    double robot_start_x_m = game_field.in2m(31.25, 0).x();
    double robot_start_y_m = game_field.in2m(0, 6.5).y();
    double robot_start_yaw = M_PI / 2.0;
    writer.drawRobot(robot_start_x_m, robot_start_y_m, robot_start_yaw);

    // 1. MOCK the actual sensor data from the real robot
    double actual_tof_reading = calculateRayIntersection(robot_start_x_m, robot_start_y_m, robot_start_yaw, game_field);

    // Draw the yellow laser line
    double true_hit_x = robot_start_x_m + (actual_tof_reading * std::cos(robot_start_yaw));
    double true_hit_y = robot_start_y_m + (actual_tof_reading * std::sin(robot_start_yaw));
    writer.drawLaser(robot_start_x_m, robot_start_y_m, true_hit_x, true_hit_y);

    // 2. Spawn the particles
    std::vector<Particle> particle_cloud = initializeParticles(robot_start_x_m, robot_start_y_m, robot_start_yaw);

    // --- NEW WEIGHTING LOGIC ---
    std::cout << "\nStarting Weight Update Loop..." << std::endl;
    double total_weight_sum = 0.0;

    // Use a reference `auto& p` so we actually edit the particle object in memory!
    for (auto& p : particle_cloud) {

        // A. Fire virtual laser
        double p_hit_dist = calculateRayIntersection(p.x, p.y, p.theta, game_field);

        // B. Apply Gaussian weighting formula (Updates p.weight in-place)
        p.weight = calculateParticleLikelihood(p_hit_dist, actual_tof_reading, TOF_SENSOR_NOISE_M);

        // C. Track sum for normalization
        total_weight_sum += p.weight;

        // D. Optimization: Check safety boundaries before visualizing (infinite distance check)
        if (!std::isinf(p_hit_dist)) {
            double hit_x = p.x + (p_hit_dist * std::cos(p.theta));
            double hit_y = p.y + (p_hit_dist * std::sin(p.theta));

            // Draw the Fainter/Ghostly cyan laser line
            writer.drawParticleLaser(p.x, p.y, hit_x, hit_y);
        }

        writer.drawParticle(p.x, p.y);
    }

    // --- CRITICAL: NORMALIZE WEIGHTS ---
    // If we don't do this, all particle probabilities would eventually approach zero.
    std::cout << "Normalizing weights (Total Sum was: " << total_weight_sum << ")..." << std::endl;
    if (total_weight_sum > 0.0) {
        for (auto& p : particle_cloud) {
            p.weight /= total_weight_sum;
        }
    } else {
        std::cerr << "[ERROR] Particle cloud is lost! All weights are zero." << std::endl;
    }

    // Save the "Before" picture
    writer.save("arena_map_1_before_resample.ppm");

    // --- THE RESAMPLING PHASE ---
    std::cout << "Resampling cloud based on sensor weights..." << std::endl;
    std::vector<Particle> resampled_cloud = resampleParticles(particle_cloud);

    // --- VISUALIZE THE AFTERMATH ---
    // Create a brand new blank map to draw the final result
    PPMWriter writer2(1000, 500);
    for (const auto& w : game_field.walls) { writer2.drawWall(w); }
    writer2.drawRobot(robot_start_x_m, robot_start_y_m, robot_start_yaw);

    // Draw only the surviving, clumped particles
    for (const auto& p : resampled_cloud) {
        writer2.drawParticle(p.x, p.y);
    }

    writer2.save("arena_map_2_after_resample.ppm");
    std::cout << "Localization cycle complete!" << std::endl;

    return 0;
}