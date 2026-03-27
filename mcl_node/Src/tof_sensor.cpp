//
// Created by nokoru on 3/25/26.
//

#include "tof_sensor.h"
#include "mcl.h" // For calculateRayIntersection and calculateParticleLikelihood
#include <cmath>

    // The physical blueprint of the 5 sensors (Center of Wheels = Origin)
    // Order: [0]Right, [1]Back-Right, [2]Back-Center, [3]Back-Left, [4]Left
    const SensorMount TOF_MOUNTS[5] = {
        // TOF 1: Right Side (Pointing -90 degrees)
        {-0.0003, -0.0757, -M_PI / 2.0},

        // TOF 2: Back-Right (Fanned outwards towards the right)
        {-0.0901, -0.0272, -(M_PI - (11.5 * M_PI / 180.0))},

        // TOF 3: Back-Center (Pointing straight back, 180 degrees)
        {-0.0934,  0.0,     M_PI},

        // TOF 4: Back-Left (Fanned outwards towards the left)
        {-0.0901,  0.0272,  (M_PI - (11.5 * M_PI / 180.0))},

        // TOF 5: Left Side (Pointing +90 degrees)
        {-0.0003,  0.0757,  M_PI / 2.0}
    };

void applyToFMeasurement(std::vector<Particle>& particles, const std::vector<float>& tof_readings, const ArenaMap& map) {
    double sensor_noise_std = 0.1;

    for (auto& p : particles) {
        p.weight = 1.0;

        for (int i = 0; i < 5; i++) {
            double actual_dist = tof_readings[i];

            // Ignore sensors that read 0.0 or out-of-bounds errors
            if (actual_dist <= 0.01 || actual_dist > 4.0) {
                continue;
            }

            SensorMount mount = TOF_MOUNTS[i];

            // 1. Calculate the true starting point of the beam (The Offset Math)
            double true_start_x = p.x + (mount.dx * std::cos(p.theta)) - (mount.dy * std::sin(p.theta));
            double true_start_y = p.y + (mount.dx * std::sin(p.theta)) + (mount.dy * std::cos(p.theta));

            // 2. Calculate the absolute direction the beam is pointing
            double ray_yaw = p.theta + mount.yaw;

            // 3. Raycast and compare
            double simulated_dist = calculateRayIntersection(true_start_x, true_start_y, ray_yaw, map);
            double likelihood = calculateParticleLikelihood(simulated_dist, actual_dist, sensor_noise_std);

            p.weight *= likelihood;
        }
    }
}