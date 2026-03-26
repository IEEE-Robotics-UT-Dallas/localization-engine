//
// Created by nokoru on 3/25/26.
//

#include "tof_sensor.h"
#include "mcl.h" // For calculateRayIntersection and calculateParticleLikelihood
#include <cmath>

// The physical blueprint of your 5 sensors: [0]FL, [1]FR, [2]L, [3]R, [4]B
// Hidden safely inside this source file!
const SensorMount TOF_MOUNTS[5] = {
    {0.152,  0.05,   25.0 * (M_PI / 180.0)},
    {0.152, -0.05,  -25.0 * (M_PI / 180.0)},
    {0.0,    0.152,  M_PI / 2.0},
    {0.0,   -0.152, -M_PI / 2.0},
    {-0.152, 0.0,    M_PI}
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