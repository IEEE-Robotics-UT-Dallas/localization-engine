//
// Created by nokoru on 3/24/26.
//

#ifndef PARTICLE_H
#define PARTICLE_H

#include <vector>

// Represents a single hypothetical robot in the Particle Filter
struct Particle {
    double x;       // X position in meters
    double y;       // Y position in meters
    double theta;   // Heading (yaw) in radians
    double weight;  // Probability score (0.0 to 1.0)
};

// Function declaration so main.cpp knows this exists
std::vector<Particle> initializeParticles(double start_x, double start_y, double start_yaw);

#endif // PARTICLE_H