//
// Created by nokoru on 3/24/26.
//

#ifndef MCL_H
#define MCL_H

#include "particle.h"
#include "map.h" // Assuming this holds your ArenaMap and Wall structs

// Calculates the distance to the nearest wall from a given pose and angle
double calculateRayIntersection(double start_x, double start_y, double ray_yaw, const ArenaMap& map);
// Calculates the Gaussian probability (weight) comparing two distances.
// higher returned value means a closer match.
double calculateParticleLikelihood(double simulated_dist, double actual_dist, double sensor_noise_std);
// Takes the weighted cloud and returns a new cloud of surviving particles
std::vector<Particle> resampleParticles(const std::vector<Particle>& old_particles);
// Moves the particles based on odometry commands, injecting realistic physics noise
void moveParticles(std::vector<Particle>& particles, double distance_m, double delta_yaw_rad);
#endif // MCL_H