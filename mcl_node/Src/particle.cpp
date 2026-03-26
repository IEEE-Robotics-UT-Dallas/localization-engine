#include "particle.h"
#include <random>
#include <cmath>

constexpr int NUM_PARTICLES = 500;

// The size of the spawn box around the robot's center (in meters)
constexpr double POSITION_VARIANCE = 0.15; 

// The variance in the starting angle. 
// If you know you set the robot down facing roughly forward, 
// a +/- 15 degree variance (PI/12) is much better than a completely random 360 spin.
constexpr double YAW_VARIANCE = M_PI / 12.0; 

std::vector<Particle> initializeParticles(double start_x, double start_y, double start_yaw) {
    std::vector<Particle> particles;
    particles.reserve(NUM_PARTICLES); 

    std::random_device rd;
    std::mt19937 gen(rd());

    // Center the random distribution exactly around the provided starting coordinates
    std::uniform_real_distribution<double> dist_x(start_x - POSITION_VARIANCE, start_x + POSITION_VARIANCE);
    std::uniform_real_distribution<double> dist_y(start_y - POSITION_VARIANCE, start_y + POSITION_VARIANCE);
    std::uniform_real_distribution<double> dist_theta(start_yaw - YAW_VARIANCE, start_yaw + YAW_VARIANCE);

    double initial_weight = 1.0 / NUM_PARTICLES;

    for (int i = 0; i < NUM_PARTICLES; ++i) {
        Particle p;
        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = dist_theta(gen);
        p.weight = initial_weight;
        
        particles.push_back(p);
    }

    return particles;
}