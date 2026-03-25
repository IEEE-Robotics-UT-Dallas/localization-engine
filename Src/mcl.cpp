//
// Created by nokoru on 3/24/26.
//

#include <mcl.h>
#include <cmath>
#include <limits>
#include <random>
#include <vector>
double calculateRayIntersection(double start_x, double start_y, double ray_yaw, const ArenaMap& map) {
    // Start with an infinitely long laser beam
    double min_distance = std::numeric_limits<double>::infinity();

    // The x and y components of the laser's direction
    double ray_dx = std::cos(ray_yaw);
    double ray_dy = std::sin(ray_yaw);

    for (const auto& wall : map.walls) {
        double w_x1 = wall.start_point.x();
        double w_y1 = wall.start_point.y();
        double w_x2 = wall.end_point.x();
        double w_y2 = wall.end_point.y();

        double wall_dx = w_x2 - w_x1;
        double wall_dy = w_y2 - w_y1;

        // Denominator to check if the laser is perfectly parallel to the wall
        double denominator = (ray_dx * wall_dy) - (ray_dy * wall_dx);

        // If denominator is near 0, they are parallel (no intersection)
        if (std::abs(denominator) < 1e-6) continue;

        double diff_x = w_x1 - start_x;
        double diff_y = w_y1 - start_y;

        // Solve for 't' (distance along laser) and 'u' (percent along wall)
        double t = (diff_x * wall_dy - diff_y * wall_dx) / denominator;
        double u = (diff_x * ray_dy - diff_y * ray_dx) / denominator;

        // Check if the intersection is valid:
        // t > 0 : The wall is in front of the sensor, not behind it
        // u >= 0 && u <= 1 : The laser hit the actual wall segment, not the empty space beyond it
        if (t > 0.0 && u >= 0.0 && u <= 1.0) {
            // Keep track of the closest wall hit
            if (t < min_distance) {
                min_distance = t;
            }
        }
    }

    return min_distance;
}

double calculateParticleLikelihood(double simulated_dist, double actual_dist, double sensor_noise_std) {
    // 1. Handle edge cases where lasers hit nothing (infinity)
    bool sim_inf = std::isinf(simulated_dist);
    bool act_inf = std::isinf(actual_dist);

    if (sim_inf && act_inf) return 1.0; // Both see open space - perfect match!
    if (sim_inf != act_inf) return 1e-9; // One sees a wall, one doesn't - very bad match

    // 2. The standard Gaussian (Normal) PDF formula:
    // P(x) = [ 1 / (std * sqrt(2*PI)) ] * exp( - (difference)^2 / (2 * std^2) )
    double diff = simulated_dist - actual_dist;
    double var = sensor_noise_std * sensor_noise_std;

    double numerator = std::exp(-(diff * diff) / (2 * var));
    double denominator = sensor_noise_std * std::sqrt(2.0 * M_PI);

    // We only care about the relative numerator value for probability,
    // but using the full formula is cleaner.
    return numerator / denominator;
}

std::vector<Particle> resampleParticles(const std::vector<Particle>& old_particles) {
    std::vector<Particle> new_particles;
    int num_particles = old_particles.size();
    new_particles.reserve(num_particles);

    // 1. Extract all the weights into a list for the lottery
    std::vector<double> weights;
    for (const auto& p : old_particles) {
        weights.push_back(p.weight);
    }

    // 2. Set up the random number generator and the weighted distribution
    std::random_device rd;
    std::mt19937 gen(rd());
    std::discrete_distribution<int> dist(weights.begin(), weights.end());

    // 3. The new baseline weight for the next generation
    double initial_weight = 1.0 / num_particles;

    // 4. Spin the loaded roulette wheel 500 times
    for (int i = 0; i < num_particles; ++i) {
        int chosen_index = dist(gen);

        // Clone the winning particle
        Particle surviving_particle = old_particles[chosen_index];

        // Reset its weight so the new generation starts evenly before the next movement
        surviving_particle.weight = initial_weight;

        new_particles.push_back(surviving_particle);
    }

    return new_particles;
}