#ifndef TOF_SENSOR_H
#define TOF_SENSOR_H

#include <vector>
#include "particle.h" // Ensure this points to where your Particle struct is defined
#include "map.h"      // Ensure this points to where ArenaMap is defined

// A simple blueprint for where a sensor physically lives on the robot
struct SensorMount {
    double dx;
    double dy;
    double yaw;
};

// The pure math function that will process the array and update particle weights
void applyToFMeasurement(std::vector<Particle>& particles, const std::vector<float>& tof_readings, const ArenaMap& map);

#endif // TOF_SENSOR_H