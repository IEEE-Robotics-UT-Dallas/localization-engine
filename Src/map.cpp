//
// Created by nokoru on 3/23/26.
//
#include <map.h>
#include <iostream>

Eigen::Vector2d ArenaMap::in2m(double x_inches, double y_inches) {
    const double INCH_TO_METER = 0.0254;
    return Eigen::Vector2d(x_inches * INCH_TO_METER, y_inches * INCH_TO_METER);
}

ArenaMap::ArenaMap() {
    std::cout << "Loading IEEE Vector Map (V3: Verified Geometry)..." << std::endl;

    // Outer Boundary
    walls.push_back({in2m(0, 0), in2m(93, 0)});
    walls.push_back({in2m(93, 0), in2m(93, 45)});
    walls.push_back({in2m(93, 45), in2m(0, 45)});
    walls.push_back({in2m(0, 45), in2m(0, 0)});

    // Bottom Bridge Base
    walls.push_back({in2m(55.25, 0), in2m(55.25, 14.5)});
    walls.push_back({in2m(55.25, 14.5), in2m(61.75, 14.5)});
    walls.push_back({in2m(61.75, 14.5), in2m(61.75, 0)});

    // Top Bridge Base
    walls.push_back({in2m(55.25, 45), in2m(55.25, 30.5)});
    walls.push_back({in2m(55.25, 30.5), in2m(61.75, 30.5)});
    walls.push_back({in2m(61.75, 30.5), in2m(61.75, 45)});

    // Correct 3.5"x1.5" Corner Stubs
    walls.push_back({in2m(89.5, 0), in2m(89.5, 1.5)});
    walls.push_back({in2m(89.5, 1.5), in2m(93, 1.5)});
    walls.push_back({in2m(89.5, 45), in2m(89.5, 43.5)});
    walls.push_back({in2m(89.5, 43.5), in2m(93, 43.5)});
}