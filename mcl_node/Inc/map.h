//
// Created by nokoru on 3/23/26.
//

#ifndef LOCALIZATIONALGORITHM_MAP_H
#define LOCALIZATIONALGORITHM_MAP_H

#include <vector>
#include <iostream>
#include <Eigen/Dense>

struct Wall {
    Eigen::Vector2d start_point;
    Eigen::Vector2d end_point;
};

class ArenaMap {
public:
    std::vector<Wall> walls;

    Eigen::Vector2d in2m(double x_inches, double y_inches);

    ArenaMap();

	void generateBoundaryGML(const std::string& filename);
};

#endif //LOCALIZATIONALGORITHM_MAP_H