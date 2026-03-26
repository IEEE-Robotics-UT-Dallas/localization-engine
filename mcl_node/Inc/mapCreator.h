//
// Created by nokoru on 3/23/26.
//

#ifndef LOCALIZATIONALGORITHM_MAPCREATOR_H
#define LOCALIZATIONALGORITHM_MAPCREATOR_H

#include <vector>
#include <string>
#include <map.h>

class PPMWriter {
private:
    int width, height;
    std::vector<int> image_buffer;
    double scale = 400.0;
    int margin = 20;

public:
    PPMWriter(int w, int h);
    void drawLineGeneric(int x0, int y0, int x1, int y1, int r, int g, int b);
    void drawWall(const Wall& w);
    void drawRobot(double robot_x, double robot_y, double robot_yaw);
    void setPixel(int x, int y, int r, int g, int b);
    void save(const std::string& filename);
    void drawParticle(double x_meters, double y_meters);
    void drawLaser(double start_x_m, double start_y_m, double end_x_m, double end_y_m);
    void drawParticleLaser(double start_x_m, double start_y_m, double end_x_m, double end_y_m);
};

#endif //LOCALIZATIONALGORITHM_MAPCREATOR_H