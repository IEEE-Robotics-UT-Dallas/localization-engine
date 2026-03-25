//
// Created by nokoru on 3/23/26.
//

#include <cmath>
#include <fstream>
#include <iostream>
#include <mapCreator.h>
#include <particle.h>

PPMWriter::PPMWriter(int w, int h) : width(w), height(h) {
    image_buffer.assign(width * height * 3, 255);
}

void PPMWriter::drawLineGeneric(int x0, int y0, int x1, int y1, int r, int g, int b) {
    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;

    while(true){
        setPixel(x0, y0, r, g, b);
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void PPMWriter::drawWall(const Wall& w) {
    int x0 = static_cast<int>(w.start_point.x() * scale) + margin;
    int y0 = height - static_cast<int>(w.start_point.y() * scale) - margin - 1;
    int x1 = static_cast<int>(w.end_point.x() * scale) + margin;
    int y1 = height - static_cast<int>(w.end_point.y() * scale) - margin - 1;
    drawLineGeneric(x0, y0, x1, y1, 0, 0, 255);
}

void PPMWriter::drawRobot(double robot_x, double robot_y, double robot_yaw) {
    int cx = static_cast<int>(robot_x * scale) + margin;
    int cy = height - static_cast<int>(robot_y * scale) - margin - 1;
    int dot_radius = 5;
    for (int dy = -dot_radius; dy <= dot_radius; ++dy) {
        for (int dx = -dot_radius; dx <= dot_radius; ++dx) {
            setPixel(cx + dx, cy + dy, 255, 0, 0);
        }
    }
    double heading_length_m = 8.0 * 0.0254;
    double end_x_m = robot_x + std::cos(robot_yaw) * heading_length_m;
    double end_y_m = robot_y + std::sin(robot_yaw) * heading_length_m;
    int ex = static_cast<int>(end_x_m * scale) + margin;
    int ey = height - static_cast<int>(end_y_m * scale) - margin - 1;
    drawLineGeneric(cx, cy, ex, ey, 255, 165, 0);
}

void PPMWriter::setPixel(int x, int y, int r, int g, int b) {
    if (x >= 0 && x < width && y >= 0 && y < height) {
        int index = (y * width + x) * 3;
        image_buffer[index] = r;
        image_buffer[index + 1] = g;
        image_buffer[index + 2] = b;
    }
}

void PPMWriter::save(const std::string& filename) {
    std::ofstream file(filename, std::ios::binary);

    // THE FIX: Actually check if the file was allowed to be created
    if (!file.is_open()) {
        std::cerr << "\n[ERROR] Docker blocked the file creation! It is lost in the void.\n" << std::endl;
        return;
    }

    file << "P3\n" << width << " " << height << "\n255\n";
    for (int i = 0; i < width * height * 3; ++i) {
        file << image_buffer[i] << " ";
        if ((i + 1) % (width * 3) == 0) file << "\n";
    }
    std::cout << "[SUCCESS] Map image saved safely to: " << filename << std::endl;
}

void PPMWriter::drawParticle(double x_meters, double y_meters) {
    // Match the exact coordinate math used in your drawRobot and drawWall functions
    int px = static_cast<int>(x_meters * scale) + margin;
    int py = height - static_cast<int>(y_meters * scale) - margin - 1;

    // Use your existing setPixel method! It already handles the safety bounds.
    // We draw a 2x2 bright green square so the particles are easily visible.
    setPixel(px, py, 0, 255, 0);
    setPixel(px + 1, py, 0, 255, 0);
    setPixel(px, py + 1, 0, 255, 0);
    setPixel(px + 1, py + 1, 0, 255, 0);
}

void PPMWriter::drawLaser(double start_x_m, double start_y_m, double end_x_m, double end_y_m) {
    // Convert the physical start and end meters into image pixels
    int x0 = static_cast<int>(start_x_m * scale) + margin;
    int y0 = height - static_cast<int>(start_y_m * scale) - margin - 1;
    int x1 = static_cast<int>(end_x_m * scale) + margin;
    int y1 = height - static_cast<int>(end_y_m * scale) - margin - 1;

    // Draw a bright yellow line from the sensor to the wall hit
    drawLineGeneric(x0, y0, x1, y1, 255, 255, 0);
}

void PPMWriter::drawParticleLaser(double start_x_m, double start_y_m, double end_x_m, double end_y_m) {
    int x0 = static_cast<int>(start_x_m * scale) + margin;
    int y0 = height - static_cast<int>(start_y_m * scale) - margin - 1;
    int x1 = static_cast<int>(end_x_m * scale) + margin;
    int y1 = height - static_cast<int>(end_y_m * scale) - margin - 1;

    // Draw a darker cyan line so it doesn't overpower the main robot
    drawLineGeneric(x0, y0, x1, y1, 230, 250, 250);
}