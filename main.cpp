#include <iostream>
#include <fstream>
#include <cmath>
#include "main.h" // Pulls in the Wall struct and ArenaMap class

class PPMWriter {
private:
    int width, height;
    std::vector<int> image_buffer;
    double scale = 400.0;
    int margin = 20;

public:
    PPMWriter(int w, int h) : width(w), height(h) {
        image_buffer.assign(width * height * 3, 255);
    }

    void drawLineGeneric(int x0, int y0, int x1, int y1, int r, int g, int b) {
        int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy, e2;

        while(true){
            setPixel(x0, y0, r, g, b);
            if (x0 == x1 && y0 == y1) break;
            e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }

    void drawWall(const Wall& w) {
        int x0 = static_cast<int>(w.start_point.x() * scale) + margin;
        int y0 = height - static_cast<int>(w.start_point.y() * scale) - margin - 1;
        int x1 = static_cast<int>(w.end_point.x() * scale) + margin;
        int y1 = height - static_cast<int>(w.end_point.y() * scale) - margin - 1;
        drawLineGeneric(x0, y0, x1, y1, 0, 0, 255);
    }

    void drawRobot(double robot_x, double robot_y, double robot_yaw) {
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

    void setPixel(int x, int y, int r, int g, int b) {
        if (x >= 0 && x < width && y >= 0 && y < height) {
            int index = (y * width + x) * 3;
            image_buffer[index] = r;
            image_buffer[index + 1] = g;
            image_buffer[index + 2] = b;
        }
    }

    void save(const std::string& filename) {
        std::ofstream file(filename, std::ios::binary);
        file << "P3\n" << width << " " << height << "\n255\n";
        for (int i = 0; i < width * height * 3; ++i) {
            file << image_buffer[i] << " ";
            if ((i + 1) % (width * 3) == 0) file << "\n";
        }
        std::cout << "Map image with robot saved to: " << filename << std::endl;
    }
};

int main() {
    ArenaMap game_field;
    PPMWriter writer(1000, 500);

    for (const auto& w : game_field.walls) {
        writer.drawWall(w);
    }

    double robot_start_x_m = game_field.in2m(31.25, 0).x();
    double robot_start_y_m = game_field.in2m(0, 6.5).y();
    double robot_start_yaw = M_PI / 2.0;

    writer.drawRobot(robot_start_x_m, robot_start_y_m, robot_start_yaw);
    writer.save("arena_map_with_robot.ppm");

    return 0;
}