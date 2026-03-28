#include <map.h>
#include <iostream>
#include <fstream>
#include <mapCreator.h>

Eigen::Vector2d ArenaMap::in2m(double x_inches, double y_inches) {
    const double INCH_TO_METER = 0.0254;
    return Eigen::Vector2d(x_inches * INCH_TO_METER, y_inches * INCH_TO_METER);
}

ArenaMap::ArenaMap() {
    std::cout << "Loading IEEE Vector Map (V4: Adding Teammate Obstacles)..." << std::endl;

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

    // ==========================================
    // NEW: TEAMMATE OBSTACLES (Already in Meters)
    // ==========================================
    // Object 1
    walls.push_back({Eigen::Vector2d(0.5906, 0.0), Eigen::Vector2d(0.7445, 0.0)});
    walls.push_back({Eigen::Vector2d(0.7445, 0.0), Eigen::Vector2d(0.7445, 0.1539)});
    walls.push_back({Eigen::Vector2d(0.7445, 0.1539), Eigen::Vector2d(0.5906, 0.1539)});
    walls.push_back({Eigen::Vector2d(0.5906, 0.1539), Eigen::Vector2d(0.5906, 0.0)});

    // Object 2
    walls.push_back({Eigen::Vector2d(1.2494, 0.9891), Eigen::Vector2d(1.4033, 0.9891)});
    walls.push_back({Eigen::Vector2d(1.4033, 0.9891), Eigen::Vector2d(1.4033, 1.1430)});
    walls.push_back({Eigen::Vector2d(1.4033, 1.1430), Eigen::Vector2d(1.2494, 1.1430)});
    walls.push_back({Eigen::Vector2d(1.2494, 1.1430), Eigen::Vector2d(1.2494, 0.9891)});
}

void ArenaMap::generateBoundaryGML(const std::string& filename) {
    std::ofstream gml_file(filename);

    if (!gml_file.is_open()) return;

    gml_file << "<?xml version=\"1.0\" encoding=\"utf-8\" ?>\n<ogr:FeatureCollection\n     xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\n     xmlns:ogr=\"http://ogr.maptools.org/\"\n     xmlns:gml=\"http://www.opengis.net/gml\">\n  <gml:featureMember>\n    <ogr:coverage_area>\n      <ogr:geometryProperty><gml:Polygon>\n";

    // 1. Outer Sweeping Perimeter
    gml_file << "        <gml:outerBoundaryIs><gml:LinearRing><gml:coordinates>\n          ";
    std::vector<Eigen::Vector2d> perimeter = {
        in2m(0, 0), in2m(55.25, 0), in2m(55.25, 14.5), in2m(61.75, 14.5),
        in2m(61.75, 0), in2m(89.5, 0), in2m(89.5, 1.5), in2m(93, 1.5),
        in2m(93, 43.5), in2m(89.5, 43.5), in2m(89.5, 45), in2m(61.75, 45),
        in2m(61.75, 30.5), in2m(55.25, 30.5), in2m(55.25, 45), in2m(0, 45), in2m(0, 0)
    };
    for (const auto& pt : perimeter) gml_file << pt.x() << "," << pt.y() << " ";
    gml_file << "\n        </gml:coordinates></gml:LinearRing></gml:outerBoundaryIs>\n";

    // 2. Object 1 Hole
    gml_file << "        <gml:innerBoundaryIs><gml:LinearRing><gml:coordinates>\n          ";
    gml_file << "0.5906,0.0 0.7445,0.0 0.7445,0.1539 0.5906,0.1539 0.5906,0.0";
    gml_file << "\n        </gml:coordinates></gml:LinearRing></gml:innerBoundaryIs>\n";

    // 3. Object 2 Hole
    gml_file << "        <gml:innerBoundaryIs><gml:LinearRing><gml:coordinates>\n          ";
    gml_file << "1.2494,0.9891 1.4033,0.9891 1.4033,1.1430 1.2494,1.1430 1.2494,0.9891";
    gml_file << "\n        </gml:coordinates></gml:LinearRing></gml:innerBoundaryIs>\n";

    gml_file << "      </gml:Polygon></ogr:geometryProperty>\n    </ogr:coverage_area>\n  </gml:featureMember>\n</ogr:FeatureCollection>\n";
    gml_file.close();
}

void ArenaMap::generatePGM(const std::string& filename) {
    PPMWriter ppm(1000, 600); // Scale 400 pixels/meter
    for (const auto& wall : walls) ppm.drawWall(wall);
    ppm.save(filename);
}