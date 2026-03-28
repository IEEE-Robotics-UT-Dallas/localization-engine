//
// Created by nokoru on 3/23/26.
//
#include <map.h>
#include <iostream>
#include <fstream>

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

void ArenaMap::generateBoundaryGML(const std::string& filename) {
    std::ofstream gml_file(filename);

    if (!gml_file.is_open()) {
        std::cerr << "\n[ERROR] Docker blocked the GML file creation! It is lost in the void.\n" << std::endl;
        return;
    }

    // 1. Write the strict XML headers for opennav_coverage
    gml_file << "<?xml version=\"1.0\" encoding=\"utf-8\" ?>\n";
    gml_file << "<ogr:FeatureCollection\n";
    gml_file << "     xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\n";
    gml_file << "     xmlns:ogr=\"http://ogr.maptools.org/\"\n";
    gml_file << "     xmlns:gml=\"http://www.opengis.net/gml\">\n";
    gml_file << "  <gml:featureMember>\n";
    gml_file << "    <ogr:coverage_area>\n";
    gml_file << "      <ogr:geometryProperty><gml:Polygon><gml:outerBoundaryIs><gml:LinearRing><gml:coordinates>\n";
    gml_file << "        ";

    // 2. Extract the Outer Boundary points.
    // Since the first 4 walls in your constructor are the outer perimeter,
    // we just grab the start_point of those first 4 walls.
    for (int i = 0; i < 4; ++i) {
        gml_file << walls[i].start_point.x() << "," << walls[i].start_point.y() << " ";
    }

    // 3. Close the loop (Mandatory for GML)
    // We append the very first point again to perfectly seal the polygon.
    gml_file << walls[0].start_point.x() << "," << walls[0].start_point.y();

    // 4. Write the closing XML tags
    gml_file << "\n      </gml:coordinates></gml:LinearRing></gml:outerBoundaryIs></gml:Polygon></ogr:geometryProperty>\n";
    gml_file << "    </ogr:coverage_area>\n";
    gml_file << "  </gml:featureMember>\n";
    gml_file << "</ogr:FeatureCollection>\n";

    gml_file.close();
    std::cout << "[SUCCESS] Arena boundary GML saved safely to: " << filename << std::endl;
}