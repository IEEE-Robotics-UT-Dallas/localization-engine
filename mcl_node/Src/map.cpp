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

    gml_file << "<?xml version=\"1.0\" encoding=\"utf-8\" ?>\n";
    gml_file << "<ogr:FeatureCollection\n";
    gml_file << "     xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\n";
    gml_file << "     xmlns:ogr=\"http://ogr.maptools.org/\"\n";
    gml_file << "     xmlns:gml=\"http://www.opengis.net/gml\">\n";
    gml_file << "  <gml:featureMember>\n";
    gml_file << "    <ogr:coverage_area>\n";
    gml_file << "      <ogr:geometryProperty><gml:Polygon><gml:outerBoundaryIs><gml:LinearRing><gml:coordinates>\n";
    gml_file << "        ";

    // Trace the continuous, concave perimeter of the arena in order
    std::vector<Eigen::Vector2d> perimeter = {
        in2m(0, 0),             // 1. Bottom Left Origin
        in2m(55.25, 0),         // 2. Right along bottom wall to Bottom Bridge Base
        in2m(55.25, 14.5),      // 3. Up the side of the bridge base
        in2m(61.75, 14.5),      // 4. Across the front of the bridge base
        in2m(61.75, 0),         // 5. Down the side of the bridge base
        in2m(89.5, 0),          // 6. Right along bottom wall to Corner Stub
        in2m(89.5, 1.5),        // 7. Up the side of the stub
        in2m(93, 1.5),          // 8. Across the stub to the Right Wall
        in2m(93, 43.5),         // 9. Up the Right Wall to the Top Corner Stub
        in2m(89.5, 43.5),       // 10. Left across the top stub
        in2m(89.5, 45),         // 11. Up the side of the stub to the Top Wall
        in2m(61.75, 45),        // 12. Left along top wall to Top Bridge Base
        in2m(61.75, 30.5),      // 13. Down the side of the top bridge base
        in2m(55.25, 30.5),      // 14. Across the front of the top bridge base
        in2m(55.25, 45),        // 15. Up the side of the top bridge base
        in2m(0, 45),            // 16. Left along top wall to Top Left corner
        in2m(0, 0)              // 17. Down the Left Wall back to Origin (Closes loop)
    };

    // Write the formatted coordinates
    for (const auto& pt : perimeter) {
        gml_file << pt.x() << "," << pt.y() << " ";
    }

    gml_file << "\n      </gml:coordinates></gml:LinearRing></gml:outerBoundaryIs></gml:Polygon></ogr:geometryProperty>\n";
    gml_file << "    </ogr:coverage_area>\n";
    gml_file << "  </gml:featureMember>\n";
    gml_file << "</ogr:FeatureCollection>\n";

    gml_file.close();
    std::cout << "[SUCCESS] Concave Arena boundary GML saved safely to: " << filename << std::endl;
}