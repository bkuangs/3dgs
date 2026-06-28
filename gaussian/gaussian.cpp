// Inputs: Hash maps
// Outputs: .ply files
// Description: Turn sparse point clouds into Gaussian blobs, as a PLY vertex

#include "parse_colmap.hpp"

#include <fstream>
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
    const std::string sparseDir = argc > 1 ? argv[1] : "data/colmap/sparse";
    const std::string outputPath = argc > 2 ? argv[2] : "data/colmap/gaussians_init.ply";

    const auto points = parsePoints3D(sparseDir + "/points3D.txt");
    if (points.empty()) {
        std::cerr << "No points loaded from " << sparseDir << "/points3D.txt\n";
        return 1;
    }

    std::ofstream out(outputPath);
    if (!out.is_open()) {
        std::cerr << "Could not write " << outputPath << "\n";
        return 1;
    }

    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "element vertex " << points.size() << "\n";
    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";
    out << "property uchar red\n";
    out << "property uchar green\n";
    out << "property uchar blue\n";
    out << "property float opacity\n";
    out << "property float scale_0\n";
    out << "property float scale_1\n";
    out << "property float scale_2\n";
    out << "property float rot_0\n";
    out << "property float rot_1\n";
    out << "property float rot_2\n";
    out << "property float rot_3\n";
    out << "end_header\n";

    for (const auto& [id, point] : points) {
        out << point.xyz.x << " "
            << point.xyz.y << " "
            << point.xyz.z << " "
            << point.r << " "
            << point.g << " "
            << point.b << " "
            << 1.0f << " "
            << 0.01f << " "
            << 0.01f << " "
            << 0.01f << " "
            << 1.0f << " "
            << 0.0f << " "
            << 0.0f << " "
            << 0.0f << "\n";
    }

    std::cout << "Wrote " << points.size() << " points to " << outputPath << "\n";
    return 0;
}
