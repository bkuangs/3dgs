// Inputs: Hash maps
// Outputs: .ply files
// Description: Turn sparse point clouds into Gaussian blobs, as a PLY vertex

#include "parse_colmap.cpp"

int main(int argc, char** argv)
{
    std::string sparseDir = "data/colmap/sparse";
    std::string outputPath = "data/colmap/gaussians_init.ply";

    auto points = parsePoints3D(sparseDir + "/points3D.txt");

    std::ofstream out(outputPath);

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
        out << point.x << " "
            << point.y << " "
            << point.z << " "
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
}