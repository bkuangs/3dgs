// Inputs: Text files and hash maps
// Outputs: Validation statistics
// Description: Check that the parsed files were read correctly before rendering

#include "parse_colmap.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <unordered_map>

static bool projectPoint(
    const Camera& camera,
    const Vec3& xCam,
    double& u,
    double& v)
{
    if (xCam.z <= 0.0) {
        return false;
    }

    const double x = xCam.x / xCam.z;
    const double y = xCam.y / xCam.z;

    if (camera.model == "SIMPLE_PINHOLE" && camera.params.size() >= 3) {
        const double f = camera.params[0];
        const double cx = camera.params[1];
        const double cy = camera.params[2];
        u = f * x + cx;
        v = f * y + cy;
        return true;
    }

    if (camera.model == "PINHOLE" && camera.params.size() >= 4) {
        const double fx = camera.params[0];
        const double fy = camera.params[1];
        const double cx = camera.params[2];
        const double cy = camera.params[3];
        u = fx * x + cx;
        v = fy * y + cy;
        return true;
    }

    if (camera.model == "SIMPLE_RADIAL" && camera.params.size() >= 4) {
        const double f = camera.params[0];
        const double cx = camera.params[1];
        const double cy = camera.params[2];
        const double k = camera.params[3];
        const double r2 = x * x + y * y;
        const double radial = 1.0 + k * r2;
        u = f * radial * x + cx;
        v = f * radial * y + cy;
        return true;
    }

    std::cerr << "Unsupported camera model or missing params: "
              << camera.model << " for camera " << camera.cameraId << "\n";
    return false;
}

int main(int argc, char** argv)
{
    const std::string sparseDir = argc > 1 ? argv[1] : "data/colmap/sparse";
    const auto cameras = parseCameras(sparseDir + "/cameras.txt");
    const auto images = parseImages(sparseDir + "/images.txt");
    const auto points = parsePoints3D(sparseDir + "/points3D.txt");

    std::cout << "Loaded " << cameras.size() << " cameras, "
              << images.size() << " images, "
              << points.size() << " points3D\n";

    if (cameras.empty() || images.empty() || points.empty()) {
        return 1;
    }

    double sumError = 0.0;
    double minError = std::numeric_limits<double>::infinity();
    double maxError = 0.0;
    size_t checked = 0;
    size_t behindCamera = 0;
    size_t missingPoints = 0;

    std::cout << "\nCamera centers from C = -R^T * t:\n";

    for (const auto& [imageId, image] : images) {
        const auto cameraIt = cameras.find(image.cameraId);
        if (cameraIt == cameras.end()) {
            std::cerr << "Image " << imageId << " references missing camera "
                      << image.cameraId << "\n";
            continue;
        }

        const Mat3 r = quaternionToRotation(image.qw, image.qx, image.qy, image.qz);
        const Vec3 center = negate(multiplyTranspose(r, image.t));

        std::cout << image.name << " "
                  << center.x << " "
                  << center.y << " "
                  << center.z << "\n";

        const Camera& camera = cameraIt->second;
        for (const Point2D& observation : image.points2D) {
            if (observation.point3DId == -1) {
                continue;
            }

            const auto pointIt = points.find(observation.point3DId);
            if (pointIt == points.end()) {
                ++missingPoints;
                continue;
            }

            const Vec3 xCam = add(multiply(r, pointIt->second.xyz), image.t);

            double u = 0.0;
            double v = 0.0;
            if (!projectPoint(camera, xCam, u, v)) {
                if (xCam.z <= 0.0) {
                    ++behindCamera;
                }
                continue;
            }

            const double dx = u - observation.x;
            const double dy = v - observation.y;
            const double error = std::sqrt(dx * dx + dy * dy);

            sumError += error;
            minError = std::min(minError, error);
            maxError = std::max(maxError, error);
            ++checked;
        }
    }

    std::cout << "\nReprojection validation:\n";
    std::cout << "checked observations: " << checked << "\n";
    std::cout << "missing points: " << missingPoints << "\n";
    std::cout << "behind camera: " << behindCamera << "\n";

    if (checked > 0) {
        std::cout << "mean error px: " << (sumError / checked) << "\n";
        std::cout << "min error px: " << minError << "\n";
        std::cout << "max error px: " << maxError << "\n";
    }

    return checked > 0 ? 0 : 1;
}
