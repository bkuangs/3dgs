#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Mat3 {
    double m[3][3] = {};
};

struct Camera {
    int cameraId = -1;
    std::string model;
    int width = 0;
    int height = 0;
    std::vector<double> params;
};

struct Point2D {
    double x = 0.0;
    double y = 0.0;
    long long point3DId = -1;
};

struct Image {
    int imageId = -1;
    double qw = 1.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    Vec3 t;
    int cameraId = -1;
    std::string name;
    std::vector<Point2D> points2D;
};

struct Point3D {
    Vec3 xyz;
    int r = 0;
    int g = 0;
    int b = 0;
    double error = 0.0;
};

static bool isDataLine(const std::string& line)
{
    return !line.empty() && line[0] != '#';
}

static std::string trimLeft(std::string value)
{
    const size_t start = value.find_first_not_of(" \t");
    if (start == std::string::npos) {
        return "";
    }
    return value.substr(start);
}

static Mat3 quaternionToRotation(double qw, double qx, double qy, double qz)
{
    const double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    qw /= norm;
    qx /= norm;
    qy /= norm;
    qz /= norm;

    Mat3 r;
    r.m[0][0] = 1.0 - 2.0 * (qy * qy + qz * qz);
    r.m[0][1] = 2.0 * (qx * qy - qz * qw);
    r.m[0][2] = 2.0 * (qx * qz + qy * qw);

    r.m[1][0] = 2.0 * (qx * qy + qz * qw);
    r.m[1][1] = 1.0 - 2.0 * (qx * qx + qz * qz);
    r.m[1][2] = 2.0 * (qy * qz - qx * qw);

    r.m[2][0] = 2.0 * (qx * qz - qy * qw);
    r.m[2][1] = 2.0 * (qy * qz + qx * qw);
    r.m[2][2] = 1.0 - 2.0 * (qx * qx + qy * qy);
    return r;
}

static Vec3 multiply(const Mat3& r, const Vec3& v)
{
    return {
        r.m[0][0] * v.x + r.m[0][1] * v.y + r.m[0][2] * v.z,
        r.m[1][0] * v.x + r.m[1][1] * v.y + r.m[1][2] * v.z,
        r.m[2][0] * v.x + r.m[2][1] * v.y + r.m[2][2] * v.z,
    };
}

static Vec3 multiplyTranspose(const Mat3& r, const Vec3& v)
{
    return {
        r.m[0][0] * v.x + r.m[1][0] * v.y + r.m[2][0] * v.z,
        r.m[0][1] * v.x + r.m[1][1] * v.y + r.m[2][1] * v.z,
        r.m[0][2] * v.x + r.m[1][2] * v.y + r.m[2][2] * v.z,
    };
}

static Vec3 add(const Vec3& a, const Vec3& b)
{
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

static Vec3 negate(const Vec3& v)
{
    return {-v.x, -v.y, -v.z};
}

static std::unordered_map<int, Camera> parseCameras(const std::string& path)
{
    std::ifstream file(path);
    std::unordered_map<int, Camera> cameras;

    if (!file.is_open()) {
        std::cerr << "Could not open " << path << "\n";
        return cameras;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (!isDataLine(line)) {
            continue;
        }

        Camera camera;
        std::istringstream stream(line);
        stream >> camera.cameraId >> camera.model >> camera.width >> camera.height;

        double param = 0.0;
        while (stream >> param) {
            camera.params.push_back(param);
        }

        cameras[camera.cameraId] = camera;
    }

    return cameras;
}

static std::unordered_map<int, Image> parseImages(const std::string& path)
{
    std::ifstream file(path);
    std::unordered_map<int, Image> images;

    if (!file.is_open()) {
        std::cerr << "Could not open " << path << "\n";
        return images;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (!isDataLine(line)) {
            continue;
        }

        Image image;
        std::istringstream imageLine(line);
        imageLine >> image.imageId
                  >> image.qw >> image.qx >> image.qy >> image.qz
                  >> image.t.x >> image.t.y >> image.t.z
                  >> image.cameraId;

        std::string name;
        std::getline(imageLine, name);
        image.name = trimLeft(name);

        std::string pointsLine;
        if (!std::getline(file, pointsLine)) {
            std::cerr << "Missing POINTS2D line after image " << image.imageId << "\n";
            break;
        }

        std::istringstream pointsStream(pointsLine);
        Point2D point;
        while (pointsStream >> point.x >> point.y >> point.point3DId) {
            image.points2D.push_back(point);
        }

        images[image.imageId] = image;
    }

    return images;
}

static std::unordered_map<long long, Point3D> parsePoints3D(const std::string& path)
{
    std::ifstream file(path);
    std::unordered_map<long long, Point3D> points;

    if (!file.is_open()) {
        std::cerr << "Could not open " << path << "\n";
        return points;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (!isDataLine(line)) {
            continue;
        }

        long long pointId = -1;
        Point3D point;
        std::istringstream stream(line);
        stream >> pointId
               >> point.xyz.x >> point.xyz.y >> point.xyz.z
               >> point.r >> point.g >> point.b
               >> point.error;

        points[pointId] = point;
    }

    return points;
}

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
