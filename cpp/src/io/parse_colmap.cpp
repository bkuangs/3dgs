#include "parse_colmap.hpp"

#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>

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

std::unordered_map<long long, Point3D> parsePoints3D(const std::string& filePath)
{
    std::ifstream file(filePath);
    std::unordered_map<long long, Point3D> points;

    if (!file.is_open()) {
        std::cerr << "Could not open " << filePath << "\n";
        return points;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (!isDataLine(line)) {
            continue;
        }

        std::istringstream stream(line);

        long long id;
        Point3D point;

        stream 
            >> id
            >> point.xyz.x >> point.xyz.y >> point.xyz.z
            >> point.r >> point.g >> point.b
            >> point.error;

        int imageId;
        int point2Idx;

        while (stream >> imageId >> point2Idx) 
        {
            point.track.push_back({imageId, point2Idx});
        }

        points[id] = point;
    }

    return points;
}

std::unordered_map<int, Image> parseImages(const std::string& filePath)
{
    std::ifstream file(filePath);
    std::unordered_map<int, Image> images;

    if (!file.is_open()) {
        std::cerr << "Could not open " << filePath << "\n";
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

std::unordered_map<int, Camera> parseCameras(const std::string& filePath)
{
    std::ifstream file(filePath);
    std::unordered_map<int, Camera> cameras;

    if (!file.is_open()) {
        std::cerr << "Could not open " << filePath << "\n";
        return cameras;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (!isDataLine(line)) {
            continue;
        }

        std::istringstream stream(line);

        Camera camera;
        stream >> camera.cameraId
               >> camera.model
               >> camera.width
               >> camera.height;

        double param;
        while (stream >> param) {
            camera.params.push_back(param);
        }

        cameras[camera.cameraId] = camera;
    }

    return cameras;
}

Mat3 quaternionToRotation(double qw, double qx, double qy, double qz)
{
    const double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    if (norm == 0.0) {
        return {};
    }

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

Vec3 multiply(const Mat3& r, const Vec3& v)
{
    return {
        r.m[0][0] * v.x + r.m[0][1] * v.y + r.m[0][2] * v.z,
        r.m[1][0] * v.x + r.m[1][1] * v.y + r.m[1][2] * v.z,
        r.m[2][0] * v.x + r.m[2][1] * v.y + r.m[2][2] * v.z,
    };
}

Vec3 multiplyTranspose(const Mat3& r, const Vec3& v)
{
    return {
        r.m[0][0] * v.x + r.m[1][0] * v.y + r.m[2][0] * v.z,
        r.m[0][1] * v.x + r.m[1][1] * v.y + r.m[2][1] * v.z,
        r.m[0][2] * v.x + r.m[1][2] * v.y + r.m[2][2] * v.z,
    };
}

Vec3 add(const Vec3& a, const Vec3& b)
{
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 negate(const Vec3& v)
{
    return {-v.x, -v.y, -v.z};
}
