#pragma once

#include <string>
#include <unordered_map>
#include <vector>

struct TrackElement
{
    int imageId;        // image number the point appeared in
    int point2Idx;      // 2d keypoint index in that image
};

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
    std::vector<TrackElement> track;
};

std::unordered_map<long long, Point3D> parsePoints3D(const std::string& filePath);
std::unordered_map<int, Image> parseImages(const std::string& filePath);
std::unordered_map<int, Camera> parseCameras(const std::string& filePath);

Mat3 quaternionToRotation(double qw, double qx, double qy, double qz);
Vec3 multiply(const Mat3& r, const Vec3& v);
Vec3 multiplyTranspose(const Mat3& r, const Vec3& v);
Vec3 add(const Vec3& a, const Vec3& b);
Vec3 negate(const Vec3& v);
