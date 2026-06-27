#pragma once

#include <string>
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
};