#include <string>
#include <vector>

struct TrackElement
{
    int imageId;        // image number the point appeared in
    int point2Idx;      // 2d keypoint index in that image
};

struct Point2D
{
    double x, y;
    long long point3DId;    // -1 means no 3D match
};

struct Point3D
{
    double x, y, z;
    int r, g, b;
    double error;       // reprojection error, in pixels
    std::vector<TrackElement> track;
};

struct Image
{
    int imageId;
    double qw, qx, qy, qz;
    double tx, ty, tz;
    int cameraId;
    std::string name;
    std::vector<Point2D> points2D;
};

struct Camera {
    int cameraId;
    std::string model;
    int width;
    int height;
    std::vector<double> params;
};