#include <unordered_map>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

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

std::unordered_map<long long, Point3D> parsePoints3D(const std::string& filePath)
{
    std::ifstream file(filePath); // Open the file for reading

    // Always check if the file opened successfully
    if (!file.is_open()) {
        std::cerr << "Error: Could not open the file " << filePath << std::endl;
        return;
    }

    std::string line;
    std::unordered_map<long long, Point3D> points;
    while (std::getline(file, line)) { // Read line by line
        if (line.empty() || line[0] == '#')
        {
            continue;
        }

        std::istringstream stream(line);

        long long id;
        Point3D point;

        stream 
            >> id
            >> point.x >> point.y >> point.z
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

/*
image format:
IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
POINTS2D[] as (X, Y, POINT3D_ID)
- IMAGE_ID is the key for this image. Like POINT3D_ID, it is an ID, not necessarily an array index.
- CAMERA_ID links this image to intrinsics in cameras.txt.
- qw qx qy qz tx ty tz is the image pose. COLMAP stores a world-to-camera transform: camera_space_point = R * world_point + t

parse and store:
- image name
- camera id
- quaternion + translation
- 2D points with POINT3D_ID links
*/
std::unordered_map<int, Image> parseImages(const std::string& filePath)
{
    std::ifstream file(filePath);
    std::unordered_map<int, Image> images;

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }

        Image image;

        std::istringstream imageLine(line);
        imageLine >> image.imageId
                >> image.qw >> image.qx >> image.qy >> image.qz
                >> image.tx >> image.ty >> image.tz
                >> image.cameraId
                >> image.name;
        
        std::string pointsLine;
        if (!std::getline(file, pointsLine)) {
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

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') {
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