#define GL_SILENCE_DEPRECATION

#include "parse_colmap.hpp"

#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

struct PlyProperty {
    std::string name;
};

struct PointVertex {
    glm::vec3 position = glm::vec3(0.0f);
    glm::vec3 color = glm::vec3(1.0f);
};

struct LineVertex {
    glm::vec3 position = glm::vec3(0.0f);
    glm::vec3 color = glm::vec3(1.0f);
};

struct Bounds {
    glm::vec3 min = glm::vec3(std::numeric_limits<float>::max());
    glm::vec3 max = glm::vec3(std::numeric_limits<float>::lowest());
};

struct InputState {
    bool keys[GLFW_KEY_LAST + 1] = {};
    bool leftMouse = false;
    bool rightMouse = false;
    bool middleMouse = false;
    bool hasMouse = false;
    double lastX = 0.0;
    double lastY = 0.0;
};

struct AppState {
    std::vector<PointVertex> points;
    std::vector<LineVertex> frustumLines;
    Bounds bounds;
    glm::vec3 target = glm::vec3(0.0f);
    float radius = 1.0f;
    float distance = 3.0f;
    float yaw = 0.0f;
    float pitch = 0.0f;
    glm::vec3 freePosition = glm::vec3(0.0f, 0.0f, 3.0f);
    float freeYaw = 0.0f;
    float freePitch = 0.0f;
    float pointSize = 2.0f;
    bool orbitMode = true;
    InputState input;
};

static AppState* g_app = nullptr;

static float clampFloat(float value, float minValue, float maxValue)
{
    return std::max(minValue, std::min(value, maxValue));
}

static std::vector<std::string> splitWords(const std::string& line)
{
    std::istringstream stream(line);
    std::vector<std::string> words;
    std::string word;
    while (stream >> word) {
        words.push_back(word);
    }
    return words;
}

static int findProperty(const std::vector<PlyProperty>& properties, const std::string& name)
{
    for (size_t i = 0; i < properties.size(); ++i) {
        if (properties[i].name == name) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

static float parseFloatAt(const std::vector<std::string>& values, int index, float fallback)
{
    if (index < 0 || static_cast<size_t>(index) >= values.size()) {
        return fallback;
    }
    return std::stof(values[static_cast<size_t>(index)]);
}

static glm::vec3 parseColorAt(
    const std::vector<std::string>& values,
    int redIndex,
    int greenIndex,
    int blueIndex)
{
    if (redIndex < 0 || greenIndex < 0 || blueIndex < 0) {
        return glm::vec3(1.0f);
    }

    const float red = parseFloatAt(values, redIndex, 255.0f);
    const float green = parseFloatAt(values, greenIndex, 255.0f);
    const float blue = parseFloatAt(values, blueIndex, 255.0f);
    return glm::vec3(red, green, blue) / 255.0f;
}

static bool loadAsciiPly(const std::string& path, std::vector<PointVertex>& points, Bounds& bounds)
{
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Could not open PLY " << path << "\n";
        return false;
    }

    std::string line;
    if (!std::getline(file, line) || line != "ply") {
        std::cerr << path << " is not a PLY file\n";
        return false;
    }

    size_t vertexCount = 0;
    bool ascii = false;
    bool readingVertexProperties = false;
    std::vector<PlyProperty> properties;

    while (std::getline(file, line)) {
        const std::vector<std::string> words = splitWords(line);
        if (words.empty()) {
            continue;
        }

        if (words[0] == "format" && words.size() >= 2) {
            ascii = words[1] == "ascii";
        } else if (words[0] == "element" && words.size() >= 3) {
            readingVertexProperties = words[1] == "vertex";
            if (readingVertexProperties) {
                vertexCount = static_cast<size_t>(std::stoull(words[2]));
            }
        } else if (words[0] == "property" && readingVertexProperties && words.size() >= 3) {
            properties.push_back({words.back()});
        } else if (words[0] == "end_header") {
            break;
        }
    }

    if (!ascii) {
        std::cerr << "Only ASCII PLY is supported in this MVP\n";
        return false;
    }

    const int xIndex = findProperty(properties, "x");
    const int yIndex = findProperty(properties, "y");
    const int zIndex = findProperty(properties, "z");
    int redIndex = findProperty(properties, "red");
    int greenIndex = findProperty(properties, "green");
    int blueIndex = findProperty(properties, "blue");
    if (redIndex < 0) {
        redIndex = findProperty(properties, "r");
    }
    if (greenIndex < 0) {
        greenIndex = findProperty(properties, "g");
    }
    if (blueIndex < 0) {
        blueIndex = findProperty(properties, "b");
    }

    if (xIndex < 0 || yIndex < 0 || zIndex < 0) {
        std::cerr << "PLY is missing x/y/z vertex properties\n";
        return false;
    }

    points.clear();
    points.reserve(vertexCount);

    for (size_t i = 0; i < vertexCount && std::getline(file, line); ++i) {
        const std::vector<std::string> values = splitWords(line);
        if (values.size() < properties.size()) {
            continue;
        }

        PointVertex point;
        point.position = glm::vec3(
            parseFloatAt(values, xIndex, 0.0f),
            parseFloatAt(values, yIndex, 0.0f),
            parseFloatAt(values, zIndex, 0.0f));
        point.color = parseColorAt(values, redIndex, greenIndex, blueIndex);
        points.push_back(point);

        bounds.min = glm::min(bounds.min, point.position);
        bounds.max = glm::max(bounds.max, point.position);
    }

    if (points.empty()) {
        std::cerr << "PLY contains no readable vertices\n";
        return false;
    }

    return true;
}

static glm::vec3 toGlm(const Vec3& v)
{
    return glm::vec3(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
}

static glm::vec3 rotateTranspose(const Mat3& r, const glm::vec3& v)
{
    return glm::vec3(
        static_cast<float>(r.m[0][0] * v.x + r.m[1][0] * v.y + r.m[2][0] * v.z),
        static_cast<float>(r.m[0][1] * v.x + r.m[1][1] * v.y + r.m[2][1] * v.z),
        static_cast<float>(r.m[0][2] * v.x + r.m[1][2] * v.y + r.m[2][2] * v.z));
}

static void addLine(
    std::vector<LineVertex>& lines,
    const glm::vec3& a,
    const glm::vec3& b,
    const glm::vec3& color)
{
    lines.push_back({a, color});
    lines.push_back({b, color});
}

static bool cameraIntrinsics(
    const Camera& camera,
    float& fx,
    float& fy,
    float& cx,
    float& cy)
{
    if (camera.model == "SIMPLE_PINHOLE" && camera.params.size() >= 3) {
        fx = fy = static_cast<float>(camera.params[0]);
        cx = static_cast<float>(camera.params[1]);
        cy = static_cast<float>(camera.params[2]);
        return true;
    }

    if (camera.model == "PINHOLE" && camera.params.size() >= 4) {
        fx = static_cast<float>(camera.params[0]);
        fy = static_cast<float>(camera.params[1]);
        cx = static_cast<float>(camera.params[2]);
        cy = static_cast<float>(camera.params[3]);
        return true;
    }

    if (camera.model == "SIMPLE_RADIAL" && camera.params.size() >= 4) {
        fx = fy = static_cast<float>(camera.params[0]);
        cx = static_cast<float>(camera.params[1]);
        cy = static_cast<float>(camera.params[2]);
        return true;
    }

    fx = fy = static_cast<float>(std::max(camera.width, camera.height));
    cx = static_cast<float>(camera.width) * 0.5f;
    cy = static_cast<float>(camera.height) * 0.5f;
    return camera.width > 0 && camera.height > 0;
}

static std::vector<LineVertex> loadCameraFrustums(const std::string& sparseDir, float sceneRadius)
{
    const auto cameras = parseCameras(sparseDir + "/cameras.txt");
    const auto images = parseImages(sparseDir + "/images.txt");
    std::vector<LineVertex> lines;

    if (cameras.empty() || images.empty()) {
        return lines;
    }

    const float depth = std::max(sceneRadius * 0.08f, 0.05f);
    const glm::vec3 color(1.0f, 0.82f, 0.2f);

    for (const auto& [imageId, image] : images) {
        const auto cameraIt = cameras.find(image.cameraId);
        if (cameraIt == cameras.end()) {
            continue;
        }

        const Camera& camera = cameraIt->second;
        float fx = 1.0f;
        float fy = 1.0f;
        float cx = 0.0f;
        float cy = 0.0f;
        if (!cameraIntrinsics(camera, fx, fy, cx, cy)) {
            continue;
        }

        const Mat3 r = quaternionToRotation(image.qw, image.qx, image.qy, image.qz);
        const glm::vec3 center = toGlm(negate(multiplyTranspose(r, image.t)));

        const float width = static_cast<float>(camera.width);
        const float height = static_cast<float>(camera.height);
        const glm::vec3 cornersCam[4] = {
            glm::vec3((0.0f - cx) / fx * depth, (0.0f - cy) / fy * depth, depth),
            glm::vec3((width - cx) / fx * depth, (0.0f - cy) / fy * depth, depth),
            glm::vec3((width - cx) / fx * depth, (height - cy) / fy * depth, depth),
            glm::vec3((0.0f - cx) / fx * depth, (height - cy) / fy * depth, depth),
        };

        glm::vec3 cornersWorld[4];
        for (int i = 0; i < 4; ++i) {
            cornersWorld[i] = center + rotateTranspose(r, cornersCam[i]);
            addLine(lines, center, cornersWorld[i], color);
        }

        addLine(lines, cornersWorld[0], cornersWorld[1], color);
        addLine(lines, cornersWorld[1], cornersWorld[2], color);
        addLine(lines, cornersWorld[2], cornersWorld[3], color);
        addLine(lines, cornersWorld[3], cornersWorld[0], color);
    }

    return lines;
}

static glm::vec3 viewDirection(float yaw, float pitch)
{
    return glm::normalize(glm::vec3(
        std::cos(pitch) * std::sin(yaw),
        std::sin(pitch),
        -std::cos(pitch) * std::cos(yaw)));
}

static glm::vec3 orbitPosition(const AppState& app)
{
    return app.target - viewDirection(app.yaw, app.pitch) * app.distance;
}

static glm::mat4 viewMatrix(const AppState& app)
{
    if (app.orbitMode) {
        const glm::vec3 position = orbitPosition(app);
        return glm::lookAt(position, app.target, glm::vec3(0.0f, 1.0f, 0.0f));
    }

    const glm::vec3 direction = viewDirection(app.freeYaw, app.freePitch);
    return glm::lookAt(app.freePosition, app.freePosition + direction, glm::vec3(0.0f, 1.0f, 0.0f));
}

static void updateFreeCamera(AppState& app, float dt)
{
    if (app.orbitMode) {
        return;
    }

    const glm::vec3 forward = viewDirection(app.freeYaw, app.freePitch);
    const glm::vec3 right = glm::normalize(glm::cross(forward, glm::vec3(0.0f, 1.0f, 0.0f)));
    const glm::vec3 up(0.0f, 1.0f, 0.0f);
    const float speed = (app.input.keys[GLFW_KEY_LEFT_SHIFT] || app.input.keys[GLFW_KEY_RIGHT_SHIFT])
        ? app.radius * 3.0f
        : app.radius;
    const float step = speed * dt;

    if (app.input.keys[GLFW_KEY_W]) {
        app.freePosition += forward * step;
    }
    if (app.input.keys[GLFW_KEY_S]) {
        app.freePosition -= forward * step;
    }
    if (app.input.keys[GLFW_KEY_D]) {
        app.freePosition += right * step;
    }
    if (app.input.keys[GLFW_KEY_A]) {
        app.freePosition -= right * step;
    }
    if (app.input.keys[GLFW_KEY_E]) {
        app.freePosition += up * step;
    }
    if (app.input.keys[GLFW_KEY_Q]) {
        app.freePosition -= up * step;
    }
}

static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    (void)scancode;
    (void)mods;

    if (!g_app || key < 0 || key > GLFW_KEY_LAST) {
        return;
    }

    if (action == GLFW_PRESS) {
        g_app->input.keys[key] = true;

        if (key == GLFW_KEY_ESCAPE) {
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        } else if (key == GLFW_KEY_TAB) {
            if (g_app->orbitMode) {
                g_app->freePosition = orbitPosition(*g_app);
                g_app->freeYaw = g_app->yaw;
                g_app->freePitch = g_app->pitch;
            } else {
                g_app->target = g_app->freePosition + viewDirection(g_app->freeYaw, g_app->freePitch) * g_app->distance;
                g_app->yaw = g_app->freeYaw;
                g_app->pitch = g_app->freePitch;
            }
            g_app->orbitMode = !g_app->orbitMode;
            std::cout << (g_app->orbitMode ? "Orbit camera\n" : "Free camera\n");
        } else if (key == GLFW_KEY_EQUAL || key == GLFW_KEY_KP_ADD || key == GLFW_KEY_RIGHT_BRACKET) {
            g_app->pointSize = clampFloat(g_app->pointSize + 1.0f, 1.0f, 20.0f);
            std::cout << "Point size: " << g_app->pointSize << "\n";
        } else if (key == GLFW_KEY_MINUS || key == GLFW_KEY_KP_SUBTRACT || key == GLFW_KEY_LEFT_BRACKET) {
            g_app->pointSize = clampFloat(g_app->pointSize - 1.0f, 1.0f, 20.0f);
            std::cout << "Point size: " << g_app->pointSize << "\n";
        }
    } else if (action == GLFW_RELEASE) {
        g_app->input.keys[key] = false;
    }
}

static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods)
{
    (void)window;
    (void)mods;

    if (!g_app) {
        return;
    }

    const bool pressed = action == GLFW_PRESS;
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        g_app->input.leftMouse = pressed;
    } else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        g_app->input.rightMouse = pressed;
    } else if (button == GLFW_MOUSE_BUTTON_MIDDLE) {
        g_app->input.middleMouse = pressed;
    }
}

static void cursorCallback(GLFWwindow* window, double x, double y)
{
    (void)window;

    if (!g_app) {
        return;
    }

    if (!g_app->input.hasMouse) {
        g_app->input.lastX = x;
        g_app->input.lastY = y;
        g_app->input.hasMouse = true;
        return;
    }

    const double dx = x - g_app->input.lastX;
    const double dy = y - g_app->input.lastY;
    g_app->input.lastX = x;
    g_app->input.lastY = y;

    const float rotateScale = 0.005f;
    const float panScale = std::max(g_app->distance, 0.001f) * 0.0015f;
    const float maxPitch = 1.5f;

    if (g_app->orbitMode) {
        if (g_app->input.leftMouse) {
            g_app->yaw -= static_cast<float>(dx) * rotateScale;
            g_app->pitch = clampFloat(g_app->pitch - static_cast<float>(dy) * rotateScale, -maxPitch, maxPitch);
        }

        if (g_app->input.rightMouse || g_app->input.middleMouse) {
            const glm::vec3 forward = viewDirection(g_app->yaw, g_app->pitch);
            const glm::vec3 right = glm::normalize(glm::cross(forward, glm::vec3(0.0f, 1.0f, 0.0f)));
            const glm::vec3 up = glm::normalize(glm::cross(right, forward));
            g_app->target -= right * static_cast<float>(dx) * panScale;
            g_app->target += up * static_cast<float>(dy) * panScale;
        }
    } else if (g_app->input.rightMouse) {
        g_app->freeYaw += static_cast<float>(dx) * rotateScale;
        g_app->freePitch = clampFloat(g_app->freePitch - static_cast<float>(dy) * rotateScale, -maxPitch, maxPitch);
    }
}

static void scrollCallback(GLFWwindow* window, double xOffset, double yOffset)
{
    (void)window;
    (void)xOffset;

    if (!g_app) {
        return;
    }

    if (g_app->orbitMode) {
        g_app->distance *= std::pow(0.9f, static_cast<float>(yOffset));
        g_app->distance = clampFloat(g_app->distance, g_app->radius * 0.02f, g_app->radius * 100.0f);
    } else {
        g_app->pointSize = clampFloat(g_app->pointSize + static_cast<float>(yOffset), 1.0f, 20.0f);
        std::cout << "Point size: " << g_app->pointSize << "\n";
    }
}

static void render(const AppState& app, int width, int height)
{
    glViewport(0, 0, width, height);
    glClearColor(0.05f, 0.055f, 0.065f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    const float aspect = height > 0 ? static_cast<float>(width) / static_cast<float>(height) : 1.0f;
    const glm::mat4 projection = glm::perspective(glm::radians(55.0f), aspect, app.radius * 0.001f, app.radius * 200.0f);
    const glm::mat4 view = viewMatrix(app);

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(glm::value_ptr(projection));
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(glm::value_ptr(view));

    glEnable(GL_DEPTH_TEST);
    glPointSize(app.pointSize);

    glBegin(GL_POINTS);
    for (const PointVertex& point : app.points) {
        glColor3f(point.color.r, point.color.g, point.color.b);
        glVertex3f(point.position.x, point.position.y, point.position.z);
    }
    glEnd();

    if (!app.frustumLines.empty()) {
        glLineWidth(1.0f);
        glBegin(GL_LINES);
        for (const LineVertex& vertex : app.frustumLines) {
            glColor3f(vertex.color.r, vertex.color.g, vertex.color.b);
            glVertex3f(vertex.position.x, vertex.position.y, vertex.position.z);
        }
        glEnd();
    }
}

int main(int argc, char** argv)
{
    bool checkOnly = false;
    std::vector<std::string> args;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--check") {
            checkOnly = true;
        } else {
            args.push_back(arg);
        }
    }

    if (args.empty()) {
        std::cerr << "Usage: viewer [--check] <points.ply> [colmap_sparse_dir]\n";
        return 1;
    }

    AppState app;
    if (!loadAsciiPly(args[0], app.points, app.bounds)) {
        return 1;
    }

    app.target = (app.bounds.min + app.bounds.max) * 0.5f;
    app.radius = glm::length(app.bounds.max - app.bounds.min) * 0.5f;
    if (app.radius <= 0.0f) {
        app.radius = 1.0f;
    }
    app.distance = app.radius * 2.5f;
    app.freePosition = orbitPosition(app);

    if (args.size() > 1) {
        app.frustumLines = loadCameraFrustums(args[1], app.radius);
    }

    if (checkOnly) {
        std::cout << "Check OK: loaded " << app.points.size() << " points";
        if (args.size() > 1) {
            std::cout << " and " << (app.frustumLines.size() / 16) << " camera frustums";
        }
        std::cout << "\n";
        return 0;
    }

    std::cout << "Loaded " << app.points.size() << " points";
    if (!app.frustumLines.empty()) {
        std::cout << " and " << (app.frustumLines.size() / 16) << " camera frustums";
    }
    std::cout << "\n";
    std::cout << "Controls: left-drag orbit, right/middle-drag pan, scroll zoom, Tab free/orbit, WASD/QE free move, +/- point size\n";

    if (!glfwInit()) {
        std::cerr << "Could not initialize GLFW\n";
        return 1;
    }

    GLFWwindow* window = glfwCreateWindow(1280, 800, "3DGS Viewer MVP", nullptr, nullptr);
    if (!window) {
        std::cerr << "Could not create GLFW window\n";
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    g_app = &app;
    glfwSetKeyCallback(window, keyCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetCursorPosCallback(window, cursorCallback);
    glfwSetScrollCallback(window, scrollCallback);

    double lastTime = glfwGetTime();
    while (!glfwWindowShouldClose(window)) {
        const double currentTime = glfwGetTime();
        const float dt = static_cast<float>(currentTime - lastTime);
        lastTime = currentTime;

        updateFreeCamera(app, dt);

        int width = 0;
        int height = 0;
        glfwGetFramebufferSize(window, &width, &height);
        render(app, width, height);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    g_app = nullptr;
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
