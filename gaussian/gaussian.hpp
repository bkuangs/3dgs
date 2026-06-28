#pragma once

#include <array>

#include <glm/vec3.hpp>

constexpr int SH_COUNT = 16;
constexpr int SH_CHANNEL_COUNT = 3;
constexpr int SH_FLOAT_COUNT = SH_COUNT * SH_CHANNEL_COUNT;

struct GaussianSplat {
    glm::vec3 centroid = glm::vec3(0.0f);
    float opacity = 0.0f;
    std::array<float, SH_FLOAT_COUNT> sphericalHarmonics = {};
    std::array<float, 3> scale = {0.0f, 0.0f, 0.0f};
    std::array<float, 4> rotation = {1.0f, 0.0f, 0.0f, 0.0f};
};
