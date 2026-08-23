#pragma once

#include <algorithm>
#include <cmath>

struct Vec3 {
    float x;
    float y;
    float z;

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }

    float length() const { return std::sqrt(x * x + y * y + z * z); }
};

inline float dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

struct Mat3 {
    float m[3][3];

    Vec3 transform(const Vec3& v) const {
        return {
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z,
        };
    }
};

struct Transform {
    Vec3 pos = {0.0f, 0.0f, 0.0f};
    Vec3 rot = {0.0f, 0.0f, 0.0f};

    Mat3 getMatrix() const {
        constexpr float kPi = 3.14159265358979323846f;
        const float rx = rot.x * (kPi / 180.0f);
        const float ry = rot.y * (kPi / 180.0f);
        const float rz = rot.z * (kPi / 180.0f);

        const float cx = std::cos(rx);
        const float sx = std::sin(rx);
        const float cy = std::cos(ry);
        const float sy = std::sin(ry);
        const float cz = std::cos(rz);
        const float sz = std::sin(rz);

        return Mat3{{
            {cy * cz, -cy * sz, sy},
            {cx * sz + sx * sy * cz, cx * cz - sx * sy * sz, -sx * cy},
            {sx * sz - cx * sy * cz, sx * cz + cx * sy * sz, cx * cy},
        }};
    }
};

struct AABB {
    Vec3 min = {1e9f, 1e9f, 1e9f};
    Vec3 max = {-1e9f, -1e9f, -1e9f};

    void expand(const Vec3& p) {
        min.x = std::min(min.x, p.x);
        min.y = std::min(min.y, p.y);
        min.z = std::min(min.z, p.z);
        max.x = std::max(max.x, p.x);
        max.y = std::max(max.y, p.y);
        max.z = std::max(max.z, p.z);
    }

    float volume() const {
        if (max.x < min.x) {
            return 0.0f;
        }
        return (max.x - min.x) * (max.y - min.y) * (max.z - min.z);
    }

    bool intersects(const AABB& o) const {
        return (min.x <= o.max.x && max.x >= o.min.x) &&
               (min.y <= o.max.y && max.y >= o.min.y) &&
               (min.z <= o.max.z && max.z >= o.min.z);
    }
};
