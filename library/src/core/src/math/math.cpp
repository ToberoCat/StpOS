//
// Created by tobias on 12/26/24.
//
#include "libstp/math/math.h"

#include <cmath>

float libstp::math::lerp(const float a, const float b, const float t)
{
    return a + t * (b - a);
}

float libstp::math::easeInOut(const float a, const float b, const float t)
{
    return lerp(a, b, static_cast<float>(1 - cos(t * M_PI)) * 0.5f);
}

float libstp::math::clampf(const float value, const float min, const float max)
{
    return std::min(std::max(value, min), max);
}

double libstp::math::clampDouble(const double value, const double min, const double max)
{
    return std::min(std::max(value, min), max);
}

int libstp::math::clampInt(const int value, const int min, const int max)
{
    return std::min(std::max(value, min), max);
}

int libstp::math::sign(const int value)
{
    return (value > 0) - (value < 0);
}

float libstp::math::signf(const float value)
{
    return static_cast<float>((value > 0) - (value < 0));
}

float libstp::math::minimalAngleDifference(const float a, const float b)
{
    auto angle1 = fmod(a, 2 * M_PI);
    if (angle1 < 0) angle1 += 2 * M_PI;

    auto angle2 = fmod(b, 2 * M_PI);
    if (angle2 < 0) angle2 += 2 * M_PI;

    const double diff = fabs(angle1 - angle2);
    return std::min(diff, 2 * M_PI - diff);
}

std::tuple<double, double, double> libstp::math::quaternionToEuler(Eigen::Quaterniond q)
{
    Eigen::Matrix3d rotationMatrix = q.toRotationMatrix();
    double roll, yaw;
    double pitch = std::asin(-rotationMatrix(2, 0)); // arcsin(-m31)

    if (std::abs(pitch - M_PI / 2) < 1e-6) {
        // Gimbal lock at +90 degrees
        roll = std::atan2(rotationMatrix(0, 1), rotationMatrix(0, 2));
        yaw = 0.0;
    } else if (std::abs(pitch + M_PI / 2) < 1e-6) {
        // Gimbal lock at -90 degrees
        roll = std::atan2(-rotationMatrix(0, 1), -rotationMatrix(0, 2));
        yaw = 0.0;
    } else {
        roll = std::atan2(rotationMatrix(2, 1), rotationMatrix(2, 2));  // atan2(m32, m33)
        yaw = std::atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));   // atan2(m21, m11)
    }

    return {roll, pitch, yaw};
}
