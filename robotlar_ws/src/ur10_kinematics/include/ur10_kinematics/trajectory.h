#ifndef UR10_KINEMATICS_TRAJECTORY_H
#define UR10_KINEMATICS_TRAJECTORY_H

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

struct Point3D
{
    double x, y, z;
};

inline double parseCoordinate(const std::string &text)
{
    std::size_t consumed = 0;
    const double value = std::stod(text, &consumed);
    if (consumed != text.size() || !std::isfinite(value))
        throw std::invalid_argument("Coordinates must be finite numbers without trailing characters.");
    return value;
}

inline std::vector<Point3D> generateWaypoints(const Point3D &start, const Point3D &end)
{
    const double total_dist = std::hypot(std::hypot(end.x - start.x, end.y - start.y), end.z - start.z);
    if (!std::isfinite(total_dist))
        throw std::invalid_argument("Trajectory endpoints must be finite.");
    if (total_dist == 0.0)
        return {start};
    const double step_count = std::ceil(total_dist / 0.1);
    if (step_count > 10000)
        throw std::length_error("Trajectory exceeds the waypoint limit.");
    const int steps = static_cast<int>(step_count);
    std::vector<Point3D> waypoints;
    for (int i = 0; i <= steps; ++i)
    {
        const double ratio = static_cast<double>(i) / steps;
        waypoints.push_back({start.x + ratio * (end.x - start.x),
                             start.y + ratio * (end.y - start.y),
                             start.z + ratio * (end.z - start.z)});
    }
    return waypoints;
}

inline std::vector<float> inverseKinematics(const Point3D &target)
{
    if (!std::isfinite(target.x) || !std::isfinite(target.y) || !std::isfinite(target.z))
        return {};
    const double d1 = 0.1273;
    const double a2 = 0.612;
    const double a3 = 0.5723;
    const double d6 = 0.0922;
    const double r = std::hypot(target.x, target.y);
    const double s = target.z - d6 - d1;
    const double D = (r * r + s * s - a2 * a2 - a3 * a3) / (2 * a2 * a3);
    if (!std::isfinite(D) || std::abs(D) > 1.0)
        return {};
    const double theta1 = std::atan2(target.y, target.x);
    const double theta3 = std::atan2(-std::sqrt(1 - D * D), D);
    const double theta2 = std::atan2(s, r) - std::atan2(a3 * std::sin(theta3), a2 + a3 * std::cos(theta3));
    return {static_cast<float>(theta1), static_cast<float>(theta2), static_cast<float>(theta3), 0.0f, 0.0f, 0.0f};
}

inline std::vector<std::vector<float>> planTrajectory(const Point3D &start, const Point3D &end)
{
    std::vector<std::vector<float>> commands;
    for (const auto &waypoint : generateWaypoints(start, end))
    {
        auto angles = inverseKinematics(waypoint);
        if (angles.empty())
            throw std::invalid_argument("Trajectory contains an unreachable waypoint.");
        commands.push_back(angles);
    }
    return commands;
}

#endif
