#include "../include/ur10_kinematics/trajectory.h"
#include <cassert>
#include <limits>

template <typename Callable>
bool rejects(Callable operation)
{
    try { operation(); }
    catch (const std::exception &) { return true; }
    return false;
}

int main()
{
    assert(parseCoordinate("0.7") == 0.7);
    assert(rejects([] { parseCoordinate("0.7garbage"); }));
    assert(rejects([] { parseCoordinate("nan"); }));
    assert(rejects([] { parseCoordinate("inf"); }));
    assert(rejects([] { parseCoordinate("1e999"); }));
    const Point3D start{0.7, 0.7, 0.7};
    const auto stationary = generateWaypoints(start, start);
    assert(stationary.size() == 1 && stationary[0].x == start.x);
    const auto stationary_commands = planTrajectory(start, start);
    assert(stationary_commands.size() == 1 && stationary_commands[0].size() == 6);
    const Point3D end{0.1, 0.1, 0.1};
    const auto points = generateWaypoints(start, end);
    assert(points.front().x == start.x && std::abs(points.back().x - end.x) < 1e-12);
    for (std::size_t i = 1; i < points.size(); ++i)
    {
        const double interval = std::hypot(std::hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y), points[i].z - points[i - 1].z);
        assert(interval <= 0.1 + 1e-12);
    }
    for (const auto &angles : planTrajectory(start, end))
    {
        assert(angles.size() == 6);
        for (float angle : angles) assert(std::isfinite(angle));
    }
    assert(inverseKinematics({10.0, 0.0, 0.0}).empty());
    assert(inverseKinematics({std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}).empty());
    assert(rejects([&] { planTrajectory(start, {10.0, 0.0, 0.0}); }));
    assert(rejects([] { planTrajectory({0.1, 0.0, 0.2195}, {-0.1, 0.0, 0.2195}); }));
    assert(rejects([&] { generateWaypoints(start, {std::numeric_limits<double>::infinity(), 0.0, 0.0}); }));
    assert(rejects([&] { generateWaypoints(start, {2000.0, 0.0, 0.0}); }));
    return 0;
}
