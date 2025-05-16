#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <cmath>

struct Point3D
{
    double x, y, z;
};

double distance(const Point3D &p1, const Point3D &p2)
{
    return std::sqrt(
        std::pow(p1.x - p2.x, 2) +
        std::pow(p1.y - p2.y, 2) +
        std::pow(p1.z - p2.z, 2));
}

std::vector<Point3D> generateWaypoints(const Point3D &start, const Point3D &end)
{
    std::vector<Point3D> waypoints;

    double total_dist = distance(start, end);
    int steps = std::ceil(total_dist / 0.1);

    for (int i = 0; i <= steps; ++i)
    {
        double ratio = static_cast<double>(i) / steps;
        Point3D point;
        point.x = start.x + ratio * (end.x - start.x);
        point.y = start.y + ratio * (end.y - start.y);
        point.z = start.z + ratio * (end.z - start.z);
        waypoints.push_back(point);
    }

    return waypoints;
}

std::vector<float> inverseKinematics(const Point3D &target)
{
    std::vector<float> joint_angles(6, 0.0);

    const double d1 = 0.1273;
    const double a2 = 0.612;
    const double a3 = 0.5723;
    const double d4 = 0.163941;
    const double d5 = 0.1157;
    const double d6 = 0.0922;

    double wx = target.x;
    double wy = target.y;
    double wz = target.z - d6;

    double theta1 = atan2(wy, wx);

    double r = sqrt(wx * wx + wy * wy);
    double s = wz - d1;

    double D = (r * r + s * s - a2 * a2 - a3 * a3) / (2 * a2 * a3);
    ROS_WARN("D: %f", D);
    if (fabs(D) > 1.0)
    {
        ROS_WARN("Pozisyon icin IK cozum bulunamadi [%.3f, %.3f, %.3f]", target.x, target.y, target.z);
        return {};
    }

    double theta3 = atan2(-sqrt(1 - D * D), D);

    double phi1 = atan2(s, r);
    double phi2 = atan2(a3 * sin(theta3), a2 + a3 * cos(theta3));
    double theta2 = phi1 - phi2;

    double theta4 = 0.0;
    double theta5 = 0.0;
    double theta6 = 0.0;

    joint_angles[0] = theta1;
    joint_angles[1] = theta2;
    joint_angles[2] = theta3;
    joint_angles[3] = theta4;
    joint_angles[4] = theta5;
    joint_angles[5] = theta6;

    return joint_angles;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_node");
    ros::NodeHandle nh;

    ros::Publisher angle_pub = nh.advertise<std_msgs::Float32MultiArray>("/ur10_arm/acilar", 10);

    if (argc < 7 || argc > 7)
    {
        ROS_ERROR("Kullanim: rosrun gazebo_plugins_rtg trajectory_node x0 y0 z0 x1 y1 z1");
        return 1;
    }

    Point3D start{std::stod(argv[1]), std::stod(argv[2]), std::stod(argv[3])};
    Point3D end{std::stod(argv[4]), std::stod(argv[5]), std::stod(argv[6])};

    auto waypoints = generateWaypoints(start, end);

    ros::Rate rate(1);

    for (const auto &waypoint : waypoints)
    {
        auto joint_angles = inverseKinematics(waypoint);

        std_msgs::Float32MultiArray msg;
        msg.data.insert(msg.data.end(), joint_angles.begin(), joint_angles.end());

        angle_pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Trajektori tamamlandi.");
    return 0;
}