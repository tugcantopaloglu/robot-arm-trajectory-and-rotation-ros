#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "ur10_kinematics/trajectory.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_node");
    std::vector<std::string> args;
    ros::removeROSArgs(argc, argv, args);
    if (args.size() != 7)
    {
        ROS_ERROR("Usage: rosrun ur10_kinematics ur10_trajectory_node x0 y0 z0 x1 y1 z1");
        return 1;
    }

    std::vector<std::vector<float>> commands;
    try
    {
        const Point3D start{parseCoordinate(args[1]), parseCoordinate(args[2]), parseCoordinate(args[3])};
        const Point3D end{parseCoordinate(args[4]), parseCoordinate(args[5]), parseCoordinate(args[6])};
        commands = planTrajectory(start, end);
    }
    catch (const std::exception &ex)
    {
        ROS_ERROR("Invalid trajectory: %s", ex.what());
        return 1;
    }

    ros::NodeHandle nh;
    ros::Publisher angle_pub = nh.advertise<std_msgs::Float32MultiArray>("/ur10_arm/acilar", 10);
    const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(5.0);
    ros::WallRate connection_rate(10);
    while (ros::ok() && angle_pub.getNumSubscribers() == 0 && ros::WallTime::now() < deadline)
    {
        ros::spinOnce();
        connection_rate.sleep();
    }
    if (!ros::ok() || angle_pub.getNumSubscribers() == 0)
    {
        ROS_ERROR("No arm command subscriber connected within five seconds.");
        return 1;
    }
    ros::Rate rate(1);
    for (const auto &joint_angles : commands)
    {
        if (!ros::ok())
            return 1;
        std_msgs::Float32MultiArray msg;
        msg.data = joint_angles;
        angle_pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Trajectory completed.");
    return 0;
}
