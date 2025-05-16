#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "ur10_kinematics/kinematics.h"
class UR10KinematicsNode
{
public:
    UR10KinematicsNode()
    {
        joint_angles_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/ur10_arm/acilar", 1);
        odometry_sub_ = nh_.subscribe("/ur10_arm/odometri", 1, &UR10KinematicsNode::odometryCallback, this);
        ros::Duration(1.0).sleep();
        kinematics_ = new UR10Kinematics();
    }

    ~UR10KinematicsNode()
    {
        if (kinematics_)
        {
            delete kinematics_;
        }
    }

    void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        actual_position_ = Eigen::Vector3d(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z);

        actual_orientation_ = Eigen::Quaterniond(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z);

        ROS_INFO("Gercek pozisyon: [%.4f, %.4f, %.4f]",
                 actual_position_(0), actual_position_(1), actual_position_(2));

        Eigen::Vector3d rpy = kinematics_->quaternionToRPY(actual_orientation_);
        ROS_INFO("Gercek yonelim (RPY): [%.4f, %.4f, %.4f]", rpy(0), rpy(1), rpy(2));
    }

    void testForwardKinematics(const std::vector<double> &joint_angles)
    {
        std_msgs::Float32MultiArray msg;
        msg.data.resize(joint_angles.size());

        for (size_t i = 0; i < joint_angles.size(); ++i)
        {
            msg.data[i] = static_cast<float>(joint_angles[i]);
        }

        joint_angles_pub_.publish(msg);
        ROS_INFO("Gonderilen eklem acilari: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                 joint_angles[0], joint_angles[1], joint_angles[2],
                 joint_angles[3], joint_angles[4], joint_angles[5]);

        Eigen::Matrix4d end_effector_pose = kinematics_->forwardKinematics(joint_angles);
        Eigen::Vector3d position = end_effector_pose.block<3, 1>(0, 3);
        Eigen::Matrix3d rotation = end_effector_pose.block<3, 3>(0, 0);
        Eigen::Quaterniond orientation = kinematics_->rotationMatrixToQuaternion(rotation);
        Eigen::Vector3d rpy = kinematics_->quaternionToRPY(orientation);

        ROS_INFO("Teorik pozisyon: [%.4f, %.4f, %.4f]",
                 position(0), position(1), position(2));
        ROS_INFO("Teorik yonelim (RPY): [%.4f, %.4f, %.4f]",
                 rpy(0), rpy(1), rpy(2));

        ros::Duration(2.0).sleep();
        ros::spinOnce();

        Eigen::Vector3d position_error = position - actual_position_;

        ROS_INFO("Pozisyon hatasi: [%.4f, %.4f, %.4f], Buyukluk: %.4f",
                 position_error(0), position_error(1), position_error(2),
                 position_error.norm());

        ros::Duration(1.0).sleep();
    }

    void testInverseKinematics(const Eigen::Vector3d &target_position)
    {
        Eigen::Matrix4d target_pose = Eigen::Matrix4d::Identity();
        target_pose.block<3, 1>(0, 3) = target_position;

        ros::spinOnce();
        Eigen::Matrix3d current_rot = actual_orientation_.toRotationMatrix();
        target_pose.block<3, 3>(0, 0) = current_rot;

        std::vector<std::vector<double>> solutions = kinematics_->inverseKinematics(target_pose);

        if (solutions.empty())
        {
            ROS_WARN("Hedef pozisyon icin IK cozum bulunamadi [%.4f, %.4f, %.4f]",
                     target_position(0), target_position(1), target_position(2));
            return;
        }

        std::vector<double> joint_angles = solutions[0];

        ROS_INFO("IK cozum eklem acilari: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                 joint_angles[0], joint_angles[1], joint_angles[2],
                 joint_angles[3], joint_angles[4], joint_angles[5]);

        testForwardKinematics(joint_angles);
    }

    void runTests()
    {
        std::vector<std::vector<double>> test_angles = {
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            {0.5, -0.2, 0.6, -0.6, -0.4, 0.5},
            {0.8, -0.5, 0.4, -0.3, -0.7, 0.2}};

        ROS_INFO("Ileri Kinematik Testi");
        for (const auto &angles : test_angles)
        {
            testForwardKinematics(angles);
            ros::Duration(1.0).sleep();
        }

        ROS_INFO("Ters Kinematik Testi");
        std::vector<Eigen::Vector3d> test_positions = {
            Eigen::Vector3d(1.0, 0.3, 1.5),
            Eigen::Vector3d(0.8, -0.2, 1.7),
            Eigen::Vector3d(1.2, 0.4, 1.6)};

        for (const auto &position : test_positions)
        {
            testInverseKinematics(position);
            ros::Duration(1.0).sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher joint_angles_pub_;
    ros::Subscriber odometry_sub_;

    UR10Kinematics *kinematics_;

    Eigen::Vector3d actual_position_;
    Eigen::Quaterniond actual_orientation_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur10_kinematics_node");

    ROS_INFO("UR10 Kinematik Dugumu Baslatiliyor");

    UR10KinematicsNode node;

    ros::Duration(1.0).sleep();

    node.runTests();

    ROS_INFO("UR10 Kinematik Dugumu Tamamlandi");

    return 0;
}