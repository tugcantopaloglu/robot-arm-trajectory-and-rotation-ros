#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Eigen/Dense>
#include <vector>
#include <cmath>

struct DHParameters
{
    double a;    
    double alpha; 
    double d;     
    double theta; 
};

class UR10Kinematics
{
public:
    UR10Kinematics();
    ~UR10Kinematics();

    Eigen::Matrix4d forwardKinematics(const std::vector<double> &joint_angles);

    std::vector<std::vector<double>> inverseKinematics(const Eigen::Matrix4d &end_effector_pose);

    Eigen::Quaterniond rotationMatrixToQuaternion(const Eigen::Matrix3d &rotation);

    Eigen::Vector3d quaternionToRPY(const Eigen::Quaterniond &q);

private:
    std::vector<DHParameters> dh_params_;

    Eigen::Matrix4d calculateTransformDH(const DHParameters &dh);

    double atan2_safe(double y, double x);
    double acos_safe(double x);
};

#endif
