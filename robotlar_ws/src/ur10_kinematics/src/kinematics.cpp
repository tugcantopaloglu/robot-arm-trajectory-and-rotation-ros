#include "ur10_kinematics/kinematics.h"

UR10Kinematics::UR10Kinematics()
{
    dh_params_.resize(6);

    dh_params_[0].a = 0.0;
    dh_params_[0].alpha = M_PI / 2;
    dh_params_[0].d = 0.1157;
    dh_params_[0].theta = 0.0;

    dh_params_[1].a = 0.5723;
    dh_params_[1].alpha = 0.0;
    dh_params_[1].d = 0.0;
    dh_params_[1].theta = 0.0;

    dh_params_[2].a = 0.5639;
    dh_params_[2].alpha = 0.0;
    dh_params_[2].d = 0.0;
    dh_params_[2].theta = 0.0;

    dh_params_[3].a = 0.0;
    dh_params_[3].alpha = M_PI / 2;
    dh_params_[3].d = 0.1719;
    dh_params_[3].theta = 0.0;

    dh_params_[4].a = 0.0;
    dh_params_[4].alpha = -M_PI / 2;
    dh_params_[4].d = 0.1273;
    dh_params_[4].theta = 0.0;

    dh_params_[5].a = 0.0;
    dh_params_[5].alpha = 0.0;
    dh_params_[5].d = 0.2209;
    dh_params_[5].theta = 0.0;
}

UR10Kinematics::~UR10Kinematics() {}

Eigen::Matrix4d UR10Kinematics::calculateTransformDH(const DHParameters &dh)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    double ct = cos(dh.theta);
    double st = sin(dh.theta);
    double ca = cos(dh.alpha);
    double sa = sin(dh.alpha);

    T(0, 0) = ct;
    T(0, 1) = -st * ca;
    T(0, 2) = st * sa;
    T(0, 3) = dh.a * ct;

    T(1, 0) = st;
    T(1, 1) = ct * ca;
    T(1, 2) = -ct * sa;
    T(1, 3) = dh.a * st;

    T(2, 0) = 0.0;
    T(2, 1) = sa;
    T(2, 2) = ca;
    T(2, 3) = dh.d;

    T(3, 0) = 0.0;
    T(3, 1) = 0.0;
    T(3, 2) = 0.0;
    T(3, 3) = 1.0;

    return T;
}

Eigen::Matrix4d UR10Kinematics::forwardKinematics(const std::vector<double> &joint_angles)
{
    if (joint_angles.size() != 6)
    {
        throw std::invalid_argument("UR10 robotu 6 açı gerektirir");
    }

    for (int i = 0; i < 6; ++i)
    {
        dh_params_[i].theta = joint_angles[i];
    }

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    for (int i = 0; i < 6; ++i)
    {
        T = T * calculateTransformDH(dh_params_[i]);
    }

    return T;
}

double UR10Kinematics::atan2_safe(double y, double x)
{
    if (std::abs(x) < 1e-10 && std::abs(y) < 1e-10)
    {
        return 0.0;
    }
    return std::atan2(y, x);
}

double UR10Kinematics::acos_safe(double x)
{
    if (x > 1.0)
    {
        return 0.0;
    }
    else if (x < -1.0)
    {
        return M_PI;
    }
    return std::acos(x);
}

std::vector<std::vector<double>> UR10Kinematics::inverseKinematics(const Eigen::Matrix4d &end_effector_pose)
{
    std::vector<std::vector<double>> solutions;

    Eigen::Vector3d pos = end_effector_pose.block<3, 1>(0, 3);
    Eigen::Matrix3d rot = end_effector_pose.block<3, 3>(0, 0);

    double d1 = dh_params_[0].d;
    double a2 = dh_params_[1].a;
    double a3 = dh_params_[2].a;
    double d4 = dh_params_[3].d;
    double d5 = dh_params_[4].d;
    double d6 = dh_params_[5].d;

    Eigen::Vector3d wrist_center = pos - d6 * rot.col(2);

    double theta1[2];
    theta1[0] = atan2_safe(wrist_center(1), wrist_center(0));
    theta1[1] = atan2_safe(wrist_center(1), wrist_center(0)) + M_PI;

    for (int i = 0; i < 2; ++i)
    {
        double c1 = cos(theta1[i]);
        double s1 = sin(theta1[i]);

        double r = sqrt(wrist_center(0) * wrist_center(0) + wrist_center(1) * wrist_center(1));

        double x = wrist_center(0) * c1 + wrist_center(1) * s1;
        double y = wrist_center(2) - d1;

        double L = sqrt(x * x + y * y);

        double cos_theta3 = (L * L - a2 * a2 - a3 * a3) / (2 * a2 * a3);

        if (cos_theta3 >= -1.0 && cos_theta3 <= 1.0)
        {
            double theta3[2];
            theta3[0] = acos_safe(cos_theta3);
            theta3[1] = -acos_safe(cos_theta3);

            for (int j = 0; j < 2; ++j)
            {
                double s3 = sin(theta3[j]);
                double c3 = cos(theta3[j]);

                double theta2 = atan2_safe(y, x) - atan2_safe(a3 * s3, a2 + a3 * c3);

                Eigen::Matrix4d T01 = Eigen::Matrix4d::Identity();
                Eigen::Matrix4d T12 = Eigen::Matrix4d::Identity();
                Eigen::Matrix4d T23 = Eigen::Matrix4d::Identity();

                DHParameters dh1 = dh_params_[0];
                dh1.theta = theta1[i];
                DHParameters dh2 = dh_params_[1];
                dh2.theta = theta2;
                DHParameters dh3 = dh_params_[2];
                dh3.theta = theta3[j];

                T01 = calculateTransformDH(dh1);
                T12 = calculateTransformDH(dh2);
                T23 = calculateTransformDH(dh3);

                Eigen::Matrix4d T03 = T01 * T12 * T23;
                Eigen::Matrix3d R03 = T03.block<3, 3>(0, 0);

                Eigen::Matrix3d R36 = R03.transpose() * rot;

                double theta5 = acos_safe(R36(2, 2));

                double theta4, theta6;
                if (abs(sin(theta5)) < 1e-5)
                {
                    theta4 = 0;
                    theta6 = atan2_safe(R36(1, 0), R36(0, 0));
                }
                else
                {
                    theta4 = atan2_safe(R36(1, 2) / sin(theta5), R36(0, 2) / sin(theta5));
                    theta6 = atan2_safe(R36(2, 1) / sin(theta5), -R36(2, 0) / sin(theta5));
                }

                std::vector<double> solution = {theta1[i], theta2, theta3[j], theta4, theta5, theta6};
                solutions.push_back(solution);
            }
        }
    }

    return solutions;
}

Eigen::Quaterniond UR10Kinematics::rotationMatrixToQuaternion(const Eigen::Matrix3d &rotation)
{
    return Eigen::Quaterniond(rotation);
}

Eigen::Vector3d UR10Kinematics::quaternionToRPY(const Eigen::Quaterniond &q)
{
    Eigen::Vector3d rpy;

    double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    rpy(0) = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        rpy(1) = std::copysign(M_PI / 2, sinp);
    else
        rpy(1) = std::asin(sinp);

    double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    rpy(2) = std::atan2(siny_cosp, cosy_cosp);

    return rpy;
}