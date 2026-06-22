#include "math_utils.hpp"

void eulers_to_quat(const Eigen::Vector3d& eulers, Eigen::Quaterniond& quat)
{
    const double phi2   = eulers.x() / 2.0; // Roll
    const double theta2 = eulers.y() / 2.0; // Pitch
    const double psi2   = eulers.z() / 2.0; // Yaw

    const double sinphi2   = std::sin(phi2);
    const double cosphi2   = std::cos(phi2);
    const double sintheta2 = std::sin(theta2);
    const double costheta2 = std::cos(theta2);
    const double sinpsi2   = std::sin(psi2);
    const double cospsi2   = std::cos(psi2);

    // Standard ZYX (Roll-Pitch-Yaw) intrinsic rotation sequence
    // Equation for euler->Quat for ZYX is different from XYZ, ZYX means you're doing q_yaw x q_pitch x q_roll
    // If you pass ZYX into ZYX quat equation and XYZ into XYZ quat equation you get the same quaternion (universal)
    quat.w() = cosphi2 * costheta2 * cospsi2 + sinphi2 * sintheta2 * sinpsi2;
    quat.x() = sinphi2 * costheta2 * cospsi2 - cosphi2 * sintheta2 * sinpsi2;
    quat.y() = cosphi2 * sintheta2 * cospsi2 + sinphi2 * costheta2 * sinpsi2;
    quat.z() = cosphi2 * costheta2 * sinpsi2 - sinphi2 * sintheta2 * cospsi2;
}

void SLERP_v4(Eigen::Quaterniond& a, const Eigen::Quaterniond& b, float gain)
{
    if (gain > 1.0f || gain < 0.0f)
        return;

    Eigen::Quaterniond b_target = b;
    float dp = a.dot(b_target); 

    if (dp < 0.0f) 
    {
        b_target.coeffs() *= -1.0f;
        dp = -dp;
    }

    // 3. Clamp to prevent floating-point precision issues with acos
    dp = fminf(fmaxf(dp, -1.0f), 1.0f);
    float theta = acosf(dp);
    float sin3_theta = sinf(theta);

    if (fabsf(sin3_theta) > CLOSE_TO_ZERO)
    {
        float sin1_theta = sinf((1.0f - gain) * theta);
        float sin2_theta = sinf(gain * theta);

        Eigen::Quaterniond result;
        result.w() = (a.w() * sin1_theta + b_target.w() * sin2_theta) / sin3_theta;
        result.x() = (a.x() * sin1_theta + b_target.x() * sin2_theta) / sin3_theta;
        result.y() = (a.y() * sin1_theta + b_target.y() * sin2_theta) / sin3_theta;
        result.z() = (a.z() * sin1_theta + b_target.z() * sin2_theta) / sin3_theta;
        a = result;
    }

    a.normalize();
}

double yaw_from_quat(const Eigen::Quaterniond& q)
{
    double w = q.w();
    double x = q.x();
    double y = q.y();
    double z = q.z();

    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    return yaw;
}