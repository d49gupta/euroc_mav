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
    quat.normalize();
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

void SLERP_v3(Eigen::Vector3d& a, const Eigen::Vector3d& b, double gain)
{
    if (gain > 1.0 || gain < 0.0)
        return;

    // 1. We must operate on normalized vectors for geometric angular interpolation
    Eigen::Vector3d a_norm = a.normalized();
    Eigen::Vector3d b_norm = b.normalized();

    // 2. Compute the dot product between the directional vectors
    double dp = a_norm.dot(b_norm);

    // 3. Clamp to prevent floating-point rounding errors from causing acos() to return NaN
    dp = std::fmin(std::fmax(dp, -1.0), 1.0);
    
    double ang = std::acos(dp);
    double sin3_ang = std::sin(ang);

    // 4. Check for division-by-zero singularity (when vectors are virtually parallel)
    if (std::abs(sin3_ang) > 0.0001)
    {
        double sin1_ang = std::sin((1.0 - gain) * ang);
        double sin2_ang = std::sin(gain * ang);

        // Compute the interpolated directional vector safely into a temporary object
        Eigen::Vector3d res_dir = (a_norm * sin1_ang + b_norm * sin2_ang) / sin3_ang;

        // 5. Scale the directional vector back to match the interpolated magnitude
        double mag = (1.0 - gain) * a.norm() + gain * b.norm();
        
        a = res_dir * mag;
    }
    else
    {
        // Fallback: If the angle is near 0, standard LERP matches SLERP perfectly
        a = (1.0 - gain) * a + gain * b;
    }
}
Eigen::Vector3d quat_to_eulers(const Eigen::Quaterniond q_curr)
{
    double roll  = std::atan2(2.0 * (q_curr.w() * q_curr.x() + q_curr.y() * q_curr.z()), 1.0 - 2.0 * (q_curr.x() * q_curr.x() + q_curr.y() * q_curr.y()));
    double pitch = std::asin(std::fmin(std::fmax(2.0 * (q_curr.w() * q_curr.y() - q_curr.z() * q_curr.x()), -1.0), 1.0));
    double yaw   = std::atan2(2.0 * (q_curr.w() * q_curr.z() + q_curr.x() * q_curr.y()), 1.0 - 2.0 * (q_curr.y() * q_curr.y() + q_curr.z() * q_curr.z()));
    Eigen::Vector3d euler(roll, pitch, yaw);
    return euler;
}

Eigen::Matrix3d VectoSkew(Eigen::Vector3d& vec)
{
    Eigen::Matrix3d skew;
    skew.setZero();

    skew(0, 1) = -vec.z();
    skew(0, 2) = vec.y();
    skew(1, 0) = vec.z();
    skew(1, 2) = -vec.x();
    skew(2, 0) = -vec.y();
    skew(2, 1) = vec.x();
    
    return skew;
}

Eigen::Matrix3d vectoR(Eigen::Vector3d& rvec)
{
    double angle = rvec.norm();

    Eigen::Matrix3d mat;
    if (angle < 1e-12) {
        mat = Eigen::Matrix3d::Identity() + VectoSkew(rvec); // approximation
    } 
    else {
        mat = Eigen::AngleAxisd(angle, rvec / angle).toRotationMatrix();
    }
    return mat;
}