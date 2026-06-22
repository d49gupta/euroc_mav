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