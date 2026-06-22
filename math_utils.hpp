#ifndef MATH_UTILS_HPP
#define MATH_UTILS_HPP

#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace std;

#define CLOSE_TO_ZERO 0.000001
#define MIN_V3_MAG 0.01
#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)
#define GRAV 9.81
#define TWO_PI 6.28318530718f

const Eigen::Vector3d NORM_GRAV_MAG(0.0f, 0.0f, 1.0f);
const Eigen::Vector3d GRAV_VEC(0.0f, 0.0f, 9.81f);

void eulers_to_quat(const Eigen::Vector3d& eulers, Eigen::Quaterniond& quat);

#endif