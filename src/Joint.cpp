#include "Joint.h"
#include <math.h>

Joint::Joint(double d, double a, double deg_theta, double deg_alpha) {
    this->theta = deg2rad(deg_theta);
    this->d = d;
    this->a = a;
    this->alpha = deg2rad(deg_alpha);
}

double Joint::deg2rad(double degrees) {
    return degrees * M_PI / 180.0;
}

Eigen::Matrix4d Joint::dh_matrix() const {
    Eigen::Matrix4d dh;
    dh << cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta),
          sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta),
          0, sin(alpha), cos(alpha), d,
          0, 0, 0, 1;
    return dh;
}
