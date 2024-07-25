#ifndef JOINT_H
#define JOINT_H

#include <Eigen/Dense>

class Joint {
public:
    double d;
    double a;
    double theta;
    double alpha;

    // Conversão de graus para radianos
    static double deg2rad(double degrees);

    Joint(double d, double a, double deg_theta, double deg_alpha);

    Eigen::Matrix4d dh_matrix() const;
};

#endif // JOINT_H
