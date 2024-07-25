#ifndef FORWARDKINEMATICSDH_H
#define FORWARDKINEMATICSDH_H

#include "Joint.h"
#include <vector>

class ForwardKinematicsDH {
public:
    std::vector<Joint> joints_;
    ForwardKinematicsDH(const std::vector<Joint>& joints);

    Eigen::Matrix4d compute_end_effector() const;
    Eigen::MatrixXd compute_jacobian() const;
};

#endif // FORWARDKINEMATICSDH_H
