#include "ForwardKinematicsDH.h"

ForwardKinematicsDH::ForwardKinematicsDH(const std::vector<Joint>& joints) : joints_(joints) {}

Eigen::Matrix4d ForwardKinematicsDH::compute_end_effector() const {
    Eigen::Matrix4d end_effector_matrix = Eigen::Matrix4d::Identity();
    for (const Joint& joint : joints_) {
        Eigen::Matrix4d transformation = joint.dh_matrix();
        end_effector_matrix *= transformation;
    }
    return end_effector_matrix;  
}

Eigen::MatrixXd ForwardKinematicsDH::compute_jacobian() const {
    int n = joints_.size();
    Eigen::MatrixXd J(6, n);
    J.setZero();  

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    std::vector<Eigen::Vector3d> positions;
    std::vector<Eigen::Vector3d> z_axes;

    // Posição inicial e eixo Z da base
    positions.push_back(Eigen::Vector3d::Zero());
    z_axes.push_back(Eigen::Vector3d::UnitZ());  

    for (const Joint& joint : joints_) {
        T = T * joint.dh_matrix();
        positions.push_back(T.block<3, 1>(0, 3));
        z_axes.push_back(T.block<3, 1>(0, 2));
    }

    const Eigen::Vector3d& end_effector_position = positions.back();

    for (int i = 0; i < n; ++i) {
        J.block<3, 1>(0, i) = z_axes[i].cross(end_effector_position - positions[i]);
        J.block<3, 1>(3, i) = z_axes[i];
    }

    return J;
}
