#include <iostream>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <cmath>
#include "InverseKinematics.h"

// Função para converter radianos para graus
double rad2deg(double radians) {
    return radians * 180.0 / M_PI;
}

// Função para limitar ângulos entre 0 e 360 graus
double limit_angle(double angle) {
    while (angle < 0) angle += 360;
    while (angle >= 360) angle -= 360;
    return angle;
}

InverseKinematics::InverseKinematics(ForwardKinematicsDH& fk, int max_iterations, double tolerance)
    : fk_(fk), max_iterations_(max_iterations), tolerance_(tolerance) {}

void InverseKinematics::error_counter() const {
    std::cout << "Number of iterations: " << error_count_ << std::endl;
}

Eigen::VectorXd InverseKinematics::compute_ik(const Eigen::Vector3d& desired_position) {
    Eigen::VectorXd current_joint_angles(fk_.joints_.size());
    for (int i = 0; i < current_joint_angles.size(); ++i) {
        current_joint_angles[i] = fk_.joints_[i].theta;
    }

    error_count_ = 0;  // Reset error count for each call

    for (int i = 0; i < max_iterations_; ++i) {
        // Atualizar os ângulos das juntas no fk
        for (int j = 0; j < fk_.joints_.size(); ++j) {
            fk_.joints_[j].theta = current_joint_angles[j];
        }

        Eigen::Matrix4d current_end_effector = fk_.compute_end_effector();
        Eigen::Vector3d current_position = current_end_effector.block<3, 1>(0, 3);

        Eigen::Vector3d error = desired_position - current_position;

        if (error.norm() < tolerance_) {
            // Converter os ângulos das juntas para graus antes de retornar
            Eigen::VectorXd joint_angles_deg(current_joint_angles.size());
            for (int i = 0; i < current_joint_angles.size(); ++i) {
                joint_angles_deg[i] = limit_angle(rad2deg(current_joint_angles[i]));
            }
            return joint_angles_deg;  // Solução encontrada em graus
        }

        error_count_++;  // Incrementar contagem de erros

        Eigen::MatrixXd J = fk_.compute_jacobian();
        // Usar a decomposição QR do Eigen para cálculo eficiente da pseudo-inversa
        Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(J.block<3, Eigen::Dynamic>(0, 0, 3, fk_.joints_.size()));
        Eigen::VectorXd delta_theta = qr.solve(error);

        current_joint_angles += delta_theta;

        // Aplicar limites aos ângulos das juntas (opcional):
        for (int i = 0; i < current_joint_angles.size(); ++i) {
            current_joint_angles[i] = fmod(current_joint_angles[i], 2 * M_PI);
        }
    }

    throw std::runtime_error("Inverse kinematics failed to converge.");
}
