#include "ceres_replay_common.h"

#include <Eigen/LU>

#include <cmath>
#include <fstream>
#include <iterator>

namespace ORB_SLAM2 {
namespace BSRW {
namespace {

Eigen::Matrix3d Skew(const Eigen::Vector3d& vector) {
    Eigen::Matrix3d matrix;
    matrix << 0.0, -vector.z(), vector.y(),
              vector.z(), 0.0, -vector.x(),
              -vector.y(), vector.x(), 0.0;
    return matrix;
}

PoseSE3 LeftExp(const double* delta) {
    const Eigen::Vector3d omega(delta[0], delta[1], delta[2]);
    const Eigen::Vector3d upsilon(delta[3], delta[4], delta[5]);
    const double theta = omega.norm();
    const Eigen::Matrix3d omega_matrix = Skew(omega);
    Eigen::Matrix3d rotation;
    Eigen::Matrix3d V;
    if (theta < 1e-5) {
        rotation = Eigen::Matrix3d::Identity() + omega_matrix + omega_matrix * omega_matrix;
        V = rotation;
    } else {
        const Eigen::Matrix3d omega_squared = omega_matrix * omega_matrix;
        rotation = Eigen::Matrix3d::Identity()
            + std::sin(theta) / theta * omega_matrix
            + (1.0 - std::cos(theta)) / (theta * theta) * omega_squared;
        V = Eigen::Matrix3d::Identity()
            + (1.0 - std::cos(theta)) / (theta * theta) * omega_matrix
            + (theta - std::sin(theta)) / (theta * theta * theta) * omega_squared;
    }
    PoseSE3 result;
    result.quaternion = Eigen::Quaterniond(rotation).normalized();
    result.translation = V * upsilon;
    return result;
}

PoseSE3 Compose(const PoseSE3& left, const PoseSE3& right) {
    PoseSE3 result;
    result.quaternion = (left.quaternion * right.quaternion).normalized();
    result.translation = left.quaternion * right.translation + left.translation;
    return result;
}

PoseSE3 Inverse(const PoseSE3& pose) {
    PoseSE3 result;
    result.quaternion = pose.quaternion.conjugate();
    result.translation = result.quaternion * (-pose.translation);
    return result;
}

void LogSE3(const PoseSE3& pose, double* output) {
    const Eigen::Matrix3d rotation = pose.quaternion.toRotationMatrix();
    double cosine = 0.5 * (rotation.trace() - 1.0);
    cosine = std::max(-1.0, std::min(1.0, cosine));
    Eigen::Vector3d dR(rotation(2,1)-rotation(1,2),
                       rotation(0,2)-rotation(2,0),
                       rotation(1,0)-rotation(0,1));
    Eigen::Vector3d omega;
    Eigen::Matrix3d V_inverse;
    if (cosine > 0.99999) {
        omega = 0.5 * dR;
        const Eigen::Matrix3d omega_matrix = Skew(omega);
        V_inverse = Eigen::Matrix3d::Identity() - 0.5 * omega_matrix
            + (1.0 / 12.0) * omega_matrix * omega_matrix;
    } else {
        const double theta = std::acos(cosine);
        omega = theta / (2.0 * std::sqrt(1.0 - cosine * cosine)) * dR;
        const Eigen::Matrix3d omega_matrix = Skew(omega);
        V_inverse = Eigen::Matrix3d::Identity() - 0.5 * omega_matrix
            + (1.0 - theta / (2.0 * std::tan(theta / 2.0)))
                / (theta * theta) * omega_matrix * omega_matrix;
    }
    const Eigen::Vector3d upsilon = V_inverse * pose.translation;
    for (int i = 0; i < 3; ++i) output[i] = omega[i];
    for (int i = 0; i < 3; ++i) output[i + 3] = upsilon[i];
}

}  // namespace

bool LeftSE3Manifold::Plus(const double* x, const double* delta,
                           double* x_plus_delta) const {
    const PoseSE3 updated = Compose(LeftExp(delta), ArrayToPose(x));
    PoseToArray(updated, x_plus_delta);
    return true;
}

bool LeftSE3Manifold::PlusJacobian(const double* x, double* jacobian) const {
    const double epsilon = 1e-8;
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor> > output(jacobian);
    for (int column = 0; column < 6; ++column) {
        double plus_delta[6] = {0,0,0,0,0,0};
        double minus_delta[6] = {0,0,0,0,0,0};
        plus_delta[column] = epsilon;
        minus_delta[column] = -epsilon;
        double plus[7], minus[7];
        Plus(x, plus_delta, plus);
        Plus(x, minus_delta, minus);
        for (int row = 0; row < 7; ++row)
            output(row, column) = (plus[row] - minus[row]) / (2.0 * epsilon);
    }
    return true;
}

bool LeftSE3Manifold::Minus(const double* y, const double* x,
                            double* y_minus_x) const {
    LogSE3(Compose(ArrayToPose(y), Inverse(ArrayToPose(x))), y_minus_x);
    return true;
}

bool LeftSE3Manifold::MinusJacobian(const double* x, double* jacobian) const {
    double plus_data[42];
    PlusJacobian(x, plus_data);
    const Eigen::Map<const Eigen::Matrix<double, 7, 6, Eigen::RowMajor> > plus(plus_data);
    const Eigen::Matrix<double, 6, 7> pseudo_inverse =
        (plus.transpose() * plus).inverse() * plus.transpose();
    Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor> > output(jacobian);
    output = pseudo_inverse;
    return true;
}

ceres::CostFunction* CreateProjectionCost(const Snapshot& snapshot,
                                          const Observation& observation,
                                          EvaluationCounters* counters) {
    if (observation.dimension == 2)
        return new ceres::AutoDiffCostFunction<ProjectionCost<2>, 2, 7>(
            new ProjectionCost<2>(snapshot, observation, counters));
    return new ceres::AutoDiffCostFunction<ProjectionCost<3>, 3, 7>(
        new ProjectionCost<3>(snapshot, observation, counters));
}

void PoseToArray(const PoseSE3& pose, double* values) {
    values[0] = pose.translation.x();
    values[1] = pose.translation.y();
    values[2] = pose.translation.z();
    values[3] = pose.quaternion.x();
    values[4] = pose.quaternion.y();
    values[5] = pose.quaternion.z();
    values[6] = pose.quaternion.w();
}

PoseSE3 ArrayToPose(const double* values) {
    PoseSE3 pose;
    pose.translation = Eigen::Vector3d(values[0], values[1], values[2]);
    pose.quaternion = Eigen::Quaterniond(values[6], values[3], values[4], values[5]);
    pose.quaternion.normalize();
    return pose;
}

std::string FileSha256(const std::string& path) {
    std::ifstream input(path.c_str(), std::ios::binary);
    const std::string bytes((std::istreambuf_iterator<char>(input)),
                            std::istreambuf_iterator<char>());
    return Sha256Hex(bytes);
}

}  // namespace BSRW
}  // namespace ORB_SLAM2
