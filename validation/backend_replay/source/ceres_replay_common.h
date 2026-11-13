#ifndef BSRW_CERES_REPLAY_COMMON_H
#define BSRW_CERES_REPLAY_COMMON_H

#include "BSRWSnapshot.h"

#include <ceres/ceres.h>
#include <Eigen/Cholesky>

#include <atomic>
#include <type_traits>

namespace ORB_SLAM2 {
namespace BSRW {

struct EvaluationCounters {
    std::atomic<long long> residual_evaluations{0};
    std::atomic<long long> jacobian_evaluations{0};
};

class LeftSE3Manifold final : public ceres::Manifold {
 public:
    int AmbientSize() const override { return 7; }
    int TangentSize() const override { return 6; }
    bool Plus(const double* x, const double* delta, double* x_plus_delta) const override;
    bool PlusJacobian(const double* x, double* jacobian) const override;
    bool Minus(const double* y, const double* x, double* y_minus_x) const override;
    bool MinusJacobian(const double* x, double* jacobian) const override;
};

template <int Dimension>
class ProjectionCost {
 public:
    ProjectionCost(const Snapshot& snapshot, const Observation& observation,
                   EvaluationCounters* counters)
        : fx_(snapshot.camera.fx), fy_(snapshot.camera.fy),
          cx_(snapshot.camera.cx), cy_(snapshot.camera.cy),
          bf_(snapshot.camera.bf), point_(observation.point_w),
          counters_(counters) {
        for (int i = 0; i < Dimension; ++i) measurement_[i] = observation.measurement[i];
        const Eigen::Matrix<double, Dimension, Dimension> information =
            observation.information.topLeftCorner<Dimension, Dimension>();
        sqrt_information_ = information.llt().matrixU();
    }

    template <typename T>
    bool operator()(const T* const pose, T* residuals) const {
        if (std::is_same<T, double>::value)
            ++counters_->residual_evaluations;
        else
            ++counters_->jacobian_evaluations;
        const Eigen::Matrix<T, 3, 1> translation(pose[0], pose[1], pose[2]);
        Eigen::Quaternion<T> quaternion(pose[6], pose[3], pose[4], pose[5]);
        quaternion.normalize();
        const Eigen::Matrix<T, 3, 1> point_c =
            quaternion * point_.template cast<T>() + translation;
        const T inverse_z = T(1.0) / point_c.z();
        const T u = T(fx_) * point_c.x() * inverse_z + T(cx_);
        const T v = T(fy_) * point_c.y() * inverse_z + T(cy_);
        Eigen::Matrix<T, Dimension, 1> raw;
        raw[0] = T(measurement_[0]) - u;
        raw[1] = T(measurement_[1]) - v;
        if (Dimension == 3)
            raw[2] = T(measurement_[2]) - (u - T(bf_) * inverse_z);
        const Eigen::Matrix<T, Dimension, 1> weighted =
            sqrt_information_.template cast<T>() * raw;
        for (int i = 0; i < Dimension; ++i) residuals[i] = weighted[i];
        return true;
    }

 private:
    double fx_, fy_, cx_, cy_, bf_;
    Eigen::Vector3d point_;
    double measurement_[Dimension];
    Eigen::Matrix<double, Dimension, Dimension> sqrt_information_;
    EvaluationCounters* counters_;
};

ceres::CostFunction* CreateProjectionCost(const Snapshot& snapshot,
                                          const Observation& observation,
                                          EvaluationCounters* counters);
void PoseToArray(const PoseSE3& pose, double* values);
PoseSE3 ArrayToPose(const double* values);
std::string FileSha256(const std::string& path);

}  // namespace BSRW
}  // namespace ORB_SLAM2

#endif

