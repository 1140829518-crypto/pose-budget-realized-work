#include "ceres_replay_common.h"

#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/types/se3quat.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>

int main() {
    ORB_SLAM2::BSRW::PoseSE3 pose;
    pose.translation = Eigen::Vector3d(0.2, -0.1, 0.05);
    pose.quaternion = Eigen::Quaterniond(
        Eigen::AngleAxisd(0.18, Eigen::Vector3d(0.3, -0.4, 0.5).normalized()));
    double pose_array[7];
    ORB_SLAM2::BSRW::PoseToArray(pose, pose_array);
    ORB_SLAM2::BSRW::LeftSE3Manifold manifold;
    const double tests[][6] = {
        {1e-7,-2e-7,3e-7,4e-7,-5e-7,6e-7},
        {0.01,-0.02,0.015,0.03,-0.01,0.02},
        {-0.12,0.04,0.08,-0.05,0.06,-0.03}};
    double max_translation = 0.0;
    double max_rotation = 0.0;
    for (const auto& delta_values : tests) {
        double updated_array[7];
        manifold.Plus(pose_array, delta_values, updated_array);
        const ORB_SLAM2::BSRW::PoseSE3 updated =
            ORB_SLAM2::BSRW::ArrayToPose(updated_array);
        Eigen::Matrix<double,6,1> delta;
        for (int i = 0; i < 6; ++i) delta[i] = delta_values[i];
        const g2o::SE3Quat base(pose.quaternion, pose.translation);
        const g2o::SE3Quat expected = g2o::SE3Quat::exp(delta) * base;
        max_translation = std::max(max_translation,
            (updated.translation - expected.translation()).norm());
        max_rotation = std::max(max_rotation,
            (updated.quaternion.toRotationMatrix() - expected.rotation().toRotationMatrix()).norm());
    }

    g2o::RobustKernelHuber kernel;
    const double delta = std::sqrt(5.991);
    kernel.setDelta(delta);
    double max_huber = 0.0;
    for (double squared_error : {0.1, 5.991, 20.0, 100.0}) {
        Eigen::Vector3d rho;
        kernel.robustify(squared_error, rho);
        max_huber = std::max(max_huber,
            std::abs(rho[0] - ORB_SLAM2::BSRW::HuberCost(squared_error, delta)));
    }
    // Bundled g2o stores delta^2 in float; the binary64 contract computes the
    // same Huber definition with a bounded rounding difference at the branch.
    const bool pass = max_translation < 1e-12 && max_rotation < 1e-12 && max_huber < 1e-6;
    std::cout << std::setprecision(17)
              << "se3_translation_max=" << max_translation
              << ",se3_rotation_max=" << max_rotation
              << ",huber_cost_max=" << max_huber
              << ",status=" << (pass?"PASS":"FAIL") << '\n';
    return pass ? 0 : 1;
}
