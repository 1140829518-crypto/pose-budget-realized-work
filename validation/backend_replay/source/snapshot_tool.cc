#include "ceres_replay_common.h"

#include <ceres/ceres.h>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>

namespace {

ORB_SLAM2::BSRW::Snapshot Synthetic(bool stereo, int budget) {
    using namespace ORB_SLAM2::BSRW;
    Snapshot snapshot;
    snapshot.snapshot_id = stereo ? "F2_synthetic_stereo" : "F1_synthetic_mono";
    snapshot.source_run_id = "PHASE4_SYNTHETIC";
    snapshot.frame_id = stereo ? 2 : 1;
    snapshot.caller_id = "SYNTHETIC_GENERATOR";
    snapshot.round_id = 1;
    snapshot.budget = budget;
    snapshot.camera.fx = 525.0; snapshot.camera.fy = 525.0;
    snapshot.camera.cx = 319.5; snapshot.camera.cy = 239.5;
    snapshot.camera.bf = 40.0;
    snapshot.initial_pose.translation = Eigen::Vector3d(0.04, -0.03, 0.02);
    snapshot.initial_pose.quaternion =
        Eigen::Quaterniond(Eigen::AngleAxisd(0.025, Eigen::Vector3d(0.2, -0.5, 0.84).normalized()));
    snapshot.orb_slam2_head = "7a9200ba9a1ad3006a9fbad02163d31437135371";
    snapshot.g2o_version = "ORB-SLAM2 bundled g2o + realized-work counters";
    snapshot.ceres_version = "2.2.0";
    PoseSE3 truth;
    for (int i = 0; i < 16; ++i) {
        Observation observation;
        observation.observation_id = static_cast<std::uint64_t>(i);
        observation.map_point_id = static_cast<std::uint64_t>(1000 + i);
        observation.type = stereo ? "stereo_rgbd" : "mono";
        observation.dimension = stereo ? 3 : 2;
        observation.point_w = Eigen::Vector3d(
            -1.2 + 0.16 * i, -0.6 + 0.18 * (i % 7), 3.0 + 0.11 * (i % 5));
        const std::vector<double> exact = Project(snapshot, observation, truth);
        observation.measurement = exact;
        observation.measurement[0] += 0.04 * ((i % 3) - 1);
        observation.measurement[1] += 0.03 * ((i % 4) - 1.5);
        if (stereo) observation.measurement[2] -= 0.025 * ((i % 5) - 2);
        if (i == 15) observation.measurement[0] += 8.0;
        observation.base_information_scale = 1.0 + 0.25 * (i % 4);
        observation.temporal_reliability = 1.0;
        observation.belief_reliability = 1.0;
        observation.final_reliability = 1.0;
        observation.information.setZero();
        for (int d = 0; d < observation.dimension; ++d)
            observation.information(d,d) = observation.base_information_scale;
        observation.active = true;
        observation.use_huber = true;
        observation.huber_delta = std::sqrt(stereo ? 7.815 : 5.991);
        snapshot.observations.push_back(observation);
    }
    return snapshot;
}

bool Write(const ORB_SLAM2::BSRW::Snapshot& snapshot, const std::string& path) {
    std::string hash, error;
    if (!ORB_SLAM2::BSRW::WriteSnapshot(snapshot, path, &hash, &error)) {
        std::cerr << error << '\n';
        return false;
    }
    std::cout << path << ',' << hash << '\n';
    return true;
}

int CheckJacobian(const std::string& path) {
    using namespace ORB_SLAM2::BSRW;
    Snapshot snapshot;
    std::string error;
    if (!ReadSnapshot(path, &snapshot, &error)) {
        std::cerr << error << '\n';
        return 3;
    }
    double pose[7];
    PoseToArray(snapshot.initial_pose, pose);
    LeftSE3Manifold manifold;
    double plus_jacobian_data[42];
    manifold.PlusJacobian(pose, plus_jacobian_data);
    const Eigen::Map<const Eigen::Matrix<double,7,6,Eigen::RowMajor> >
        plus_jacobian(plus_jacobian_data);
    const double epsilon = 1e-6;
    double max_absolute = 0.0;
    double max_relative = 0.0;
    int checked = 0;
    for (const Observation& observation : snapshot.observations) {
        if (!observation.active) continue;
        EvaluationCounters counters;
        std::unique_ptr<ceres::CostFunction> cost(
            CreateProjectionCost(snapshot, observation, &counters));
        const double* parameters[] = {pose};
        std::vector<double> residual(observation.dimension);
        std::vector<double> ambient(observation.dimension * 7);
        double* jacobians[] = {ambient.data()};
        if (!cost->Evaluate(parameters, residual.data(), jacobians)) return 4;
        Eigen::Map<const Eigen::Matrix<double,Eigen::Dynamic,7,Eigen::RowMajor> >
            ambient_matrix(ambient.data(), observation.dimension, 7);
        const Eigen::Matrix<double,Eigen::Dynamic,6> analytic =
            ambient_matrix * plus_jacobian;
        Eigen::Matrix<double,Eigen::Dynamic,6> numeric(observation.dimension,6);
        for (int column = 0; column < 6; ++column) {
            double positive_delta[6] = {0,0,0,0,0,0};
            double negative_delta[6] = {0,0,0,0,0,0};
            positive_delta[column] = epsilon;
            negative_delta[column] = -epsilon;
            double positive_pose[7], negative_pose[7];
            manifold.Plus(pose, positive_delta, positive_pose);
            manifold.Plus(pose, negative_delta, negative_pose);
            const double* positive_parameters[] = {positive_pose};
            const double* negative_parameters[] = {negative_pose};
            std::vector<double> positive(observation.dimension), negative(observation.dimension);
            cost->Evaluate(positive_parameters, positive.data(), nullptr);
            cost->Evaluate(negative_parameters, negative.data(), nullptr);
            for (int row = 0; row < observation.dimension; ++row)
                numeric(row,column) = (positive[row] - negative[row]) / (2.0 * epsilon);
        }
        for (int row = 0; row < observation.dimension; ++row)
            for (int column = 0; column < 6; ++column) {
                const double absolute = std::abs(analytic(row,column)-numeric(row,column));
                const double scale = std::max(1.0,
                    std::max(std::abs(analytic(row,column)),std::abs(numeric(row,column))));
                max_absolute = std::max(max_absolute,absolute);
                max_relative = std::max(max_relative,absolute/scale);
            }
        ++checked;
    }
    const bool pass = max_relative < 1e-6;
    std::cout << std::setprecision(17) << path << ',' << checked << ','
              << max_absolute << ',' << max_relative << ',' << (pass?"PASS":"FAIL") << '\n';
    return pass ? 0 : 5;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) return 2;
    const std::string command = argv[1];
    if ((command == "generate-mono" || command == "generate-stereo") && argc == 4)
        return Write(Synthetic(command == "generate-stereo", std::atoi(argv[3])), argv[2]) ? 0 : 3;
    if (command == "rebudget" && argc == 6) {
        ORB_SLAM2::BSRW::Snapshot snapshot;
        std::string error;
        if (!ORB_SLAM2::BSRW::ReadSnapshot(argv[2], &snapshot, &error)) {
            std::cerr << error << '\n'; return 3;
        }
        snapshot.snapshot_id = argv[4];
        snapshot.budget = std::atoi(argv[5]);
        return Write(snapshot, argv[3]) ? 0 : 4;
    }
    if (command == "jacobian-check" && argc == 3) return CheckJacobian(argv[2]);
    if (command == "validate" && argc == 3) {
        ORB_SLAM2::BSRW::Snapshot snapshot;
        std::string error;
        if (!ORB_SLAM2::BSRW::ReadSnapshot(argv[2], &snapshot, &error)) {
            std::cerr << argv[2] << ',' << error << '\n'; return 3;
        }
        std::cout << argv[2] << ',' << snapshot.input_sha256 << ','
                  << snapshot.observations.size() << ",PASS\n";
        return 0;
    }
    std::cerr << "commands: generate-mono OUT BUDGET | generate-stereo OUT BUDGET | "
                 "rebudget IN OUT SNAPSHOT_ID BUDGET | jacobian-check SNAPSHOT | validate SNAPSHOT\n";
    return 2;
}
