#include "ceres_replay_common.h"

#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

namespace {

class WorkCallback final : public ceres::IterationCallback {
 public:
    ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) override {
        if (summary.iteration > 0) {
            if (summary.step_is_successful) ++accepted;
            else ++rejected;
            linear_solver_iterations += summary.linear_solver_iterations;
        }
        return ceres::SOLVER_CONTINUE;
    }
    int accepted = 0;
    int rejected = 0;
    int linear_solver_iterations = 0;
};

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3 || argc > 4) {
        std::cerr << "usage: ceres_snapshot_replay PILOT_ID SNAPSHOT [BUDGET]\n";
        return 2;
    }
    const std::string pilot_id = argv[1];
    const std::string snapshot_path = argv[2];
    ORB_SLAM2::BSRW::Snapshot snapshot;
    std::string error;
    if (!ORB_SLAM2::BSRW::ReadSnapshot(snapshot_path, &snapshot, &error)) {
        std::cerr << error << '\n';
        return 3;
    }
    const int budget = argc == 4 ? std::atoi(argv[3]) : snapshot.budget;
    if (budget <= 0) return 4;

    double pose[7];
    ORB_SLAM2::BSRW::PoseToArray(snapshot.initial_pose, pose);
    ORB_SLAM2::BSRW::EvaluationCounters counters;
    ceres::Problem problem;
    int active_observations = 0;
    for (const auto& observation : snapshot.observations) {
        if (!observation.active) continue;
        ++active_observations;
        ceres::LossFunction* loss = observation.use_huber
            ? static_cast<ceres::LossFunction*>(new ceres::HuberLoss(observation.huber_delta))
            : nullptr;
        problem.AddResidualBlock(
            ORB_SLAM2::BSRW::CreateProjectionCost(snapshot, observation, &counters),
            loss, pose);
    }
    problem.SetManifold(pose, new ORB_SLAM2::BSRW::LeftSE3Manifold);

    ceres::Problem::EvaluateOptions evaluate_options;
    double initial_native_half_cost = 0.0;
    if (!problem.Evaluate(evaluate_options, &initial_native_half_cost, nullptr, nullptr, nullptr))
        return 5;
    const double initial_native = 2.0 * initial_native_half_cost;
    const double initial_canonical = ORB_SLAM2::BSRW::CanonicalObjective(
        snapshot, snapshot.initial_pose, true);
    const double objective_tolerance = 1e-10 * std::max(1.0, std::abs(initial_canonical));
    if (std::abs(initial_native - initial_canonical) > objective_tolerance) {
        std::cerr << "initial objective mismatch: native=" << std::setprecision(17)
                  << initial_native << " canonical=" << initial_canonical << '\n';
        return 6;
    }
    counters.residual_evaluations.store(0);
    counters.jacobian_evaluations.store(0);

    WorkCallback callback;
    ceres::Solver::Options options;
    options.minimizer_type = ceres::TRUST_REGION;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = budget;
    options.num_threads = 1;
    options.update_state_every_iteration = true;
    options.logging_type = ceres::SILENT;
    options.callbacks.push_back(&callback);
    ceres::Solver::Summary summary;
    const auto start = std::chrono::steady_clock::now();
    ceres::Solve(options, &problem, &summary);
    const double solver_ms = std::chrono::duration_cast<
        std::chrono::duration<double, std::milli> >(
            std::chrono::steady_clock::now() - start).count();
    const ORB_SLAM2::BSRW::PoseSE3 final_pose = ORB_SLAM2::BSRW::ArrayToPose(pose);
    const double final_canonical = ORB_SLAM2::BSRW::CanonicalObjective(
        snapshot, final_pose, true);
    const double final_native = 2.0 * summary.final_cost;
    if (std::abs(final_native - final_canonical) >
        1e-8 * std::max(1.0, std::abs(final_canonical))) {
        std::cerr << "final objective mismatch\n";
        return 7;
    }

    std::cout << std::setprecision(17)
        << pilot_id << ",ceres," << ORB_SLAM2::BSRW::FileSha256(snapshot_path)
        << ',' << snapshot.input_sha256 << ',' << budget
        << ',' << callback.accepted + callback.rejected
        << ',' << callback.accepted << ',' << callback.rejected
        << ',' << counters.residual_evaluations.load()
        << ',' << counters.jacobian_evaluations.load()
        << ',' << callback.linear_solver_iterations
        << ',' << initial_canonical << ',' << final_canonical
        << ',' << ceres::TerminationTypeToString(summary.termination_type)
        << ',' << solver_ms
        << ',' << (summary.iterations.empty() ? 0 : summary.iterations.size() - 1)
        << ',' << active_observations << '\n';
    return 0;
}
