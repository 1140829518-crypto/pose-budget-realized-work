#include "ceres_replay_common.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

class WorkCallback final : public ceres::IterationCallback {
 public:
    ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) override {
        if (summary.iteration > 0) {
            if (summary.step_is_successful) ++accepted;
            else ++rejected;
            linear += summary.linear_solver_iterations;
        }
        return ceres::SOLVER_CONTINUE;
    }
    int accepted = 0;
    int rejected = 0;
    int linear = 0;
};

template <typename T>
std::string Join(const std::vector<T>& values) {
    std::ostringstream out;
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i) out << '|';
        out << values[i];
    }
    return out.str();
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "usage: ceres_four_round_replay FIXTURE_ID ROUND1_SNAPSHOT BUDGET\n";
        return 2;
    }
    const std::string fixture_id = argv[1];
    const std::string path = argv[2];
    const int budget = std::atoi(argv[3]);
    if (budget <= 0) return 3;

    ORB_SLAM2::BSRW::Snapshot snapshot;
    std::string error;
    if (!ORB_SLAM2::BSRW::ReadSnapshot(path, &snapshot, &error)) {
        std::cerr << error << '\n';
        return 4;
    }
    if (snapshot.round_id != 1) {
        std::cerr << "four-round replay requires a round-1 snapshot\n";
        return 5;
    }
    if (snapshot.observations.size() < 3) return 6;

    ORB_SLAM2::BSRW::PoseSE3 pose = snapshot.initial_pose;
    std::vector<bool> active;
    active.reserve(snapshot.observations.size());
    for (const auto& observation : snapshot.observations)
        active.push_back(observation.active);

    long long total_attempted = 0, total_accepted = 0, total_rejected = 0;
    long long total_residual = 0, total_jacobian = 0, total_linear = 0;
    int logical_outer_work = 0;
    double total_ms = 0.0;
    std::vector<int> active_before, active_after, native_iterations;
    std::vector<long long> attempted, accepted, rejected, residual, jacobian, linear;
    std::vector<std::string> reasons;
    double first_initial_cost = 0.0;
    double last_final_cost = 0.0;

    for (int round = 0; round < 4; ++round) {
        double pose_values[7];
        ORB_SLAM2::BSRW::PoseToArray(pose, pose_values);
        ORB_SLAM2::BSRW::EvaluationCounters counters;
        ceres::Problem problem;
        ORB_SLAM2::BSRW::Snapshot objective_snapshot = snapshot;
        int before = 0;
        for (std::size_t i = 0; i < snapshot.observations.size(); ++i) {
            objective_snapshot.observations[i].active = active[i];
            objective_snapshot.observations[i].use_huber =
                round < 3 && snapshot.observations[i].use_huber;
            if (!active[i]) continue;
            ++before;
            const auto& observation = snapshot.observations[i];
            ceres::LossFunction* loss = round < 3 && observation.use_huber
                ? static_cast<ceres::LossFunction*>(
                    new ceres::HuberLoss(observation.huber_delta))
                : nullptr;
            problem.AddResidualBlock(
                ORB_SLAM2::BSRW::CreateProjectionCost(snapshot, observation, &counters),
                loss, pose_values);
        }
        active_before.push_back(before);
        if (before == 0) return 7;
        problem.SetManifold(pose_values, new ORB_SLAM2::BSRW::LeftSE3Manifold);

        ceres::Problem::EvaluateOptions evaluate_options;
        double native_half_cost = 0.0;
        if (!problem.Evaluate(evaluate_options, &native_half_cost, nullptr, nullptr, nullptr))
            return 8;
        const double canonical_initial = ORB_SLAM2::BSRW::CanonicalObjective(
            objective_snapshot, pose, true);
        if (std::abs(2.0 * native_half_cost - canonical_initial) >
            1e-10 * std::max(1.0, std::abs(canonical_initial))) {
            std::cerr << "round " << round + 1 << " initial objective mismatch\n";
            return 9;
        }
        if (round == 0) first_initial_cost = canonical_initial;
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
        total_ms += std::chrono::duration_cast<
            std::chrono::duration<double, std::milli> >(
                std::chrono::steady_clock::now() - start).count();
        pose = ORB_SLAM2::BSRW::ArrayToPose(pose_values);

        int after = 0;
        for (std::size_t i = 0; i < snapshot.observations.size(); ++i) {
            const auto& observation = snapshot.observations[i];
            const double chi2 = ORB_SLAM2::BSRW::WeightedSquaredError(
                observation, ORB_SLAM2::BSRW::RawResidual(snapshot, observation, pose));
            const double threshold = observation.dimension == 2
                ? snapshot.mono_outlier_threshold : snapshot.stereo_outlier_threshold;
            active[i] = chi2 <= threshold;
            if (active[i]) ++after;
        }
        const int native = summary.iterations.empty()
            ? 0 : static_cast<int>(summary.iterations.size()) - 1;
        active_after.push_back(after);
        native_iterations.push_back(native);
        attempted.push_back(callback.accepted + callback.rejected);
        accepted.push_back(callback.accepted);
        rejected.push_back(callback.rejected);
        residual.push_back(counters.residual_evaluations.load());
        jacobian.push_back(counters.jacobian_evaluations.load());
        linear.push_back(callback.linear);
        reasons.push_back(ceres::TerminationTypeToString(summary.termination_type));
        logical_outer_work += native;
        total_attempted += callback.accepted + callback.rejected;
        total_accepted += callback.accepted;
        total_rejected += callback.rejected;
        total_residual += counters.residual_evaluations.load();
        total_jacobian += counters.jacobian_evaluations.load();
        total_linear += callback.linear;

        ORB_SLAM2::BSRW::Snapshot final_snapshot = snapshot;
        for (std::size_t i = 0; i < final_snapshot.observations.size(); ++i) {
            final_snapshot.observations[i].active = active[i];
            final_snapshot.observations[i].use_huber =
                round < 2 && final_snapshot.observations[i].use_huber;
        }
        last_final_cost = ORB_SLAM2::BSRW::CanonicalObjective(
            final_snapshot, pose, true);
        if (snapshot.observations.size() < 10) break;
    }

    std::cout << std::setprecision(17)
        << fixture_id << ",ceres," << ORB_SLAM2::BSRW::FileSha256(path)
        << ',' << snapshot.input_sha256 << ',' << budget << ','
        << native_iterations.size() << ',' << logical_outer_work << ','
        << total_attempted << ',' << total_accepted << ',' << total_rejected << ','
        << total_residual << ',' << total_jacobian << ',' << total_linear << ','
        << first_initial_cost << ',' << last_final_cost << ',' << total_ms << ','
        << Join(active_before) << ',' << Join(active_after) << ','
        << Join(native_iterations) << ',' << Join(attempted) << ','
        << Join(accepted) << ',' << Join(rejected) << ',' << Join(residual) << ','
        << Join(jacobian) << ',' << Join(linear) << ',' << Join(reasons) << ','
        << pose.translation.x() << ',' << pose.translation.y() << ','
        << pose.translation.z() << ',' << pose.quaternion.x() << ','
        << pose.quaternion.y() << ',' << pose.quaternion.z() << ','
        << pose.quaternion.w() << '\n';
    return 0;
}
