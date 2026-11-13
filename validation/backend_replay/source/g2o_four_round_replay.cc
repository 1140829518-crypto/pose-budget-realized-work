#include "BSRWSnapshot.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

namespace {

using ORB_SLAM2::BSRW::Observation;
using ORB_SLAM2::BSRW::PoseSE3;
using ORB_SLAM2::BSRW::Snapshot;

class ReplayStereoEdge final : public g2o::EdgeStereoSE3ProjectXYZOnlyPose {
 public:
    void computeError() override {
        const auto* vertex = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        const Eigen::Vector3d point = vertex->estimate().map(Xw);
        const double inverse_z = 1.0 / point.z();
        Eigen::Vector3d projection;
        projection[0] = point.x() * inverse_z * fx + cx;
        projection[1] = point.y() * inverse_z * fy + cy;
        projection[2] = projection[0] - bf * inverse_z;
        _error = _measurement - projection;
    }
};

struct EdgeHandle {
    g2o::EdgeSE3ProjectXYZOnlyPose* mono = nullptr;
    ReplayStereoEdge* stereo = nullptr;
    void ComputeError() { mono ? mono->computeError() : stereo->computeError(); }
    double Chi2() const { return mono ? mono->chi2() : stereo->chi2(); }
};

const char* StopReason(g2o::SparseOptimizer::OptimizationStopReason reason) {
    switch (reason) {
    case g2o::SparseOptimizer::OptimizationRequestedMaximum: return "REQUESTED_MAXIMUM";
    case g2o::SparseOptimizer::OptimizationAlgorithmTerminate: return "ALGORITHM_TERMINATE";
    case g2o::SparseOptimizer::OptimizationAlgorithmFail: return "ALGORITHM_FAIL";
    case g2o::SparseOptimizer::OptimizationExternalStop: return "EXTERNAL_STOP";
    case g2o::SparseOptimizer::OptimizationEmptyActiveGraph: return "EMPTY_ACTIVE_GRAPH";
    case g2o::SparseOptimizer::OptimizationInitializationFailure: return "INITIALIZATION_FAILURE";
    case g2o::SparseOptimizer::OptimizationZeroRequestedIterations: return "ZERO_REQUESTED_ITERATIONS";
    case g2o::SparseOptimizer::OptimizationOtherStop: return "OTHER_STOP";
    default: return "NOT_RUN";
    }
}

template <typename T>
std::string Join(const std::vector<T>& values) {
    std::ostringstream out;
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i) out << '|';
        out << values[i];
    }
    return out.str();
}

std::string FileSha256(const std::string& path) {
    std::ifstream input(path.c_str(), std::ios::binary);
    const std::string bytes((std::istreambuf_iterator<char>(input)),
                            std::istreambuf_iterator<char>());
    return ORB_SLAM2::BSRW::Sha256Hex(bytes);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "usage: g2o_four_round_replay FIXTURE_ID ROUND1_SNAPSHOT BUDGET\n";
        return 2;
    }
    const std::string fixture_id = argv[1];
    const std::string path = argv[2];
    const int budget = std::atoi(argv[3]);
    if (budget <= 0) return 3;

    Snapshot snapshot;
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

    PoseSE3 pose = snapshot.initial_pose;
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
        g2o::SparseOptimizer optimizer;
        auto* linear_solver =
            new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
        auto* block_solver = new g2o::BlockSolver_6_3(linear_solver);
        auto* solver = new g2o::OptimizationAlgorithmLevenberg(block_solver);
        optimizer.setAlgorithm(solver);
        optimizer.setWorkInstrumentationEnabled(true);
        solver->setWorkInstrumentationEnabled(true);

        auto* vertex = new g2o::VertexSE3Expmap();
        vertex->setId(0);
        vertex->setFixed(false);
        vertex->setEstimate(g2o::SE3Quat(pose.quaternion, pose.translation));
        optimizer.addVertex(vertex);

        std::vector<EdgeHandle> edges;
        edges.reserve(snapshot.observations.size());
        int before = 0;
        for (std::size_t i = 0; i < snapshot.observations.size(); ++i) {
            const Observation& observation = snapshot.observations[i];
            EdgeHandle handle;
            if (observation.dimension == 2) {
                auto* edge = new g2o::EdgeSE3ProjectXYZOnlyPose();
                edge->setVertex(0, optimizer.vertex(0));
                edge->setMeasurement(Eigen::Vector2d(
                    observation.measurement[0], observation.measurement[1]));
                edge->setInformation(observation.information.topLeftCorner<2,2>());
                edge->Xw = observation.point_w;
                edge->fx = snapshot.camera.fx; edge->fy = snapshot.camera.fy;
                edge->cx = snapshot.camera.cx; edge->cy = snapshot.camera.cy;
                edge->setLevel(active[i] ? 0 : 1);
                if (round < 3 && observation.use_huber) {
                    auto* kernel = new g2o::RobustKernelHuber();
                    kernel->setDelta(observation.huber_delta);
                    edge->setRobustKernel(kernel);
                }
                optimizer.addEdge(edge);
                handle.mono = edge;
            } else {
                auto* edge = new ReplayStereoEdge();
                edge->setVertex(0, optimizer.vertex(0));
                edge->setMeasurement(Eigen::Vector3d(
                    observation.measurement[0], observation.measurement[1],
                    observation.measurement[2]));
                edge->setInformation(observation.information);
                edge->Xw = observation.point_w;
                edge->fx = snapshot.camera.fx; edge->fy = snapshot.camera.fy;
                edge->cx = snapshot.camera.cx; edge->cy = snapshot.camera.cy;
                edge->bf = snapshot.camera.bf;
                edge->setLevel(active[i] ? 0 : 1);
                if (round < 3 && observation.use_huber) {
                    auto* kernel = new g2o::RobustKernelHuber();
                    kernel->setDelta(observation.huber_delta);
                    edge->setRobustKernel(kernel);
                }
                optimizer.addEdge(edge);
                handle.stereo = edge;
            }
            if (active[i]) ++before;
            edges.push_back(handle);
        }
        active_before.push_back(before);
        if (!optimizer.initializeOptimization(0)) return 7;
        optimizer.computeActiveErrors();
        const double native_initial = optimizer.activeRobustChi2();
        Snapshot objective_snapshot = snapshot;
        for (std::size_t i = 0; i < objective_snapshot.observations.size(); ++i) {
            objective_snapshot.observations[i].active = active[i];
            objective_snapshot.observations[i].use_huber =
                round < 3 && objective_snapshot.observations[i].use_huber;
        }
        const double canonical_initial =
            ORB_SLAM2::BSRW::CanonicalObjective(objective_snapshot, pose, true);
        if (std::abs(native_initial - canonical_initial) >
            1e-6 * std::max(1.0, std::abs(canonical_initial))) {
            std::cerr << "round " << round + 1 << " initial objective mismatch\n";
            return 8;
        }
        if (round == 0) first_initial_cost = canonical_initial;

        solver->resetWorkStatistics();
        const auto start = std::chrono::steady_clock::now();
        const int completed = optimizer.optimize(budget);
        total_ms += std::chrono::duration_cast<
            std::chrono::duration<double, std::milli> >(
                std::chrono::steady_clock::now() - start).count();
        const auto& lm = solver->workStatistics();
        const auto& run = optimizer.lastOptimizationRunStatistics();
        const g2o::SE3Quat estimate = vertex->estimate();
        pose.translation = estimate.translation();
        pose.quaternion = estimate.rotation();

        int after = 0;
        for (std::size_t i = 0; i < edges.size(); ++i) {
            edges[i].ComputeError();
            const double threshold = snapshot.observations[i].dimension == 2
                ? snapshot.mono_outlier_threshold : snapshot.stereo_outlier_threshold;
            active[i] = edges[i].Chi2() <= threshold;
            if (active[i]) ++after;
        }
        active_after.push_back(after);
        native_iterations.push_back(completed);
        attempted.push_back(lm.lmTrials);
        accepted.push_back(lm.acceptedTrials);
        rejected.push_back(lm.rejectedTrials);
        residual.push_back(lm.residualEdgeEvaluations);
        jacobian.push_back(lm.jacobianEdgeEvaluations);
        linear.push_back(lm.linearSolveAttempts);
        reasons.push_back(StopReason(run.stopReason));
        logical_outer_work += completed;
        total_attempted += lm.lmTrials;
        total_accepted += lm.acceptedTrials;
        total_rejected += lm.rejectedTrials;
        total_residual += lm.residualEdgeEvaluations;
        total_jacobian += lm.jacobianEdgeEvaluations;
        total_linear += lm.linearSolveAttempts;

        Snapshot final_snapshot = snapshot;
        for (std::size_t i = 0; i < final_snapshot.observations.size(); ++i) {
            final_snapshot.observations[i].active = active[i];
            final_snapshot.observations[i].use_huber =
                round < 2 && final_snapshot.observations[i].use_huber;
        }
        last_final_cost = ORB_SLAM2::BSRW::CanonicalObjective(final_snapshot, pose, true);
        if (snapshot.observations.size() < 10) break;
    }

    std::cout << std::setprecision(17)
        << fixture_id << ",g2o," << FileSha256(path)
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
