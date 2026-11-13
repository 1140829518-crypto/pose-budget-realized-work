#include "BSRWSnapshot.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>

namespace {

// The bundled ORB-SLAM2 stereo edge narrows inverse depth to float in
// cam_project().  The v1 snapshot contract is binary64 throughout, so the
// replay edge keeps the same formula but evaluates it in double precision.
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

std::string FileSha256(const std::string& path) {
    std::ifstream input(path.c_str(), std::ios::binary);
    const std::string bytes((std::istreambuf_iterator<char>(input)),
                            std::istreambuf_iterator<char>());
    return ORB_SLAM2::BSRW::Sha256Hex(bytes);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3 || argc > 4) {
        std::cerr << "usage: g2o_snapshot_replay PILOT_ID SNAPSHOT [BUDGET]\n";
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

    g2o::SparseOptimizer optimizer;
    auto* linear_solver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
    auto* block_solver = new g2o::BlockSolver_6_3(linear_solver);
    auto* solver = new g2o::OptimizationAlgorithmLevenberg(block_solver);
    optimizer.setAlgorithm(solver);
    optimizer.setWorkInstrumentationEnabled(true);
    solver->setWorkInstrumentationEnabled(true);

    auto* vertex = new g2o::VertexSE3Expmap();
    vertex->setId(0);
    vertex->setFixed(false);
    vertex->setEstimate(g2o::SE3Quat(snapshot.initial_pose.quaternion,
                                     snapshot.initial_pose.translation));
    optimizer.addVertex(vertex);
    int active_observations = 0;
    for (const auto& observation : snapshot.observations) {
        if (observation.active) ++active_observations;
        if (observation.dimension == 2) {
            auto* edge = new g2o::EdgeSE3ProjectXYZOnlyPose();
            edge->setVertex(0, optimizer.vertex(0));
            edge->setMeasurement(Eigen::Vector2d(observation.measurement[0],
                                                 observation.measurement[1]));
            edge->setInformation(observation.information.topLeftCorner<2,2>());
            edge->Xw = observation.point_w;
            edge->fx = snapshot.camera.fx; edge->fy = snapshot.camera.fy;
            edge->cx = snapshot.camera.cx; edge->cy = snapshot.camera.cy;
            edge->setLevel(observation.active ? 0 : 1);
            if (observation.use_huber) {
                auto* kernel = new g2o::RobustKernelHuber();
                kernel->setDelta(observation.huber_delta);
                edge->setRobustKernel(kernel);
            }
            optimizer.addEdge(edge);
        } else {
            auto* edge = new ReplayStereoEdge();
            edge->setVertex(0, optimizer.vertex(0));
            edge->setMeasurement(Eigen::Vector3d(observation.measurement[0],
                                                 observation.measurement[1],
                                                 observation.measurement[2]));
            edge->setInformation(observation.information);
            edge->Xw = observation.point_w;
            edge->fx = snapshot.camera.fx; edge->fy = snapshot.camera.fy;
            edge->cx = snapshot.camera.cx; edge->cy = snapshot.camera.cy;
            edge->bf = snapshot.camera.bf;
            edge->setLevel(observation.active ? 0 : 1);
            if (observation.use_huber) {
                auto* kernel = new g2o::RobustKernelHuber();
                kernel->setDelta(observation.huber_delta);
                edge->setRobustKernel(kernel);
            }
            optimizer.addEdge(edge);
        }
    }
    if (!optimizer.initializeOptimization(0)) return 5;
    optimizer.computeActiveErrors();
    const double initial_native = optimizer.activeRobustChi2();
    const double initial_canonical = ORB_SLAM2::BSRW::CanonicalObjective(
        snapshot, snapshot.initial_pose, true);
    if (std::abs(initial_native - initial_canonical) >
        1e-6 * std::max(1.0, std::abs(initial_canonical))) {
        std::cerr << "initial objective mismatch: native=" << std::setprecision(17)
                  << initial_native << " canonical=" << initial_canonical << '\n';
        return 6;
    }
    solver->resetWorkStatistics();
    const auto start = std::chrono::steady_clock::now();
    const int optimize_return = optimizer.optimize(budget);
    const double solver_ms = std::chrono::duration_cast<
        std::chrono::duration<double, std::milli> >(
            std::chrono::steady_clock::now() - start).count();
    optimizer.computeActiveErrors();
    const double final_native = optimizer.activeRobustChi2();
    const g2o::SE3Quat estimate = vertex->estimate();
    ORB_SLAM2::BSRW::PoseSE3 final_pose;
    final_pose.translation = estimate.translation();
    final_pose.quaternion = estimate.rotation();
    const double final_canonical = ORB_SLAM2::BSRW::CanonicalObjective(
        snapshot, final_pose, true);
    if (std::abs(final_native - final_canonical) >
        1e-6 * std::max(1.0, std::abs(final_canonical))) {
        std::cerr << "final objective mismatch\n";
        return 7;
    }
    const auto& lm = solver->workStatistics();
    const auto& run = optimizer.lastOptimizationRunStatistics();
    std::cout << std::setprecision(17)
        << pilot_id << ",g2o," << FileSha256(snapshot_path)
        << ',' << snapshot.input_sha256 << ',' << budget
        << ',' << lm.lmTrials << ',' << lm.acceptedTrials << ',' << lm.rejectedTrials
        << ',' << lm.residualEdgeEvaluations << ',' << lm.jacobianEdgeEvaluations
        << ',' << lm.linearSolveAttempts
        << ',' << initial_canonical << ',' << final_canonical
        << ',' << StopReason(run.stopReason) << ',' << solver_ms
        << ',' << optimize_return << ',' << active_observations << '\n';
    return 0;
}
