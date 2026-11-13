#ifndef ORB_SLAM2_BSRW_SNAPSHOT_H
#define ORB_SLAM2_BSRW_SNAPSHOT_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <string>
#include <vector>

namespace ORB_SLAM2 {
namespace BSRW {

struct PoseSE3 {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d translation;
    Eigen::Quaterniond quaternion;
    PoseSE3() : translation(Eigen::Vector3d::Zero()), quaternion(Eigen::Quaterniond::Identity()) {}
};

struct CameraParameters {
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    double bf = 0.0;
};

struct Observation {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::uint64_t observation_id = 0;
    std::uint64_t map_point_id = 0;
    std::string type;
    Eigen::Vector3d point_w = Eigen::Vector3d::Zero();
    std::vector<double> measurement;
    Eigen::Matrix3d information = Eigen::Matrix3d::Zero();
    int dimension = 0;
    double base_information_scale = 1.0;
    double temporal_reliability = 1.0;
    double belief_reliability = 1.0;
    double final_reliability = 1.0;
    bool active = true;
    bool use_huber = true;
    double huber_delta = 0.0;
};

struct Snapshot {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::string schema_version = "bsrw.pose_snapshot.v1";
    std::string snapshot_id;
    std::string source_run_id;
    std::uint64_t frame_id = 0;
    std::string caller_id;
    PoseSE3 initial_pose;
    CameraParameters camera;
    std::vector<Observation, Eigen::aligned_allocator<Observation> > observations;
    int round_id = 1;
    int budget = 1;
    double mono_outlier_threshold = 5.991;
    double stereo_outlier_threshold = 7.815;
    std::string orb_slam2_head;
    std::string g2o_version;
    std::string ceres_version;
    std::string input_sha256;
};

std::string EncodeExactDouble(double value);
double DecodeExactDouble(const std::string& value);
std::string Sha256Hex(const std::string& bytes);

bool ValidateSnapshot(const Snapshot& snapshot, std::string* error);
bool WriteSnapshot(const Snapshot& snapshot, const std::string& path,
                   std::string* input_sha256, std::string* error);
bool ReadSnapshot(const std::string& path, Snapshot* snapshot, std::string* error);

Eigen::Vector3d TransformPoint(const PoseSE3& pose, const Eigen::Vector3d& point_w);
std::vector<double> Project(const Snapshot& snapshot, const Observation& observation,
                            const PoseSE3& pose);
std::vector<double> RawResidual(const Snapshot& snapshot, const Observation& observation,
                                const PoseSE3& pose);
double WeightedSquaredError(const Observation& observation,
                            const std::vector<double>& residual);
double HuberCost(double squared_error, double delta);
double CanonicalObjective(const Snapshot& snapshot, const PoseSE3& pose,
                          bool active_only);

}  // namespace BSRW
}  // namespace ORB_SLAM2

#endif

