#include "BSRWSnapshot.h"

#include <json/json.h>
#include <openssl/sha.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>

namespace ORB_SLAM2 {
namespace BSRW {
namespace {

Json::Value Exact(double value) { return Json::Value(EncodeExactDouble(value)); }

Json::Value VectorValue(const Eigen::Vector3d& value) {
    Json::Value out(Json::arrayValue);
    for (int i = 0; i < 3; ++i) out.append(Exact(value[i]));
    return out;
}

std::string CanonicalJson(const Json::Value& value) {
    Json::StreamWriterBuilder builder;
    builder["commentStyle"] = "None";
    builder["indentation"] = "";
    builder["enableYAMLCompatibility"] = false;
    builder["dropNullPlaceholders"] = false;
    builder["emitUTF8"] = true;
    return Json::writeString(builder, value) + "\n";
}

Json::Value SnapshotRoot(const Snapshot& snapshot, bool include_hash) {
    Json::Value root(Json::objectValue);
    root["schema_version"] = snapshot.schema_version;
    root["snapshot_id"] = snapshot.snapshot_id;
    root["source_run_id"] = snapshot.source_run_id;
    root["frame_id"] = Json::UInt64(snapshot.frame_id);
    root["caller_id"] = snapshot.caller_id;
    root["quaternion_convention"] = "xyzw; unit quaternion; T_cw rotation";
    root["perturbation_convention"] =
        "left SE(3): T_new=Exp([omega,upsilon])*T_old; tangent order wx,wy,wz,tx,ty,tz";
    root["coordinate_convention"] =
        "T_cw maps world metres to camera metres; image coordinates in pixels";
    root["numeric_encoding"] = "IEEE-754 binary64 C99 hexadecimal strings";

    Json::Value pose(Json::objectValue);
    pose["transform"] = "T_cw";
    pose["translation_m"] = VectorValue(snapshot.initial_pose.translation);
    Json::Value quaternion(Json::arrayValue);
    quaternion.append(Exact(snapshot.initial_pose.quaternion.x()));
    quaternion.append(Exact(snapshot.initial_pose.quaternion.y()));
    quaternion.append(Exact(snapshot.initial_pose.quaternion.z()));
    quaternion.append(Exact(snapshot.initial_pose.quaternion.w()));
    pose["quaternion_xyzw"] = quaternion;
    root["initial_SE3"] = pose;

    Json::Value camera(Json::objectValue);
    camera["fx_px"] = Exact(snapshot.camera.fx);
    camera["fy_px"] = Exact(snapshot.camera.fy);
    camera["cx_px"] = Exact(snapshot.camera.cx);
    camera["cy_px"] = Exact(snapshot.camera.cy);
    camera["bf_px_m"] = Exact(snapshot.camera.bf);
    root["camera_parameters"] = camera;

    std::map<std::uint64_t, Eigen::Vector3d> points;
    for (const Observation& observation : snapshot.observations)
        points[observation.map_point_id] = observation.point_w;
    Json::Value map_points(Json::arrayValue);
    for (const auto& point : points) {
        Json::Value item(Json::objectValue);
        item["map_point_id"] = Json::UInt64(point.first);
        item["position_w_m"] = VectorValue(point.second);
        map_points.append(item);
    }
    root["map_points"] = map_points;

    Json::Value observations(Json::arrayValue);
    Json::Value information(Json::arrayValue);
    Json::Value reliability(Json::arrayValue);
    Json::Value kernels(Json::arrayValue);
    Json::Value active(Json::arrayValue);
    for (const Observation& observation : snapshot.observations) {
        Json::Value item(Json::objectValue);
        item["observation_id"] = Json::UInt64(observation.observation_id);
        item["map_point_id"] = Json::UInt64(observation.map_point_id);
        item["type"] = observation.type;
        Json::Value measurement(Json::arrayValue);
        for (double value : observation.measurement) measurement.append(Exact(value));
        item["measurement_px"] = measurement;
        observations.append(item);

        Json::Value info(Json::objectValue);
        info["observation_id"] = Json::UInt64(observation.observation_id);
        Json::Value matrix(Json::arrayValue);
        for (int row = 0; row < observation.dimension; ++row) {
            Json::Value values(Json::arrayValue);
            for (int col = 0; col < observation.dimension; ++col)
                values.append(Exact(observation.information(row, col)));
            matrix.append(values);
        }
        info["matrix"] = matrix;
        information.append(info);

        Json::Value weight(Json::objectValue);
        weight["observation_id"] = Json::UInt64(observation.observation_id);
        weight["base_information_scale"] = Exact(observation.base_information_scale);
        weight["temporal_reliability"] = Exact(observation.temporal_reliability);
        weight["belief_reliability"] = Exact(observation.belief_reliability);
        weight["final_reliability"] = Exact(observation.final_reliability);
        reliability.append(weight);

        Json::Value kernel(Json::objectValue);
        kernel["observation_id"] = Json::UInt64(observation.observation_id);
        kernel["type"] = observation.use_huber ? "Huber" : "None";
        kernel["delta"] = Exact(observation.huber_delta);
        kernels.append(kernel);
        if (observation.active) active.append(Json::UInt64(observation.observation_id));
    }
    root["observations"] = observations;
    root["information_matrices"] = information;
    root["reliability_weights"] = reliability;
    Json::Value robust(Json::objectValue);
    robust["argument"] = "squared Mahalanobis error e^T Omega e";
    robust["per_observation"] = kernels;
    root["robust_kernel"] = robust;
    root["active_observations"] = active;
    root["round_id"] = snapshot.round_id;
    root["budget"] = snapshot.budget;
    Json::Value threshold(Json::objectValue);
    threshold["mono_chi2"] = Exact(snapshot.mono_outlier_threshold);
    threshold["stereo_rgbd_chi2"] = Exact(snapshot.stereo_outlier_threshold);
    root["outlier_threshold"] = threshold;
    Json::Value versions(Json::objectValue);
    versions["orb_slam2_head"] = snapshot.orb_slam2_head;
    versions["g2o"] = snapshot.g2o_version;
    versions["ceres"] = snapshot.ceres_version;
    root["software_versions"] = versions;
    if (include_hash) root["input_sha256"] = snapshot.input_sha256;
    return root;
}

bool Require(bool condition, const std::string& message, std::string* error) {
    if (condition) return true;
    if (error) *error = message;
    return false;
}

const Json::Value* FindByObservationId(const Json::Value& array, std::uint64_t id) {
    for (Json::ArrayIndex i = 0; i < array.size(); ++i)
        if (array[i]["observation_id"].asUInt64() == id) return &array[i];
    return nullptr;
}

}  // namespace

std::string EncodeExactDouble(double value) {
    std::ostringstream stream;
    stream << std::hexfloat << std::setprecision(std::numeric_limits<double>::max_digits10)
           << value;
    return stream.str();
}

double DecodeExactDouble(const std::string& value) {
    std::size_t consumed = 0;
    const double result = std::stod(value, &consumed);
    if (consumed != value.size()) throw std::runtime_error("invalid exact double: " + value);
    return result;
}

std::string Sha256Hex(const std::string& bytes) {
    unsigned char digest[SHA256_DIGEST_LENGTH];
    SHA256(reinterpret_cast<const unsigned char*>(bytes.data()), bytes.size(), digest);
    std::ostringstream stream;
    stream << std::hex << std::setfill('0');
    for (unsigned char value : digest) stream << std::setw(2) << static_cast<int>(value);
    return stream.str();
}

bool ValidateSnapshot(const Snapshot& snapshot, std::string* error) {
    if (!Require(snapshot.schema_version == "bsrw.pose_snapshot.v1", "unsupported schema", error)) return false;
    if (!Require(!snapshot.snapshot_id.empty(), "empty snapshot_id", error)) return false;
    if (!Require(snapshot.round_id >= 1 && snapshot.round_id <= 4, "round_id out of range", error)) return false;
    if (!Require(snapshot.budget > 0, "budget must be positive", error)) return false;
    if (!Require(snapshot.observations.size() >= 3, "fewer than three observations", error)) return false;
    if (!Require(std::abs(snapshot.initial_pose.quaternion.norm() - 1.0) < 1e-12,
                 "initial quaternion is not unit length", error)) return false;
    std::map<std::uint64_t, bool> ids;
    for (const Observation& observation : snapshot.observations) {
        if (!Require(observation.dimension == 2 || observation.dimension == 3,
                     "observation dimension is not 2 or 3", error)) return false;
        if (!Require(static_cast<int>(observation.measurement.size()) == observation.dimension,
                     "measurement dimension mismatch", error)) return false;
        if (!Require(ids.insert(std::make_pair(observation.observation_id, true)).second,
                     "duplicate observation_id", error)) return false;
        if (!Require(observation.type == "mono" || observation.type == "stereo_rgbd",
                     "unknown observation type", error)) return false;
        if (!Require(observation.information.topLeftCorner(observation.dimension,
                                                            observation.dimension)
                         .isApprox(observation.information.topLeftCorner(observation.dimension,
                                                                         observation.dimension)
                                       .transpose(), 0.0),
                     "information matrix is not exactly symmetric", error)) return false;
    }
    return true;
}

bool WriteSnapshot(const Snapshot& source, const std::string& path,
                   std::string* input_sha256, std::string* error) {
    Snapshot snapshot = source;
    if (!ValidateSnapshot(snapshot, error)) return false;
    snapshot.input_sha256 = Sha256Hex(CanonicalJson(SnapshotRoot(snapshot, false)));
    const std::string bytes = CanonicalJson(SnapshotRoot(snapshot, true));
    const std::string temporary = path + ".tmp";
    std::ofstream output(temporary.c_str(), std::ios::binary | std::ios::trunc);
    if (!output) return Require(false, "cannot open temporary snapshot", error);
    output.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    output.close();
    if (!output) return Require(false, "cannot write complete snapshot", error);
    if (std::rename(temporary.c_str(), path.c_str()) != 0)
        return Require(false, "cannot atomically rename snapshot", error);
    if (input_sha256) *input_sha256 = snapshot.input_sha256;
    return true;
}

bool ReadSnapshot(const std::string& path, Snapshot* snapshot, std::string* error) {
    if (!Require(snapshot != nullptr, "null snapshot output", error)) return false;
    std::ifstream input(path.c_str(), std::ios::binary);
    if (!Require(static_cast<bool>(input), "cannot open snapshot", error)) return false;
    Json::CharReaderBuilder builder;
    builder["collectComments"] = false;
    Json::Value root;
    std::string parse_error;
    if (!Json::parseFromStream(builder, input, &root, &parse_error))
        return Require(false, "JSON parse failure: " + parse_error, error);
    try {
        Snapshot result;
        result.schema_version = root["schema_version"].asString();
        result.snapshot_id = root["snapshot_id"].asString();
        result.source_run_id = root["source_run_id"].asString();
        result.frame_id = root["frame_id"].asUInt64();
        result.caller_id = root["caller_id"].asString();
        const Json::Value& pose = root["initial_SE3"];
        for (int i = 0; i < 3; ++i)
            result.initial_pose.translation[i] = DecodeExactDouble(pose["translation_m"][i].asString());
        const Json::Value& q = pose["quaternion_xyzw"];
        result.initial_pose.quaternion = Eigen::Quaterniond(
            DecodeExactDouble(q[3].asString()), DecodeExactDouble(q[0].asString()),
            DecodeExactDouble(q[1].asString()), DecodeExactDouble(q[2].asString()));
        const Json::Value& camera = root["camera_parameters"];
        result.camera.fx = DecodeExactDouble(camera["fx_px"].asString());
        result.camera.fy = DecodeExactDouble(camera["fy_px"].asString());
        result.camera.cx = DecodeExactDouble(camera["cx_px"].asString());
        result.camera.cy = DecodeExactDouble(camera["cy_px"].asString());
        result.camera.bf = DecodeExactDouble(camera["bf_px_m"].asString());
        result.round_id = root["round_id"].asInt();
        result.budget = root["budget"].asInt();
        result.mono_outlier_threshold =
            DecodeExactDouble(root["outlier_threshold"]["mono_chi2"].asString());
        result.stereo_outlier_threshold =
            DecodeExactDouble(root["outlier_threshold"]["stereo_rgbd_chi2"].asString());
        result.orb_slam2_head = root["software_versions"]["orb_slam2_head"].asString();
        result.g2o_version = root["software_versions"]["g2o"].asString();
        result.ceres_version = root["software_versions"]["ceres"].asString();
        result.input_sha256 = root["input_sha256"].asString();

        std::map<std::uint64_t, Eigen::Vector3d> points;
        for (const Json::Value& item : root["map_points"]) {
            Eigen::Vector3d point;
            for (int i = 0; i < 3; ++i)
                point[i] = DecodeExactDouble(item["position_w_m"][i].asString());
            points[item["map_point_id"].asUInt64()] = point;
        }
        std::map<std::uint64_t, bool> active;
        for (const Json::Value& id : root["active_observations"])
            active[id.asUInt64()] = true;
        for (const Json::Value& item : root["observations"]) {
            Observation observation;
            observation.observation_id = item["observation_id"].asUInt64();
            observation.map_point_id = item["map_point_id"].asUInt64();
            observation.type = item["type"].asString();
            observation.dimension = observation.type == "mono" ? 2 : 3;
            observation.point_w = points.at(observation.map_point_id);
            for (const Json::Value& value : item["measurement_px"])
                observation.measurement.push_back(DecodeExactDouble(value.asString()));
            const Json::Value* info = FindByObservationId(
                root["information_matrices"], observation.observation_id);
            const Json::Value* weight = FindByObservationId(
                root["reliability_weights"], observation.observation_id);
            const Json::Value* kernel = FindByObservationId(
                root["robust_kernel"]["per_observation"], observation.observation_id);
            if (!info || !weight || !kernel)
                throw std::runtime_error("missing aligned observation metadata");
            for (int row = 0; row < observation.dimension; ++row)
                for (int col = 0; col < observation.dimension; ++col)
                    observation.information(row, col) =
                        DecodeExactDouble((*info)["matrix"][row][col].asString());
            observation.base_information_scale =
                DecodeExactDouble((*weight)["base_information_scale"].asString());
            observation.temporal_reliability =
                DecodeExactDouble((*weight)["temporal_reliability"].asString());
            observation.belief_reliability =
                DecodeExactDouble((*weight)["belief_reliability"].asString());
            observation.final_reliability =
                DecodeExactDouble((*weight)["final_reliability"].asString());
            observation.use_huber = (*kernel)["type"].asString() == "Huber";
            observation.huber_delta = DecodeExactDouble((*kernel)["delta"].asString());
            observation.active = active.count(observation.observation_id) != 0;
            result.observations.push_back(observation);
        }
        const std::string expected = Sha256Hex(CanonicalJson(SnapshotRoot(result, false)));
        if (!Require(expected == result.input_sha256, "input_sha256 mismatch", error)) return false;
        if (!ValidateSnapshot(result, error)) return false;
        *snapshot = result;
        return true;
    } catch (const std::exception& exception) {
        return Require(false, std::string("snapshot decode failure: ") + exception.what(), error);
    }
}

Eigen::Vector3d TransformPoint(const PoseSE3& pose, const Eigen::Vector3d& point_w) {
    return pose.quaternion * point_w + pose.translation;
}

std::vector<double> Project(const Snapshot& snapshot, const Observation& observation,
                            const PoseSE3& pose) {
    const Eigen::Vector3d point_c = TransformPoint(pose, observation.point_w);
    const double inverse_z = 1.0 / point_c.z();
    const double u = snapshot.camera.fx * point_c.x() * inverse_z + snapshot.camera.cx;
    const double v = snapshot.camera.fy * point_c.y() * inverse_z + snapshot.camera.cy;
    if (observation.dimension == 2) return {u, v};
    return {u, v, u - snapshot.camera.bf * inverse_z};
}

std::vector<double> RawResidual(const Snapshot& snapshot, const Observation& observation,
                                const PoseSE3& pose) {
    const std::vector<double> projection = Project(snapshot, observation, pose);
    std::vector<double> residual(observation.dimension);
    for (int i = 0; i < observation.dimension; ++i)
        residual[i] = observation.measurement[i] - projection[i];
    return residual;
}

double WeightedSquaredError(const Observation& observation,
                            const std::vector<double>& residual) {
    double value = 0.0;
    for (int row = 0; row < observation.dimension; ++row)
        for (int col = 0; col < observation.dimension; ++col)
            value += residual[row] * observation.information(row, col) * residual[col];
    return value;
}

double HuberCost(double squared_error, double delta) {
    const double delta_squared = delta * delta;
    return squared_error <= delta_squared
        ? squared_error
        : 2.0 * delta * std::sqrt(squared_error) - delta_squared;
}

double CanonicalObjective(const Snapshot& snapshot, const PoseSE3& pose,
                          bool active_only) {
    double objective = 0.0;
    for (const Observation& observation : snapshot.observations) {
        if (active_only && !observation.active) continue;
        const double squared_error =
            WeightedSquaredError(observation, RawResidual(snapshot, observation, pose));
        objective += observation.use_huber
            ? HuberCost(squared_error, observation.huber_delta)
            : squared_error;
    }
    return objective;
}

}  // namespace BSRW
}  // namespace ORB_SLAM2
