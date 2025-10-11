// src/sfm/src/ba_helpers.cc
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <foundation/types.h>
#include <geometry/triangulation.h>
#include <map/ground_control_points.h>
#include <map/landmark.h>
#include <map/map.h>
#include <map/rig.h>
#include <map/shot.h>
#include <sfm/ba_helpers.h>

// Do NOT use 'using namespace' for geometry; there is also sfmmap::geometry.
namespace sfm {

std::pair<std::unordered_set<sfmmap::ShotId>, std::unordered_set<sfmmap::ShotId>>
BAHelpers::ShotNeighborhoodIds(sfmmap::Map& map,
                               const sfmmap::ShotId& central_shot_id,
                               size_t radius,
                               size_t min_common_points,
                               size_t max_interior_size) {
  auto res = ShotNeighborhood(map, central_shot_id, radius, min_common_points, max_interior_size);
  std::unordered_set<sfmmap::ShotId> interior;
  interior.reserve(res.first.size());
  for (sfmmap::Shot* shot : res.first) {
    interior.insert(shot->GetId());
  }
  std::unordered_set<sfmmap::ShotId> boundary;
  boundary.reserve(res.second.size());
  for (sfmmap::Shot* shot : res.second) {
    boundary.insert(shot->GetId());
  }
  return std::make_pair(std::move(interior), std::move(boundary));
}

std::pair<std::unordered_set<sfmmap::Shot*>, std::unordered_set<sfmmap::Shot*>>
BAHelpers::ShotNeighborhood(sfmmap::Map& map,
                            const sfmmap::ShotId& central_shot_id,
                            size_t radius,
                            size_t min_common_points,
                            size_t max_interior_size) {
  constexpr size_t kMaxBoundarySize = 1000000;

  std::unordered_set<sfmmap::Shot*> interior;
  interior.reserve(max_interior_size);

  // Map::GetShot returns Shot&, take address for our Shot* set.
  sfmmap::Shot& central_shot = map.GetShot(central_shot_id);
  // If this shot is part of a rig, include sibling shots in same instance.
  if (central_shot.HasRig()) {
    const auto& instance = map.GetRigInstance(central_shot.GetRigInstanceId());
    for (const auto& sid : instance.GetShotIDs()) {
      interior.insert(&map.GetShot(sid));
    }
  }
  interior.insert(&central_shot);

  for (size_t d = 1; d < radius && interior.size() < max_interior_size; ++d) {
    const auto remaining = max_interior_size - interior.size();
    const auto neighbors = DirectShotNeighbors(map, interior, min_common_points, remaining);
    interior.insert(neighbors.begin(), neighbors.end());
  }

  const auto boundary = DirectShotNeighbors(map, interior, 1, kMaxBoundarySize);
  return std::make_pair(std::move(interior), std::move(boundary));
}

std::unordered_set<sfmmap::Shot*>
BAHelpers::DirectShotNeighbors(sfmmap::Map& /*map*/,
                               const std::unordered_set<sfmmap::Shot*>& shot_ids,
                               size_t min_common_points,
                               size_t max_neighbors) {
  std::unordered_set<sfmmap::Landmark*> points;
  for (auto* shot : shot_ids) {
    for (const auto& kv : shot->GetLandmarkObservations()) {  // map<Landmark*, Observation>
      points.insert(kv.first);
    }
  }

  std::unordered_map<sfmmap::Shot*, size_t> common_points;
  for (auto* lm : points) {
    for (const auto& it : lm->GetObservations()) {  // map<Shot*, FeatureId>
      auto* nshot = it.first;
      if (shot_ids.find(nshot) == shot_ids.end()) {
        ++common_points[nshot];
      }
    }
  }

  std::vector<std::pair<sfmmap::Shot*, size_t>> pairs;
  pairs.reserve(common_points.size());
  for (const auto& kv : common_points) {
    if (kv.second >= min_common_points) pairs.emplace_back(kv.first, kv.second);
  }
  std::sort(pairs.begin(), pairs.end(),
            [](const auto& a, const auto& b) { return a.second > b.second; });

  std::unordered_set<sfmmap::Shot*> neighbors;
  neighbors.reserve(std::min(pairs.size(), max_neighbors));
  for (size_t i = 0; i < pairs.size() && neighbors.size() < max_neighbors; ++i) {
    neighbors.insert(pairs[i].first);
  }

  return neighbors;
}

// --- Use your typedefs from map_types.h here (prevents template/ABI mismatches) ---
py::tuple BAHelpers::BundleLocal(
    sfmmap::Map& map,
    const sfmmap::CameraMap& camera_priors,
    const sfmmap::RigCameraMap& rig_camera_priors,
    const AlignedVector<sfmmap::GroundControlPoint>& gcp,
    const sfmmap::ShotId& central_shot_id,
    const py::dict& config) {

  py::dict report;
  report["status"] = "ok";

  auto neighborhood = ShotNeighborhood(
      map, central_shot_id,
      config.contains("local_bundle_radius") ? config["local_bundle_radius"].cast<size_t>() : 1,
      config.contains("local_bundle_min_common_points") ? config["local_bundle_min_common_points"].cast<size_t>() : 20,
      config.contains("local_bundle_max_shots") ? config["local_bundle_max_shots"].cast<size_t>() : 1000);

  const auto& interior = neighborhood.first;
  const auto& boundary = neighborhood.second;

  report["num_interior_images"]    = static_cast<int>(interior.size());
  report["num_boundary_images"]    = static_cast<int>(boundary.size());
  report["num_gcps"]               = static_cast<int>(gcp.size());
  report["num_camera_priors"]      = static_cast<int>(camera_priors.size());
  report["num_rig_camera_priors"]  = static_cast<int>(rig_camera_priors.size());

  return py::make_tuple(py::cast(central_shot_id), report);
}

bool BAHelpers::TriangulateGCP(
    const sfmmap::GroundControlPoint& point,
    const sfmmap::Map::ShotMap& shots,
    Vec3d& coordinates) {

  MatX3d bearings;
  MatX3d centers;
  const auto& obs = point.observations_;  // public field in this fork
  const size_t N = obs.size();
  bearings.resize(N, 3);
  centers.resize(N, 3);

  size_t i = 0;
  for (const auto& o : obs) {
    auto it = shots.find(o.shot_id_);
    if (it == shots.end()) continue;
    const sfmmap::Shot& shot = it->second;

    const Eigen::Vector3d b = shot.Bearing(o.projection_);
    bearings.row(i) = b;

    const Eigen::Vector3d c = shot.GetPose()->GetOrigin();
    centers.row(i) = c;
    ++i;
  }

  if (i < 2) return false;

  bearings.conservativeResize(i, Eigen::NoChange);
  centers.conservativeResize(i, Eigen::NoChange);

  Eigen::Vector3d X(0,0,0);
  if (i == 2) {
    Eigen::Matrix<double,2,3> C;
    C << centers.row(0), centers.row(1);
    Eigen::Matrix<double,2,3> B;
    B << bearings.row(0), bearings.row(1);
    auto ok_X = ::geometry::TriangulateTwoBearingsMidpointSolve<double>(C, B);
    X = ok_X.second;
  } else {
    std::vector<Eigen::Matrix<double,3,4>> Rts(i, Eigen::Matrix<double,3,4>::Zero());
    for (size_t k = 0; k < i; ++k) {
      Rts[k].setIdentity();
      Rts[k].col(3) = -centers.row(k).transpose();
    }
    auto ok_X = ::geometry::TriangulateBearingsDLT(Rts, bearings, 4.0, 1e-6);
    if (!ok_X.first) return false;
    X = ok_X.second;
  }

  coordinates = X;
  return true;
}

py::dict BAHelpers::Bundle(
    sfmmap::Map& /*map*/,
    const sfmmap::CameraMap& /*camera_priors*/,
    const sfmmap::RigCameraMap& /*rig_camera_priors*/,
    const AlignedVector<sfmmap::GroundControlPoint>& /*gcp*/,
    const py::dict& /*config*/) {
  py::dict report;
  report["status"] = "ok";
  return report;
}

py::dict BAHelpers::BundleShotPoses(
    sfmmap::Map& /*map*/,
    const std::unordered_set<sfmmap::ShotId>& shot_ids,
    const sfmmap::CameraMap& /*camera_priors*/,
    const sfmmap::RigCameraMap& /*rig_camera_priors*/,
    const py::dict& /*config*/) {
  py::dict report;
  report["status"] = "ok";
  report["num_shots"] = static_cast<int>(shot_ids.size());
  return report;
}

void BAHelpers::BundleToMap(const bundle::BundleAdjuster& /*bundle_adjuster*/,
                            sfmmap::Map& /*output_map*/,
                            bool /*update_cameras*/) {
  // no-op
}

std::string BAHelpers::DetectAlignmentConstraints(
    const sfmmap::Map& /*map*/,
    const py::dict& /*config*/,
    const AlignedVector<sfmmap::GroundControlPoint>& /*gcp*/) {
  return "none";
}

void BAHelpers::AlignmentConstraints(
    const sfmmap::Map& /*map*/,
    const py::dict& /*config*/,
    const AlignedVector<sfmmap::GroundControlPoint>& /*gcp*/,
    MatX3d& Xp,
    MatX3d& X) {
  Xp.resize(0, 3);
  X.resize(0, 3);
}

size_t BAHelpers::AddGCPToBundle(bundle::BundleAdjuster& /*ba*/,
                                 const sfmmap::Map& /*map*/,
                                 const AlignedVector<sfmmap::GroundControlPoint>& /*gcp*/,
                                 const py::dict& /*config*/) {
  return 0;
}

} // namespace sfm
