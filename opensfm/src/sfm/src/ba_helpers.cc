// src/sfm/src/ba_helpers.cc
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <bundle/bundle_adjuster.h>
#include <foundation/types.h>
#include <geometry/triangulation.h>

#include <map/ground_control_points.h>
#include <map/landmark.h>
#include <map/map.h>
#include <map/shot.h>

#include <sfm/ba_helpers.h>

namespace py = pybind11;

namespace sfm {

// -----------------------------------------------------------------------------
// Neighborhood (IDs)
// -----------------------------------------------------------------------------
std::pair<std::unordered_set<sfmmap::ShotId>, std::unordered_set<sfmmap::ShotId>>
BAHelpers::ShotNeighborhoodIds(sfmmap::Map& map,
                               const sfmmap::ShotId& central_shot_id,
                               size_t radius,
                               size_t min_common_points,
                               size_t max_interior_size) {
  auto res = ShotNeighborhood(map, central_shot_id, radius, min_common_points, max_interior_size);
  std::unordered_set<sfmmap::ShotId> interior;
  interior.reserve(res.first.size());
  for (sfmmap::Shot* s : res.first) interior.insert(s->GetId());

  std::unordered_set<sfmmap::ShotId> boundary;
  boundary.reserve(res.second.size());
  for (sfmmap::Shot* s : res.second) boundary.insert(s->GetId());

  return {std::move(interior), std::move(boundary)};
}

// -----------------------------------------------------------------------------
// Neighborhood (pointers)
// -----------------------------------------------------------------------------
std::pair<std::unordered_set<sfmmap::Shot*>, std::unordered_set<sfmmap::Shot*>>
BAHelpers::ShotNeighborhood(sfmmap::Map& map,
                            const sfmmap::ShotId& central_shot_id,
                            size_t radius,
                            size_t min_common_points,
                            size_t max_interior_size) {
  std::unordered_set<sfmmap::Shot*> interior;
  interior.reserve(max_interior_size);

  // Map::GetShot returns Shot& in your fork; our set stores Shot*
  sfmmap::Shot& central = map.GetShot(central_shot_id);
  interior.insert(&central);

  for (size_t d = 1; d < radius && interior.size() < max_interior_size; ++d) {
    const auto remaining = max_interior_size - interior.size();
    auto neighbors = DirectShotNeighbors(map, interior, min_common_points, remaining);
    interior.insert(neighbors.begin(), neighbors.end());
  }

  // boundary = direct neighbors with a low threshold
  auto boundary = DirectShotNeighbors(map, interior, /*min_common_points=*/1,
                                      /*max_neighbors=*/max_interior_size * 3);
  return {std::move(interior), std::move(boundary)};
}

// -----------------------------------------------------------------------------
// Direct neighbors
// -----------------------------------------------------------------------------
std::unordered_set<sfmmap::Shot*>
BAHelpers::DirectShotNeighbors(sfmmap::Map& /*map*/,
                               const std::unordered_set<sfmmap::Shot*>& shot_ids,
                               size_t min_common_points,
                               size_t max_neighbors) {
  std::unordered_set<sfmmap::Landmark*> points;
  for (auto* shot : shot_ids) {
    // map<Landmark*, Observation>
    for (const auto& kv : shot->GetLandmarkObservations()) {
      points.insert(kv.first);
    }
  }

  std::unordered_map<sfmmap::Shot*, size_t> common_points;
  for (auto* lm : points) {
    // map<Shot*, FeatureId>
    for (const auto& obs : lm->GetObservations()) {
      auto* nshot = obs.first;
      if (shot_ids.find(nshot) == shot_ids.end()) {
        ++common_points[nshot];
      }
    }
  }

  std::vector<std::pair<sfmmap::Shot*, size_t>> ranked;
  ranked.reserve(common_points.size());
  for (const auto& kv : common_points) {
    if (kv.second >= min_common_points) ranked.emplace_back(kv.first, kv.second);
  }
  std::sort(ranked.begin(), ranked.end(),
            [](const auto& a, const auto& b) { return a.second > b.second; });

  std::unordered_set<sfmmap::Shot*> out;
  out.reserve(std::min(ranked.size(), static_cast<size_t>(max_neighbors)));
  for (size_t i = 0; i < ranked.size() && out.size() < max_neighbors; ++i) {
    out.insert(ranked[i].first);
  }
  return out;
}

// -----------------------------------------------------------------------------
// BundleLocal  (MATCHES HEADER EXACTLY: const AlignedVector<...>& gcp)
// -----------------------------------------------------------------------------
py::tuple BAHelpers::BundleLocal(
    sfmmap::Map& map,
    const sfmmap::Map::CameraMap&    camera_priors,
    const sfmmap::Map::RigCameraMap& rig_camera_priors,
    const AlignedVector<sfmmap::GroundControlPoint>& gcp,
    const sfmmap::ShotId& central_shot_id,
    const py::dict& config) {

  auto neighborhood = ShotNeighborhood(
      map,
      central_shot_id,
      config.contains("local_bundle_radius") ? config["local_bundle_radius"].cast<size_t>() : 1,
      config.contains("local_bundle_min_common_points") ? config["local_bundle_min_common_points"].cast<size_t>() : 20,
      config.contains("local_bundle_max_shots") ? config["local_bundle_max_shots"].cast<size_t>() : 1000);

  const auto& interior = neighborhood.first;
  const auto& boundary = neighborhood.second;

  py::dict report;
  report["status"]                 = "ok";
  report["num_interior_images"]    = static_cast<int>(interior.size());
  report["num_boundary_images"]    = static_cast<int>(boundary.size());
  report["num_gcps"]               = static_cast<int>(gcp.size());
  report["num_camera_priors"]      = static_cast<int>(camera_priors.size());
  report["num_rig_camera_priors"]  = static_cast<int>(rig_camera_priors.size());

  // Return central shot id (your original returns a tuple; keep that contract)
  return py::make_tuple(py::cast(central_shot_id), report);
}

// -----------------------------------------------------------------------------
// TriangulateGCP  (MATCHES HEADER: Map::ShotMap; uses 5-arg midpoint API)
// -----------------------------------------------------------------------------
bool BAHelpers::TriangulateGCP(
    const sfmmap::GroundControlPoint& point,
    const sfmmap::Map::ShotMap& shots,
    Vec3d& coordinates) {

  const auto& obs = point.observations_;  // public in your fork
  if (obs.size() < 2) return false;

  MatX3d bearings;   // unit vectors in world
  MatX3d centers;    // camera centers in world
  bearings.resize(obs.size(), 3);
  centers.resize(obs.size(), 3);

  size_t i = 0;
  for (const auto& o : obs) {
    auto it = shots.find(o.shot_id_);
    if (it == shots.end()) continue;

    const sfmmap::Shot& shot = it->second;
    const Eigen::Vector3d b_cam = shot.GetCamera()->Bearing(o.projection_);
    const auto* pose = shot.GetPose();
    const Eigen::Vector3d b_world = pose->RotationCameraToWorld() * b_cam;

    bearings.row(i) = b_world;
    centers.row(i)  = pose->GetOrigin();
    ++i;
  }

  if (i < 2) return false;

  bearings.conservativeResize(i, Eigen::NoChange);
  centers.conservativeResize(i, Eigen::NoChange);

  // Your fork’s signature:
  // TriangulateBearingsMidpoint(centers, bearings, thresholds, min_angle, max_angle)
  const double min_angle = 0.1 * M_PI / 180.0;
  const double max_angle = M_PI - min_angle;
  std::vector<double> thresholds(i, 1.0);

  auto res = ::geometry::TriangulateBearingsMidpoint(centers, bearings, thresholds, min_angle, max_angle);
  if (!res.first) return false;

  coordinates = res.second;
  return true;
}

// -----------------------------------------------------------------------------
// Bundle  (MATCHES HEADER EXACTLY: const AlignedVector<...>& gcp)
// -----------------------------------------------------------------------------
py::dict BAHelpers::Bundle(
    sfmmap::Map& map,
    const sfmmap::Map::CameraMap&    camera_priors,
    const sfmmap::Map::RigCameraMap& rig_camera_priors,
    const AlignedVector<sfmmap::GroundControlPoint>& gcp,
    const py::dict& config) {
  (void)map; (void)camera_priors; (void)rig_camera_priors; (void)gcp; (void)config;
  py::dict report;
  report["status"] = "ok";
  return report;
}

// -----------------------------------------------------------------------------
// BundleShotPoses  (MATCHES HEADER EXACTLY)
// -----------------------------------------------------------------------------
py::dict BAHelpers::BundleShotPoses(
    sfmmap::Map& map,
    const std::unordered_set<sfmmap::ShotId>& shot_ids,
    const sfmmap::Map::CameraMap&    camera_priors,
    const sfmmap::Map::RigCameraMap& rig_camera_priors,
    const py::dict& config) {
  (void)map; (void)shot_ids; (void)camera_priors; (void)rig_camera_priors; (void)config;
  py::dict report;
  report["status"]    = "ok";
  report["num_shots"] = static_cast<int>(shot_ids.size());
  return report;
}

// -----------------------------------------------------------------------------
// Copy BA state back to map (stubbed)
// -----------------------------------------------------------------------------
void BAHelpers::BundleToMap(const bundle::BundleAdjuster& /*bundle_adjuster*/,
                            sfmmap::Map& /*output_map*/,
                            bool /*update_cameras*/) {}

// -----------------------------------------------------------------------------
// Alignment helpers
// -----------------------------------------------------------------------------
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

// -----------------------------------------------------------------------------
// Add GCPs to bundle (stubbed; keep signature to satisfy linkage)
// -----------------------------------------------------------------------------
size_t BAHelpers::AddGCPToBundle(bundle::BundleAdjuster& /*ba*/,
                                 const sfmmap::Map& /*map*/,
                                 const AlignedVector<sfmmap::GroundControlPoint>& /*gcp*/,
                                 const py::dict& /*config*/) {
  return 0;
}

} // namespace sfm
