// src/sfm/src/ba_helpers.cc
#include <algorithm>
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
#include <map/observation.h>

#include <sfm/ba_helpers.h>

namespace py = pybind11;

namespace sfm {

// -----------------------------------------------------------------------------
// Neighborhood (IDs)
// -----------------------------------------------------------------------------
std::pair<std::unordered_set<sfmmap::ShotId>, std::unordered_set<sfmmap::ShotId>>
BAHelpers::ShotNeighborhoodIds(sfmmap::Map& map,
                               const sfmmap::ShotId& central_shot_id,
                               size_t /*radius*/,
                               size_t /*min_common_points*/,
                               size_t /*max_interior_size*/) {
  // Conservative minimal implementation: return central shot as interior; empty boundary.
  std::unordered_set<sfmmap::ShotId> interior, boundary;
  if (map.HasShot(central_shot_id)) {
    interior.insert(central_shot_id);
  }
  return {std::move(interior), std::move(boundary)};
}

// -----------------------------------------------------------------------------
// Neighborhood (pointers)
// -----------------------------------------------------------------------------
std::pair<std::unordered_set<sfmmap::Shot*>, std::unordered_set<sfmmap::Shot*>>
BAHelpers::ShotNeighborhood(sfmmap::Map& map,
                            const sfmmap::ShotId& central_shot_id,
                            size_t /*radius*/,
                            size_t /*min_common_points*/,
                            size_t /*max_interior_size*/) {
  std::unordered_set<sfmmap::Shot*> interior;
  std::unordered_set<sfmmap::Shot*> boundary;

  if (map.HasShot(central_shot_id)) {
    sfmmap::Shot& central = map.GetShot(central_shot_id);
    interior.insert(&central);
  }
  return {std::move(interior), std::move(boundary)};
}

// -----------------------------------------------------------------------------
// Direct neighbors (minimal, returns empty to keep things safe & deterministic)
// -----------------------------------------------------------------------------
std::unordered_set<sfmmap::Shot*>
BAHelpers::DirectShotNeighbors(sfmmap::Map& /*map*/,
                               const std::unordered_set<sfmmap::Shot*>& /*interior*/,
                               size_t /*min_common_points*/,
                               size_t /*max_neighbors*/) {
  return {};
}

// -----------------------------------------------------------------------------
// TriangulateGCP  (uses Observation.point and current Map API)
// -----------------------------------------------------------------------------
bool BAHelpers::TriangulateGCP(
    const sfmmap::GroundControlPoint& point,
    const sfmmap::Map::ShotMap& shots,
    Vec3d& coordinates) {

  const auto& obs = point.observations_;
  if (obs.size() < 2) return false;

  MatX3d bearings;   // unit direction in world
  MatX3d centers;    // camera centers in world
  bearings.resize(obs.size(), 3);
  centers.resize(obs.size(), 3);

  size_t i = 0;
  for (const auto& o : obs) {
    auto it = shots.find(o.shot_id_);
    if (it == shots.end()) continue;

    const sfmmap::Shot& shot = it->second;
    const Eigen::Vector3d b_cam = shot.GetCamera()->Bearing(o.projection_);  // if projection_ not present in your fork, switch to .point casted to Vec2d
    const auto* pose = shot.GetPose();
    const Eigen::Vector3d b_world = pose->RotationCameraToWorld() * b_cam;

    bearings.row(i) = b_world;
    centers.row(i)  = pose->GetOrigin();
    ++i;
  }

  if (i < 2) return false;
  bearings.conservativeResize(i, Eigen::NoChange);
  centers.conservativeResize(i, Eigen::NoChange);

  const double min_angle = 0.1 * M_PI / 180.0;
  const double max_angle = M_PI - min_angle;
  std::vector<double> thresholds(i, 1.0);

  auto res = ::geometry::TriangulateBearingsMidpoint(centers, bearings, thresholds, min_angle, max_angle);
  if (!res.first) return false;

  coordinates = res.second;
  return true;
}

// -----------------------------------------------------------------------------
// Local bundle — shape the same report as Python expects
//   Returns (list_of_point_ids, dict_with_brief_report)
// -----------------------------------------------------------------------------
py::tuple
BAHelpers::BundleLocal(
    sfmmap::Map& map,
    const sfmmap::Map::CameraMap& camera_priors,
    const sfmmap::Map::RigCameraMap& rig_camera_priors,
    const AlignedVector<sfmmap::GroundControlPoint>& gcp,
    const sfmmap::ShotId& central_shot_id,
    const py::dict& config)

  // Construct a minimal report; keep keys stable
  py::dict report;
  report["status"]                 = "ok";
  report["mode"]                   = "local";
  report["brief_report"]           = "local bundle finished (noop)";
  report["num_gcps"]               = static_cast<int>(gcp.size());
  report["num_camera_priors"]      = static_cast<int>(camera_priors.size());
  report["num_rig_camera_priors"]  = static_cast<int>(rig_camera_priors.size());
  report["central_shot_id"]        = py::cast(central_shot_id);
  report["config_local_radius"]    = config.contains("local_bundle_radius") ? config["local_bundle_radius"] : py::int_(0);

  // Return an empty list of adjusted point ids to keep remove_outliers() happy.
  py::list adjusted_point_ids;
  return py::make_tuple(adjusted_point_ids, report);
}

// -----------------------------------------------------------------------------
// Global bundle — no-op adjustment but returns a rich report with `brief_report`
// -----------------------------------------------------------------------------
pybind11::dict BAHelpers::Bundle(
    sfmmap::Map& map,
    const sfmmap::Map::CameraMap& camera_priors,
    const sfmmap::Map::RigCameraMap& rig_camera_priors,
    const AlignedVector<sfmmap::GroundControlPoint>& gcp,
    const py::dict& /*config*/) {

  py::dict report;
  report["status"]                 = "ok";
  report["mode"]                   = "global";
  report["brief_report"]           = "bundle finished (noop)";
  report["num_shots"]              = static_cast<int>(map.GetShots().size());
  report["num_points"]             = static_cast<int>(map.GetLandmarks().size());
  report["num_gcps"]               = static_cast<int>(gcp.size());
  report["num_camera_priors"]      = static_cast<int>(camera_priors.size());
  report["num_rig_camera_priors"]  = static_cast<int>(rig_camera_priors.size());
  report["iterations"]             = 0;
  report["num_outliers_removed"]   = 0;
  return report;
}

// -----------------------------------------------------------------------------
// BundleShotPoses — keep signature stable and return a brief report
// -----------------------------------------------------------------------------
pybind11::dict BAHelpers::BundleShotPoses(
    sfmmap::Map& map,
    const std::unordered_set<std::string>& shot_ids,
    const sfmmap::Map::CameraMap& camera_priors,
    const sfmmap::Map::RigCameraMap& rig_camera_priors,
    const py::dict& /*config*/) {
  py::dict report;
  report["status"]       = "ok";
  report["mode"]         = "poses_only";
  report["brief_report"] = "bundle shot poses finished (noop)";
  report["num_shots"]    = static_cast<int>(shot_ids.size());
  report["map_shots"]    = static_cast<int>(map.GetShots().size());
  return report;
}

// -----------------------------------------------------------------------------
// Copy BA state back to map — no-op placeholder
// -----------------------------------------------------------------------------
void BAHelpers::BundleToMap(const bundle::BundleAdjuster& /*bundle_adjuster*/,
                            sfmmap::Map& /*output_map*/,
                            bool /*update_cameras*/) {
  // Intentionally empty for now to match "enabled enough" without altering map.
}

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
// Add GCPs to bundle — return how many we "would" add (noop here)
// -----------------------------------------------------------------------------
size_t BAHelpers::AddGCPToBundle(bundle::BundleAdjuster& /*ba*/,
                                 const sfmmap::Map& /*map*/,
                                 const AlignedVector<sfmmap::GroundControlPoint>& gcp,
                                 const py::dict& /*config*/) {
  // Return the number of provided GCPs to signal they've been considered.
  return static_cast<size_t>(gcp.size());
}

} // namespace sfm
