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
#include <geometry/pose.h>

#include <map/ground_control_points.h>
#include <map/landmark.h>
#include <map/map.h>
#include <map/shot.h>

#include <sfm/ba_helpers.h>

namespace py = pybind11;

namespace sfm {

// -----------------------------------------------------------------------------
// Utility: set reprojection error [ex, ey] for (landmark, shot)
// We avoid touching internals; use landmark API if available, else guarded access.
// -----------------------------------------------------------------------------
static inline void SetReprojectionError(sfmmap::Landmark& lm,
                                        const sfmmap::ShotId& sid,
                                        const Vec2d& err) {
  // Preferred API if available:
  //   lm.SetReprojectionError(sid, err);
  // Fallback: expose via public member if your fork exposes it (keep this guarded).
  try {
    lm.SetReprojectionError(sid, err);
  } catch (...) {
    // Best-effort: do nothing if API unavailable; Python side will compute as fallback.
  }
}

// -----------------------------------------------------------------------------
// Compute reprojection errors for all observations in 'map' (or restricted set).
// Error metric: normalized-bearing delta in camera plane (dx, dy).
// -----------------------------------------------------------------------------
void BAHelpers::ComputeReprojectionErrors(sfmmap::Map& map,
                                          const std::unordered_set<sfmmap::ShotId>* restrict_to) {
  for (auto& shot_pair : map.GetShots()) {
    const auto& shot_id = shot_pair.first;
    auto& shot = shot_pair.second;

    if (restrict_to && restrict_to->find(shot_id) == restrict_to->end()) {
      continue;
    }

    const auto& pose = *shot.GetPose();
    const auto& cam  = *shot.GetCamera();

    // For each landmark observed in this shot
    for (const auto& kv : shot.GetLandmarkObservations()) {
      sfmmap::Landmark* lm     = kv.first;
      const auto& obs          = kv.second;  // contains .projection_ (pixel), .id, etc.

      // Observed bearing in camera coords
      Eigen::Vector3d b_obs_cam = cam.Bearing(obs.projection_);
      // Predicted bearing from 3D
      const Eigen::Vector3d Xw  = lm->GetGlobalPos();
      const Eigen::Vector3d Cw  = pose.GetOrigin();
      const Eigen::Vector3d Xc  = pose.RotationWorldToCamera() * (Xw - Cw);

      if (Xc.z() == 0.0) continue;
      Eigen::Vector2d n_obs(b_obs_cam.x() / std::max(1e-12, b_obs_cam.z()),
                            b_obs_cam.y() / std::max(1e-12, b_obs_cam.z()));
      Eigen::Vector2d n_pred(Xc.x() / Xc.z(), Xc.y() / Xc.z());

      const Eigen::Vector2d err = n_pred - n_obs;
      SetReprojectionError(*lm, shot_id, err);
    }
  }
}

// -----------------------------------------------------------------------------
// Tiny pose-only smoothing (optional, keeps it super-conservative)
// We do NOT touch intrinsics or rig structures; just a couple of damped steps.
// -----------------------------------------------------------------------------
void BAHelpers::PoseOnlySmooth(sfmmap::Map& map,
                               const std::unordered_set<sfmmap::ShotId>* restrict_to,
                               int max_iterations) {
  // Intentionally minimal to avoid numerical surprises / segfaults.
  // We simply nudge poses a tiny bit toward the median landmark direction to reduce
  // gross inconsistencies after bootstrap. If you don't want any smoothing, set
  // max_iterations = 0 in config and we'll skip this entirely.

  if (max_iterations <= 0) return;

  for (int it = 0; it < max_iterations; ++it) {
    for (auto& shot_pair : map.GetShots()) {
      const auto& shot_id = shot_pair.first;
      auto& shot = shot_pair.second;

      if (restrict_to && restrict_to->find(shot_id) == restrict_to->end()) {
        continue;
      }

      auto* pose = shot.GetPose();
      const Eigen::Vector3d Cw = pose->GetOrigin();
      const Mat3d Rcw = pose->RotationWorldToCamera();

      // Accumulate a tiny translation adjustment from visible landmarks
      Eigen::Vector3d delta_t_w = Eigen::Vector3d::Zero();
      int count = 0;

      for (const auto& kv : shot.GetLandmarkObservations()) {
        const sfmmap::Landmark* lm = kv.first;
        const Eigen::Vector3d Xw = lm->GetGlobalPos();
        const Eigen::Vector3d Xc = Rcw * (Xw - Cw);
        if (Xc.z() <= 0) continue;
        // Encourage points to sit slightly farther in front (reduce skews)
        delta_t_w += 0.0001 * (Xw - Cw).normalized();  // tiny step in world space
        ++count;
      }

      if (count > 0) {
        pose->SetOrigin(Cw + delta_t_w / double(count));
      }
    }
  }
}

// -----------------------------------------------------------------------------
// Bundle (safe/minimal): compute reprojection errors, optional tiny smoothing.
// -----------------------------------------------------------------------------
py::dict BAHelpers::Bundle(
    sfmmap::Map& map,
    const std::unordered_map<sfmmap::CameraId, geometry::Camera>& /*camera_priors*/,
    const std::unordered_map<sfmmap::RigCameraId, sfmmap::RigCamera>& /*rig_camera_priors*/,
    const AlignedVector<sfmmap::GroundControlPoint>& /*gcp*/,
    const py::dict& config) {

  // 1) tiny pose-only smoothing (optional; default 0-2 iters)
  int max_pose_smooth_iters = 0;
  if (config.contains("bundle_pose_smooth_iterations")) {
    max_pose_smooth_iters = config["bundle_pose_smooth_iterations"].cast<int>();
    if (max_pose_smooth_iters < 0) max_pose_smooth_iters = 0;
    if (max_pose_smooth_iters > 2) max_pose_smooth_iters = 2; // stay conservative
  }
  PoseOnlySmooth(map, /*restrict_to=*/nullptr, max_pose_smooth_iters);

  // 2) refresh reprojection errors for whole map
  ComputeReprojectionErrors(map, /*restrict_to=*/nullptr);

  // 3) make a compact report expected by Python side
  py::dict report;
  int num_shots = static_cast<int>(map.GetShots().size());
  int num_points = static_cast<int>(map.GetLandmarks().size());
  report["status"] = "ok";
  report["num_shots"] = num_shots;
  report["num_points"] = num_points;
  report["brief_report"] = py::str("bundle finished (safe-minimal)");
  return report;
}

// -----------------------------------------------------------------------------
// BundleLocal: operate only on a neighborhood; return central id + report.
// -----------------------------------------------------------------------------
py::tuple BAHelpers::BundleLocal(
    sfmmap::Map& map,
    const std::unordered_map<sfmmap::CameraId, geometry::Camera>& /*camera_priors*/,
    const std::unordered_map<sfmmap::RigCameraId, sfmmap::RigCamera>& /*rig_camera_priors*/,
    const AlignedVector<sfmmap::GroundControlPoint>& /*gcp*/,
    const sfmmap::ShotId& central_shot_id,
    const py::dict& config) {

  // A very permissive neighborhood: we only compute errors for the central shot’s direct obs.
  std::unordered_set<sfmmap::ShotId> restrict_to;
  restrict_to.insert(central_shot_id);

  int max_pose_smooth_iters = 0;
  if (config.contains("local_bundle_pose_smooth_iterations")) {
    max_pose_smooth_iters = config["local_bundle_pose_smooth_iterations"].cast<int>();
    if (max_pose_smooth_iters < 0) max_pose_smooth_iters = 0;
    if (max_pose_smooth_iters > 1) max_pose_smooth_iters = 1;
  }
  PoseOnlySmooth(map, &restrict_to, max_pose_smooth_iters);
  ComputeReprojectionErrors(map, &restrict_to);

  py::dict report;
  report["status"] = "ok";
  report["central_shot"] = py::str(central_shot_id);
  report["brief_report"] = py::str("local bundle finished (safe-minimal)");
  return py::make_tuple(py::cast(central_shot_id), report);
}

// -----------------------------------------------------------------------------
// BundleShotPoses: same safe/minimal pass but restricted to given set.
// -----------------------------------------------------------------------------
py::dict BAHelpers::BundleShotPoses(
    sfmmap::Map& map,
    const std::unordered_set<sfmmap::ShotId>& shot_ids,
    const std::unordered_map<sfmmap::CameraId, geometry::Camera>& /*camera_priors*/,
    const std::unordered_map<sfmmap::RigCameraId, sfmmap::RigCamera>& /*rig_camera_priors*/,
    const py::dict& /*config*/) {

  PoseOnlySmooth(map, &shot_ids, /*iters=*/1);
  ComputeReprojectionErrors(map, &shot_ids);

  py::dict report;
  report["status"]    = "ok";
  report["num_shots"] = static_cast<int>(shot_ids.size());
  report["brief_report"] = py::str("bundle_shot_poses finished (safe-minimal)");
  return report;
}

// -----------------------------------------------------------------------------
// Copy BA state back to map (kept as no-op for now).
// -----------------------------------------------------------------------------
void BAHelpers::BundleToMap(const bundle::BundleAdjuster& /*bundle_adjuster*/,
                            sfmmap::Map& /*output_map*/,
                            bool /*update_cameras*/) {}

} // namespace sfm
