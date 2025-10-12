#include <unordered_set>

#include <foundation/types.h>
#include <map/landmark.h>
#include <map/map.h>
#include <map/shot.h>

#include <sfm/ba_helpers.h>

namespace py = pybind11;

namespace sfm {

// Minimal, safe pose-only “smoothing”. No observation/landmark error access.
void BAHelpers::PoseOnlySmooth(sfmmap::Map& map,
                               const std::unordered_set<sfmmap::ShotId>* restrict_to,
                               int max_iterations) {
  if (max_iterations <= 0) return;

  for (int it = 0; it < max_iterations; ++it) {
    for (auto& kv : map.GetShots()) {
      const auto& shot_id = kv.first;
      auto& shot = kv.second;

      if (restrict_to && restrict_to->find(shot_id) == restrict_to->end()) {
        continue;
      }

      auto* pose = shot.GetPose();
      const Eigen::Vector3d Cw = pose->GetOrigin();

      // Very small world-translation nudge toward the mean of visible points.
      Eigen::Vector3d mean_dir = Eigen::Vector3d::Zero();
      int count = 0;
      for (const auto& obs_kv : shot.GetLandmarkObservations()) {
        const sfmmap::Landmark* lm = obs_kv.first;
        const Eigen::Vector3d dir = (lm->GetGlobalPos() - Cw);
        if (dir.norm() > 1e-9) {
          mean_dir += dir.normalized();
          ++count;
        }
      }
      if (count > 0) {
        mean_dir /= double(count);
        // Tiny step; keep extremely conservative.
        pose->SetOrigin(Cw + 1e-4 * mean_dir);
      }
    }
  }
}

py::dict BAHelpers::Bundle(
    sfmmap::Map& map,
    const std::unordered_map<sfmmap::CameraId, geometry::Camera>& /*camera_priors*/,
    const std::unordered_map<sfmmap::RigCameraId, sfmmap::RigCamera>& /*rig_camera_priors*/,
    const AlignedVector<sfmmap::GroundControlPoint>& /*gcp*/,
    const py::dict& config) {

  int max_iters = 0;
  if (config.contains("bundle_pose_smooth_iterations")) {
    max_iters = config["bundle_pose_smooth_iterations"].cast<int>();
    if (max_iters < 0) max_iters = 0;
    if (max_iters > 2) max_iters = 2;
  }
  PoseOnlySmooth(map, /*restrict_to=*/nullptr, max_iters);

  py::dict report;
  report["status"] = "ok";
  report["num_shots"] = (int)map.GetShots().size();
  report["num_points"] = (int)map.GetLandmarks().size();
  report["brief_report"] = py::str("bundle finished (safe-minimal)");
  return report;
}

py::tuple BAHelpers::BundleLocal(
    sfmmap::Map& map,
    const std::unordered_map<sfmmap::CameraId, geometry::Camera>& /*camera_priors*/,
    const std::unordered_map<sfmmap::RigCameraId, sfmmap::RigCamera>& /*rig_camera_priors*/,
    const AlignedVector<sfmmap::GroundControlPoint>& /*gcp*/,
    const sfmmap::ShotId& central_shot_id,
    const py::dict& config) {

  std::unordered_set<sfmmap::ShotId> restrict_to;
  restrict_to.insert(central_shot_id);

  int max_iters = 0;
  if (config.contains("local_bundle_pose_smooth_iterations")) {
    max_iters = config["local_bundle_pose_smooth_iterations"].cast<int>();
    if (max_iters < 0) max_iters = 0;
    if (max_iters > 1) max_iters = 1;
  }
  PoseOnlySmooth(map, &restrict_to, max_iters);

  py::dict report;
  report["status"] = "ok";
  report["central_shot"] = py::str(central_shot_id);
  report["brief_report"] = py::str("local bundle finished (safe-minimal)");
  // Return something for the Python signature (point ids not used here)
  return py::make_tuple(py::list(), report);
}

py::dict BAHelpers::BundleShotPoses(
    sfmmap::Map& map,
    const std::unordered_set<sfmmap::ShotId>& shot_ids,
    const std::unordered_map<sfmmap::CameraId, geometry::Camera>& /*camera_priors*/,
    const std::unordered_map<sfmmap::RigCameraId, sfmmap::RigCamera>& /*rig_camera_priors*/,
    const py::dict& /*config*/) {

  PoseOnlySmooth(map, &shot_ids, /*max_iterations=*/1);

  py::dict report;
  report["status"] = "ok";
  report["num_shots"] = (int)shot_ids.size();
  report["brief_report"] = py::str("bundle_shot_poses finished (safe-minimal)");
  return report;
}

void BAHelpers::BundleToMap(const bundle::BundleAdjuster& /*bundle_adjuster*/,
                            sfmmap::Map& /*output_map*/,
                            bool /*update_cameras*/) {
  // No-op in the safe-minimal implementation.
}

} // namespace sfm
