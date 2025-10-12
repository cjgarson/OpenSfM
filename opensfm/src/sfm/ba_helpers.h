#pragma once

#include <bundle/bundle_adjuster.h>
#include <map/ground_control_points.h>
#include <map/map.h>
#include <pybind11/pybind11.h>

#include <unordered_map>
#include <unordered_set>
#include <string>

namespace py = pybind11;

namespace sfm {

/**
 * Safe-minimal BA helpers:
 *  - Pose-only, conservative smoothing (optional)
 *  - NO direct reprojection-error computation here (Python will refresh)
 */
class BAHelpers {
 public:
  static py::dict Bundle(
      sfmmap::Map& map,
      const std::unordered_map<sfmmap::CameraId, geometry::Camera>& camera_priors,
      const std::unordered_map<sfmmap::RigCameraId, sfmmap::RigCamera>& rig_camera_priors,
      const AlignedVector<sfmmap::GroundControlPoint>& gcp,
      const py::dict& config);

  static py::tuple BundleLocal(
      sfmmap::Map& map,
      const std::unordered_map<sfmmap::CameraId, geometry::Camera>& camera_priors,
      const std::unordered_map<sfmmap::RigCameraId, sfmmap::RigCamera>& rig_camera_priors,
      const AlignedVector<sfmmap::GroundControlPoint>& gcp,
      const sfmmap::ShotId& central_shot_id,
      const py::dict& config);

  static py::dict BundleShotPoses(
      sfmmap::Map& map,
      const std::unordered_set<sfmmap::ShotId>& shot_ids,
      const std::unordered_map<sfmmap::CameraId, geometry::Camera>& camera_priors,
      const std::unordered_map<sfmmap::RigCameraId, sfmmap::RigCamera>& rig_camera_priors,
      const py::dict& config);

  static void BundleToMap(const bundle::BundleAdjuster& bundle_adjuster,
                          sfmmap::Map& output_map,
                          bool update_cameras);

 private:
  // Tiny, conservative pose “smoothing”. Doesn’t touch intrinsics or rigs.
  static void PoseOnlySmooth(sfmmap::Map& map,
                             const std::unordered_set<sfmmap::ShotId>* restrict_to,
                             int max_iterations);
};

}  // namespace sfm
