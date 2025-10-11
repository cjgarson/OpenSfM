#pragma once
#include "bundle/bundle_adjuster.h"
#include "map/ground_control_points.h"
#include "map/map.h"
#include <pybind11/pybind11.h>

#include <unordered_map>
#include <unordered_set>

namespace py = pybind11;

namespace sfm {

class GroundControlPoint;

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
      const sfmmap::ShotId& central_shot_id, const py::dict& config);

  static py::dict BundleShotPoses(
      sfmmap::Map& map, const std::unordered_set<sfmmap::ShotId>& shot_ids,
      const std::unordered_map<sfmmap::CameraId, geometry::Camera>& camera_priors,
      const std::unordered_map<sfmmap::RigCameraId, sfmmap::RigCamera>& rig_camera_priors,
      const py::dict& config);

  static void BundleToMap(const bundle::BundleAdjuster& bundle_adjuster,
                          sfmmap::Map& output_map, bool update_cameras);

  static std::pair<std::unordered_set<sfmmap::ShotId>,
                   std::unordered_set<sfmmap::ShotId>>
  ShotNeighborhoodIds(sfmmap::Map& map, const sfmmap::ShotId& central_shot_id,
                      size_t radius, size_t min_common_points,
                      size_t max_interior_size);

  static std::pair<std::unordered_set<sfmmap::Shot*>,
                   std::unordered_set<sfmmap::Shot*>>
  ShotNeighborhood(sfmmap::Map& map, const sfmmap::ShotId& central_shot_id,
                   size_t radius, size_t min_common_points,
                   size_t max_interior_size);

  static std::string DetectAlignmentConstraints(
      const sfmmap::Map& map, const py::dict& config,
      const AlignedVector<sfmmap::GroundControlPoint>& gcp);

  static size_t AddGCPToBundle(
      bundle::BundleAdjuster& ba, const sfmmap::Map& map,
      const AlignedVector<sfmmap::GroundControlPoint>& gcp,
      const py::dict& config);

 private:
  static std::unordered_set<sfmmap::Shot*> DirectShotNeighbors(
      sfmmap::Map& map, const std::unordered_set<sfmmap::Shot*>& shot_ids,
      size_t min_common_points, size_t max_neighbors);

  static bool TriangulateGCP(
      const sfmmap::GroundControlPoint& point,
      const std::unordered_map<sfmmap::ShotId, sfmmap::Shot>& shots,
      Vec3d& coordinates);

  static void AlignmentConstraints(
      const sfmmap::Map& map, const py::dict& config,
      const AlignedVector<sfmmap::GroundControlPoint>& gcp, MatX3d& Xp, MatX3d& X);
};

}  // namespace sfm
