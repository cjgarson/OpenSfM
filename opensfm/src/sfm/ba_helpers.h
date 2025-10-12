// src/sfm/ba_helpers.h
#pragma once

#include <bundle/bundle_adjuster.h>
#include <map/ground_control_points.h>
#include <map/map.h>
#include <pybind11/pybind11.h>

#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <string>

namespace py = pybind11;

namespace sfm {

/**
 * NOTE:
 *  - All map-related types are in namespace `sfmmap`.
 *  - This header matches the static-methods implemented in src/sfm/src/ba_helpers.cc.
 *  - Do not add unrelated APIs here (AlignMaps, MergeMaps, etc.), or builds will break.
 */
class BAHelpers {
 public:
  // Global bundle
  static py::dict Bundle(
      sfmmap::Map& map,
      const sfmmap::Map::CameraMap&    camera_priors,
      const sfmmap::Map::RigCameraMap& rig_camera_priors,
      const AlignedVector<sfmmap::GroundControlPoint>& gcp,
      const py::dict& config);

  // Local bundle around a central shot
  static py::tuple BundleLocal(
      sfmmap::Map& map,
      const sfmmap::Map::CameraMap&    camera_priors,
      const sfmmap::Map::RigCameraMap& rig_camera_priors,
      const AlignedVector<sfmmap::GroundControlPoint>& gcp,
      const sfmmap::ShotId& central_shot_id,
      const py::dict& config);

  // Bundle only poses (shots fixed set)
  static py::dict BundleShotPoses(
      sfmmap::Map& map,
      const std::unordered_set<sfmmap::ShotId>& shot_ids,
      const sfmmap::Map::CameraMap&    camera_priors,
      const sfmmap::Map::RigCameraMap& rig_camera_priors,
      const py::dict& config);

  // Copy BA state back to the map
  static void BundleToMap(const bundle::BundleAdjuster& bundle_adjuster,
                          sfmmap::Map& output_map,
                          bool update_cameras);

  // Neighborhood helpers (IDs and pointers)
  static std::pair<std::unordered_set<sfmmap::ShotId>,
                   std::unordered_set<sfmmap::ShotId>>
  ShotNeighborhoodIds(sfmmap::Map& map,
                      const sfmmap::ShotId& central_shot_id,
                      size_t radius,
                      size_t min_common_points,
                      size_t max_interior_size);

  static std::pair<std::unordered_set<sfmmap::Shot*>,
                   std::unordered_set<sfmmap::Shot*>>
  ShotNeighborhood(sfmmap::Map& map,
                   const sfmmap::ShotId& central_shot_id,
                   size_t radius,
                   size_t min_common_points,
                   size_t max_interior_size);

  static std::unordered_set<sfmmap::Shot*> DirectShotNeighbors(
      sfmmap::Map& map,
      const std::unordered_set<sfmmap::Shot*>& shot_ids,
      size_t min_common_points,
      size_t max_neighbors);

  // Alignment helpers (used by some pipelines)
  static std::string DetectAlignmentConstraints(
      const sfmmap::Map& map,
      const py::dict& config,
      const AlignedVector<sfmmap::GroundControlPoint>& gcp);

  static size_t AddGCPToBundle(
      bundle::BundleAdjuster& ba,
      const sfmmap::Map& map,
      const AlignedVector<sfmmap::GroundControlPoint>& gcp,
      const py::dict& config);

 private:
  // Internal helpers implemented in ba_helpers.cc
  static bool TriangulateGCP(
      const sfmmap::GroundControlPoint& point,
      const sfmmap::Map::ShotMap& shots,
      Vec3d& coordinates);

  static void AlignmentConstraints(
      const sfmmap::Map& map,
      const py::dict& config,
      const AlignedVector<sfmmap::GroundControlPoint>& gcp,
      MatX3d& Xp,
      MatX3d& X);
};

}  // namespace sfm
