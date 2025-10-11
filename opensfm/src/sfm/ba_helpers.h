#pragma once
#include "bundle/bundle_adjuster.h"
#include "map/ground_control_points.h"
#include "map/map.h"
#include "pybind11/pybind11.h"

#include <unordered_map>
#include <unordered_set>

namespace sfm {
namespace ba_helpers {

/** Apply camera and rig camera priors, and ground control points (GCPs) to a map before bundle adjustment. */
void AddPriorInformation(sfmmap::Map& map,
      const std::unordered_map<sfmmap::CameraId, geometry::Camera>& camera_priors,
      const std::unordered_map<sfmmap::RigCameraId, sfmmap::RigCamera>& rig_camera_priors,
      const std::unordered_map<sfmmap::RigCameraId, sfmmap::RigCamera>& rig_cameras,
      const std::unordered_map<sfmmap::GroundControlPoint, sfmmap::GroundControlPoint>& gcp,
      double translation_variance);

/** Remove prior information and GCP constraints from a map after bundle adjustment. */
void RemovePriorInformation(sfmmap::Map& map,
      const std::unordered_map<sfmmap::CameraId, geometry::Camera>& camera_priors,
      const std::unordered_map<sfmmap::RigCameraId, sfmmap::RigCamera>& rig_camera_priors,
      const std::unordered_map<sfmmap::RigCameraId, sfmmap::RigCamera>& rig_cameras,
      const std::unordered_map<sfmmap::GroundControlPoint, sfmmap::GroundControlPoint>& gcp);

/** Get a neighborhood of connected shots around a central shot. */
using ShotNeighborhood = std::unordered_set<sfmmap::Shot*>;  
ShotNeighborhood GetShotNeighborhood(sfmmap::Map& map, const sfmmap::ShotId& central_shot_id,
                                     size_t radius);

/** Align two maps (map and output_map), using a set of common shots (shot_ids) and GCPs. */
void AlignMaps(const sfmmap::Map& map, sfmmap::Map& output_map, bool use_common_landmarks,
               const std::unordered_set<sfmmap::ShotId>& shot_ids,
               const std::unordered_set<sfmmap::GroundControlPoint>& gcp_points);

/** Structure to hold neighboring shot information for merging. */
struct DirectShotNeighbor {
    sfmmap::Shot* shot;
    double score;
};
using DirectShotNeighbors = std::vector<DirectShotNeighbor>;

/** Find direct neighbor shots for merging. */
DirectShotNeighbors ComputeDirectShotNeighbors(sfmmap::Map& map, const sfmmap::ShotId& central_shot_id,
                                               size_t max_neighbors);

/** Merge two maps (map and map_to_merge) given known correspondences between their shots and GCPs. */
void MergeMaps(sfmmap::Map& map, const sfmmap::Map& map_to_merge,
               const std::unordered_map<sfmmap::ShotId, sfmmap::Shot>& shots,
               const std::unordered_map<sfmmap::GroundControlPoint, sfmmap::GroundControlPoint>& gcp_points,
               bool refine_alignment = true);

}  // namespace ba_helpers
}  // namespace sfm
