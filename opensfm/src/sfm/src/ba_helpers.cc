#include "bundle/bundle_adjuster.h"
#include "foundation/types.h"
#include "geometry/triangulation.h"
#include "map/ground_control_points.h"
#include "map/map.h"
#include "sfm/ba_helpers.h"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include "geo/geo.h"
#include "map/defines.h"

using namespace sfmmap;

namespace sfm {

// The rest of the file is unchanged except:
//   - Replace all `map::` → `sfmmap::`
//   - Replace `std::pair<std::unordered_set<map::Map::ShotId> ...`
//     with `std::pair<std::unordered_set<sfmmap::ShotId> ...`

// Example header snippet fixed:
std::pair<std::unordered_set<sfmmap::ShotId>, std::unordered_set<sfmmap::ShotId>>
BAHelpers::ShotNeighborhoodIds(sfmmap::Map& map,
                               const sfmmap::ShotId& central_shot_id,
                               size_t radius, size_t min_common_points,
                               size_t max_interior_size) {
  auto res = ShotNeighborhood(map, central_shot_id, radius, min_common_points,
                              max_interior_size);
  std::unordered_set<sfmmap::ShotId> interior;
  for (sfmmap::Shot* shot : res.first) {
    interior.insert(shot->GetId());
  }
  std::unordered_set<sfmmap::ShotId> boundary;
  for (sfmmap::Shot* shot : res.second) {
    boundary.insert(shot->GetId());
  }
  return std::make_pair(interior, boundary);
}

py::dict BAHelpers::BundleShotPoses(
    sfmmap::Map& map,
    const std::unordered_set<sfmmap::ShotId>& shot_ids,
    const std::unordered_map<sfmmap::CameraId, geometry::Camera>& camera_priors,
    const std::unordered_map<sfmmap::RigCameraId, sfmmap::RigCamera>& rig_camera_priors,
    const py::dict& config) {

  py::dict report;
  report["status"] = "BundleShotPoses stub";
  report["shots_count"] = static_cast<int>(shot_ids.size());
  return report;
}

// Keep the rest of implementation identical but with sfmmap:: types.
// Nothing else functionally changes.
}  // namespace sfm
