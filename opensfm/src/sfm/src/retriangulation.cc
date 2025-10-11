#include <map/defines.h>
#include <map/tracks_manager.h>
#include <map/map.h>
#include <sfm/retriangulation.h>

#include <limits>
#include <unordered_set>

namespace sfm {
namespace retriangulation {

void RealignMaps(const sfmmap::Map& map_from, sfmmap::Map& map_to,
                 bool update_points) {
  const auto& map_from_shots = map_from.GetShots();

  const auto& from_ref   = map_from.GetTopocentricConverter();
  const auto& to_ref     = map_to.GetTopocentricConverter();
  const auto  from_to_offset = to_ref.ToTopocentric(from_ref.GetLlaRef());

  // first, record transforms that remap points of 'to'
  std::unordered_map<sfmmap::ShotId, geometry::Similarity> from_to_transforms;
  for (const auto& shot_to : map_to.GetShots()) {
    if (!map_from.HasShot(shot_to.first)) {
      continue;
    }
    const auto& shot_from = map_from.GetShot(shot_to.first);
    auto shot_from_pose   = *shot_from.GetPose();
    const auto shot_to_pose = *shot_to.second.GetPose();

    // put 'from' in LLA of 'to'
    shot_from_pose.SetOrigin(shot_from_pose.GetOrigin() + from_to_offset);

    // compute similarity that brings 'to' shot to 'from' shot
    geometry::Similarity sim = geometry::Similarity::FromRigToRig(shot_to_pose, shot_from_pose);
    from_to_transforms[shot_to.first] = sim;
  }

  // apply transforms
  std::unordered_set<sfmmap::ShotId> to_delete;
  for (auto& [shot_id, shot_to] : map_to.GetShots()) {
    if (!map_from.HasShot(shot_id)) {
      to_delete.insert(shot_id);
      continue;
    }
    const auto& sim = from_to_transforms[shot_id];
    // update shot pose
    shot_to.SetPose(sim.TransformPose(*shot_to.GetPose()));
    // update each landmark observed in this shot
    for (auto& lm_obs : shot_to.GetLandmarkObservations()) {
      const sfmmap::LandmarkId& lm_id = lm_obs.first->id_;
      if (!map_from.HasLandmark(lm_id)) {
        continue;
      }
      auto& lm_to = map_to.GetLandmark(lm_id);
      const auto& lm_from = map_from.GetLandmark(lm_id);
      // transfer point
      lm_to.SetGlobalPos(sim.Transform(lm_from.GetGlobalPos()));
    }
  }
  // remove shots and orphaned points that couldn't be aligned
  for (const auto& shot_id : to_delete) {
    map_to.RemoveShot(shot_id);
  }
  // optionally, remove points not seen by any aligned shot
  if (update_points) {
    std::unordered_set<sfmmap::LandmarkId> to_remove;
    for (const auto& lm_pair : map_to.GetLandmarks()) {
      const auto& lm_id = lm_pair.first;
      if (!map_from.HasLandmark(lm_id)) {
        to_remove.insert(lm_id);
      }
    }
    for (const auto& lm_id : to_remove) {
      map_to.RemoveLandmark(lm_id);
    }
  }

  // align biases (if any)
  for (auto& cam_pair : map_to.GetCameras()) {
    const auto& cam_id = cam_pair.first;
    if (map_from.HasBias(cam_id) && map_to.HasBias(cam_id)) {
      // combine the two bias transformations
      geometry::Similarity new_bias = map_from.GetBias(cam_id) * map_to.GetBias(cam_id);
      map_to.SetBias(cam_id, new_bias);
    }
  }
}

}  // namespace retriangulation
}  // namespace sfm
